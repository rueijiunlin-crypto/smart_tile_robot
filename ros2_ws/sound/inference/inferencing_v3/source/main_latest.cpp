#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>
#include <set>
#include <unordered_set>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <limits.h>
#include <ctime>
#include <sstream>
#include <algorithm>
#include <regex>

#include "sdk/edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "sdk/edge-impulse-sdk/dsp/numpy_types.h"
#include "sdk/edge-impulse-sdk/dsp/returntypes.hpp"

namespace {

struct app_config_t {
    std::string recordings_dir = "../../recordings";
    std::string output_csv;  // 將自動設定為最新時間戳資料夾內的 CSV
};

static std::vector<float> g_audio_samples;

struct inference_result_t {
    std::string filename;
    std::string filepath;
    std::string label;
    float confidence;
    int dsp_time_ms;
    int classification_time_ms;
    bool success;
};

// 獲取當前時間字串
std::string get_current_time() {
    std::time_t now = std::time(nullptr);
    std::tm* timeinfo = std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

static bool is_wav_file(const std::string &path) {
    if (path.length() < 4) return false;
    std::string ext = path.substr(path.length() - 4);
    for (char &c : ext) c = tolower(c);
    return ext == ".wav";
}

// 檢查是否為時間戳格式的資料夾名稱（YYYYMMDD_HHMMSS）
static bool is_timestamp_folder(const std::string &name) {
    // 格式：YYYYMMDD_HHMMSS（例如：20241121_100530）
    std::regex pattern(R"(^\d{8}_\d{6}$)");
    return std::regex_match(name, pattern);
}

// 找到最新的時間戳資料夾
static std::string find_latest_timestamp_folder(const std::string &recordings_dir) {
    DIR *dir = opendir(recordings_dir.c_str());
    if (!dir) {
        ei_printf("無法開啟錄音資料夾: %s\r\n", recordings_dir.c_str());
        return "";
    }
    
    std::vector<std::string> timestamp_folders;
    struct dirent *entry;
    
    while ((entry = readdir(dir)) != nullptr) {
        std::string name(entry->d_name);
        if (name == "." || name == "..") {
            continue;
        }
        
        std::string full_path = recordings_dir;
        if (recordings_dir.back() != '/') full_path += "/";
        full_path += name;
        
        struct stat path_stat;
        if (stat(full_path.c_str(), &path_stat) != 0) {
            continue;
        }
        
        if (S_ISDIR(path_stat.st_mode) && is_timestamp_folder(name)) {
            timestamp_folders.push_back(name);
        }
    }
    closedir(dir);
    
    if (timestamp_folders.empty()) {
        ei_printf("未找到任何時間戳資料夾\r\n");
        return "";
    }
    
    // 按名稱排序（時間戳格式可以直接按字串排序）
    std::sort(timestamp_folders.begin(), timestamp_folders.end());
    
    // 返回最新的（最後一個）
    std::string latest = timestamp_folders.back();
    std::string latest_path = recordings_dir;
    if (recordings_dir.back() != '/') latest_path += "/";
    latest_path += latest;
    
    ei_printf("找到最新時間戳資料夾: %s\r\n", latest.c_str());
    return latest_path;
}

// 掃描目錄，收集所有 WAV 檔案（不遞迴）
static void scan_directory(const std::string &dir_path, std::vector<std::string> &wav_files) {
    DIR *dir = opendir(dir_path.c_str());
    if (!dir) {
        return;
    }
    
    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name(entry->d_name);
        if (name == "." || name == "..") {
            continue;
        }
        
        std::string full_path = dir_path;
        if (dir_path.back() != '/') full_path += "/";
        full_path += name;
        
        struct stat path_stat;
        if (stat(full_path.c_str(), &path_stat) != 0) {
            continue;
        }
        
        if (S_ISREG(path_stat.st_mode) && is_wav_file(name)) {
            wav_files.push_back(full_path);
        }
    }
    closedir(dir);
}

static bool read_bytes(std::ifstream &stream, void *dest, size_t length) {
    stream.read(reinterpret_cast<char*>(dest), static_cast<std::streamsize>(length));
    return static_cast<size_t>(stream.gcount()) == length;
}

static bool load_wav_file(const std::string &path, std::vector<float> &out_samples) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        ei_printf("無法開啟 WAV 檔案: %s\r\n", path.c_str());
        return false;
    }

    char riff_header[4];
    if (!read_bytes(file, riff_header, 4) || strncmp(riff_header, "RIFF", 4) != 0) {
        ei_printf("檔案不是 RIFF 格式\r\n");
        return false;
    }

    uint32_t riff_size = 0;
    if (!read_bytes(file, &riff_size, 4)) {
        ei_printf("無法讀取 RIFF chunk size\r\n");
        return false;
    }

    char wave_header[4];
    if (!read_bytes(file, wave_header, 4) || strncmp(wave_header, "WAVE", 4) != 0) {
        ei_printf("檔案不是 WAVE 格式\r\n");
        return false;
    }

    bool fmt_found = false;
    bool data_found = false;
    uint16_t audio_format = 0;
    uint16_t num_channels = 0;
    uint32_t sample_rate = 0;
    uint16_t bits_per_sample = 0;
    std::vector<int16_t> pcm_samples;

    while (file && !(fmt_found && data_found)) {
        char chunk_id[4];
        if (!read_bytes(file, chunk_id, 4)) {
            break;
        }

        uint32_t chunk_size = 0;
        if (!read_bytes(file, &chunk_size, 4)) {
            break;
        }

        if (strncmp(chunk_id, "fmt ", 4) == 0) {
            fmt_found = true;
            if (chunk_size < 16) {
                ei_printf("fmt chunk 過短\r\n");
                return false;
            }

            if (!read_bytes(file, &audio_format, 2) ||
                !read_bytes(file, &num_channels, 2) ||
                !read_bytes(file, &sample_rate, 4)) {
                ei_printf("讀取 fmt chunk 失敗\r\n");
                return false;
            }

            uint32_t byte_rate = 0;
            uint16_t block_align = 0;
            if (!read_bytes(file, &byte_rate, 4) ||
                !read_bytes(file, &block_align, 2) ||
                !read_bytes(file, &bits_per_sample, 2)) {
                ei_printf("讀取 fmt chunk 失敗\r\n");
                return false;
            }

            if (chunk_size > 16) {
                file.seekg(chunk_size - 16, std::ios::cur);
            }
        }
        else if (strncmp(chunk_id, "data", 4) == 0) {
            if (!fmt_found) {
                ei_printf("讀到 data chunk 前尚未遇到 fmt chunk\r\n");
                return false;
            }
            data_found = true;
            if (bits_per_sample != 16) {
                ei_printf("目前僅支援 16-bit PCM WAV\r\n");
                return false;
            }

            size_t sample_count = chunk_size / sizeof(int16_t);
            pcm_samples.resize(sample_count);
            if (!read_bytes(file, pcm_samples.data(), chunk_size)) {
                ei_printf("讀取 PCM 資料失敗\r\n");
                return false;
            }
        }
        else {
            file.seekg(chunk_size, std::ios::cur);
        }

        if (chunk_size & 1) {
            file.seekg(1, std::ios::cur);
        }
    }

    if (!fmt_found || !data_found) {
        ei_printf("缺少必要 chunk (fmt 或 data)\r\n");
        return false;
    }

    if (audio_format != 1) {
        ei_printf("僅支援 PCM (audio_format=1)，目前為 %u\r\n", audio_format);
        return false;
    }

    if (sample_rate != static_cast<uint32_t>(EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("WAV 取樣率必須是 %d Hz，檔案為 %u Hz\r\n",
            (int)EI_CLASSIFIER_FREQUENCY, sample_rate);
        return false;
    }

    if (num_channels < 1 || num_channels > 2) {
        ei_printf("僅支援 1 或 2 聲道，檔案為 %u 聲道\r\n", num_channels);
        return false;
    }

    if (pcm_samples.empty()) {
        ei_printf("WAV 檔沒有資料\r\n");
        return false;
    }

    out_samples.resize(pcm_samples.size() / num_channels);
    size_t sample_index = 0;
    for (size_t i = 0; i < out_samples.size(); i++) {
        int32_t accum = 0;
        for (uint16_t ch = 0; ch < num_channels; ch++) {
            accum += pcm_samples[sample_index++];
        }
        float averaged = static_cast<float>(accum) / static_cast<float>(num_channels);
        out_samples[i] = averaged;
    }

    if (out_samples.size() < EI_CLASSIFIER_RAW_SAMPLE_COUNT) {
        ei_printf("音訊長度不足，至少需 %u 個樣本，實際只有 %zu\r\n",
            EI_CLASSIFIER_RAW_SAMPLE_COUNT, out_samples.size());
        return false;
    }

    out_samples.resize(EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    return true;
}

static int audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    if ((offset + length) > g_audio_samples.size()) {
        return EIDSP_PARAMETER_INVALID;
    }

    memcpy(out_ptr, g_audio_samples.data() + offset, length * sizeof(float));
    return EIDSP_OK;
}

static bool ensure_directory_exists(const std::string &path) {
    size_t last_slash = path.find_last_of("/\\");
    if (last_slash == std::string::npos) {
        return true;
    }
    
    std::string dir = path.substr(0, last_slash);
    struct stat info;
    if (stat(dir.c_str(), &info) != 0) {
        size_t prev_slash = dir.find_last_of("/\\");
        if (prev_slash != std::string::npos) {
            std::string parent = dir.substr(0, prev_slash);
            if (!ensure_directory_exists(parent + "/dummy")) {
                return false;
            }
        }
        if (mkdir(dir.c_str(), 0755) != 0) {
            return false;
        }
    } else if (!(info.st_mode & S_IFDIR)) {
        return false;
    }
    return true;
}

// 讀取 CSV 檔案中已處理的檔案路徑
static std::unordered_set<std::string> read_processed_files(const std::string &csv_path) {
    std::unordered_set<std::string> processed;
    std::ifstream in(csv_path);
    if (!in.is_open()) {
        return processed;  // 檔案不存在，返回空集合
    }
    
    std::string line;
    bool first_line = true;
    while (std::getline(in, line)) {
        if (first_line) {
            first_line = false;
            continue;  // 跳過標題行
        }
        
        // 解析 CSV：找到第二個欄位（filepath）
        size_t first_comma = line.find(',');
        if (first_comma == std::string::npos) continue;
        
        size_t second_comma = line.find(',', first_comma + 1);
        if (second_comma == std::string::npos) continue;
        
        std::string filepath = line.substr(first_comma + 1, second_comma - first_comma - 1);
        processed.insert(filepath);
    }
    in.close();
    return processed;
}

static bool append_result_to_csv(const std::string &path, const inference_result_t &result) {
    if (!ensure_directory_exists(path)) {
        ei_printf("無法創建輸出目錄: %s\r\n", path.c_str());
        return false;
    }
    
    bool file_exists = std::ifstream(path).good();
    std::ofstream out(path, std::ios::app);
    if (!out.is_open()) {
        out.open(path, std::ios::out);
        if (!out.is_open()) {
            ei_printf("無法寫入結果檔案: %s\r\n", path.c_str());
            return false;
        }
    }
    
    // 寫入標題（如果檔案不存在）
    if (!file_exists) {
        out << "timestamp,filepath,label,confidence,dsp_time_ms,classification_time_ms,total_time_ms\r\n";
    }
    
    // 追加結果
    out << get_current_time() << ",";
    out << result.filepath << ",";
    if (result.success && !result.label.empty()) {
        out << result.label << "," 
            << std::fixed << std::setprecision(4) << result.confidence << ","
            << result.dsp_time_ms << ","
            << result.classification_time_ms << ","
            << (result.dsp_time_ms + result.classification_time_ms);
    } else {
        out << "ERROR,0.0000,0,0,0";
    }
    out << "\r\n";
    
    return true;
}

static inference_result_t process_single_file(const std::string &input_path) {
    inference_result_t result;
    result.filepath = input_path;
    size_t last_slash = input_path.find_last_of("/\\");
    if (last_slash != std::string::npos) {
        result.filename = input_path.substr(last_slash + 1);
    } else {
        result.filename = input_path;
    }
    result.success = false;
    result.label = "";
    result.confidence = 0.0f;
    result.dsp_time_ms = 0;
    result.classification_time_ms = 0;

    std::vector<float> audio_samples;
    if (!load_wav_file(input_path, audio_samples)) {
        return result;
    }

    g_audio_samples = audio_samples;

    signal_t signal;
    signal.total_length = g_audio_samples.size();
    signal.get_data = &audio_signal_get_data;

    std::vector<float> feature_buffer(EI_CLASSIFIER_NN_INPUT_FRAME_SIZE);
    ei::matrix_t feature_matrix(1, EI_CLASSIFIER_NN_INPUT_FRAME_SIZE, feature_buffer.data());

    run_classifier_init();

    int dsp_res = extract_mfe_features(&signal, &feature_matrix, (void*)&ei_dsp_config_833329_3, EI_CLASSIFIER_FREQUENCY);
    if (dsp_res != EIDSP_OK) {
        ei_printf("extract_mfe_features 失敗，錯誤碼 %d\r\n", dsp_res);
        return result;
    }

    ei_impulse_result_t inference_result;
    EI_IMPULSE_ERROR res = run_classifier(&signal, &inference_result, false);
    if (res != EI_IMPULSE_OK) {
        ei_printf("run_classifier 失敗，錯誤碼 %d\r\n", res);
        return result;
    }

    result.dsp_time_ms = inference_result.timing.dsp;
    result.classification_time_ms = inference_result.timing.classification;

    uint16_t best_idx = 0;
    float best_value = inference_result.classification[0].value;
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (inference_result.classification[i].value > best_value) {
            best_value = inference_result.classification[i].value;
            best_idx = i;
        }
    }
    result.label = ei_classifier_inferencing_categories[best_idx];
    result.confidence = best_value;
    result.success = true;

    return result;
}

} // namespace

int main(int argc, char **argv) {
    app_config_t config;
    
    // 解析命令列參數
    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if ((arg == "--recordings-dir" || arg == "-d") && (i + 1) < argc) {
            config.recordings_dir = argv[++i];
        } else if ((arg == "--output" || arg == "-o") && (i + 1) < argc) {
            config.output_csv = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            ei_printf("用法: %s [選項]\r\n", argv[0]);
            ei_printf("選項:\r\n");
            ei_printf("  --recordings-dir / -d  錄音資料夾路徑（預設: ../../recordings）\r\n");
            ei_printf("  --output / -o         輸出 CSV 檔名（預設: 自動儲存在最新時間戳資料夾內）\r\n");
            ei_printf("\r\n");
            ei_printf("說明：此程式會自動找到最新的時間戳資料夾（格式：YYYYMMDD_HHMMSS）\r\n");
            ei_printf("      並只處理該資料夾內的 WAV 檔案\r\n");
            ei_printf("      CSV 結果會自動儲存在該時間戳資料夾內（inference_results.csv）\r\n");
            return 0;
        }
    }

    ei_printf("========================================\r\n");
    ei_printf("推論處理（只讀取最新時間戳資料夾）\r\n");
    ei_printf("========================================\r\n");
    ei_printf("錄音資料夾: %s\r\n", config.recordings_dir.c_str());
    ei_printf("\r\n");

    // 找到最新的時間戳資料夾
    std::string latest_folder = find_latest_timestamp_folder(config.recordings_dir);
    if (latest_folder.empty()) {
        return 1;
    }

    // 自動設定 CSV 輸出路徑為最新時間戳資料夾內
    if (config.output_csv.empty()) {
        config.output_csv = latest_folder + "/inference_results.csv";
    }
    ei_printf("CSV 輸出: %s\r\n", config.output_csv.c_str());
    ei_printf("\r\n");

    // 讀取已處理的檔案列表（避免重複推論）
    std::unordered_set<std::string> processed_files = read_processed_files(config.output_csv);
    ei_printf("已處理檔案數: %zu\r\n", processed_files.size());
    ei_printf("\r\n");

    // 掃描該資料夾內的 WAV 檔案
    std::vector<std::string> wav_files;
    scan_directory(latest_folder, wav_files);

    if (wav_files.empty()) {
        ei_printf("最新時間戳資料夾內未找到任何 WAV 檔案\r\n");
        return 1;
    }

    // 過濾出尚未處理的檔案
    std::vector<std::string> unprocessed_files;
    for (const auto &file : wav_files) {
        if (processed_files.find(file) == processed_files.end()) {
            unprocessed_files.push_back(file);
        }
    }

    if (unprocessed_files.empty()) {
        ei_printf("所有 WAV 檔案都已經處理過，無需重複推論\r\n");
        return 0;
    }

    ei_printf("找到 %zu 個 WAV 檔案，其中 %zu 個尚未處理，開始處理...\r\n\r\n", 
              wav_files.size(), unprocessed_files.size());

    std::vector<inference_result_t> results;
    size_t success_count = 0;
    size_t fail_count = 0;

    for (size_t i = 0; i < unprocessed_files.size(); i++) {
        std::string input_path = unprocessed_files[i];
        ei_printf("[%zu/%zu] 處理: %s\r\n", i + 1, unprocessed_files.size(), input_path.c_str());

        inference_result_t result = process_single_file(input_path);
        results.push_back(result);

        if (result.success) {
            success_count++;
            ei_printf("  ✓ %s: %s (%.4f)\r\n", result.filename.c_str(), result.label.c_str(), result.confidence);
            
            // 立即寫入 CSV
            append_result_to_csv(config.output_csv, result);
        } else {
            fail_count++;
            ei_printf("  ✗ 處理失敗\r\n");
        }
        ei_printf("\r\n");
    }

    ei_printf("========================================\r\n");
    ei_printf("處理完成：成功 %zu 個，失敗 %zu 個\r\n", success_count, fail_count);
    ei_printf("結果已寫入: %s\r\n", config.output_csv.c_str());
    ei_printf("========================================\r\n");

    return (fail_count > 0) ? 1 : 0;
}

