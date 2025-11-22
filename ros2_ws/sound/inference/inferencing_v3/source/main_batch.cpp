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
#include <map>
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
    std::string output_csv;  // 如果為空，將根據檔案所在資料夾自動設定
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
    std::regex pattern(R"(^\d{8}_\d{6}$)");
    return std::regex_match(name, pattern);
}

// 從檔案路徑中提取時間戳資料夾路徑，如果檔案在時間戳資料夾內則返回該資料夾路徑，否則返回空字串
static std::string get_timestamp_folder_from_path(const std::string &file_path, const std::string &recordings_dir) {
    // 找到 recordings_dir 之後的第一層目錄
    size_t recordings_pos = file_path.find(recordings_dir);
    if (recordings_pos == std::string::npos) {
        return "";
    }
    
    size_t start_pos = recordings_pos + recordings_dir.length();
    if (start_pos >= file_path.length()) {
        return "";
    }
    
    // 跳過開頭的斜線
    if (file_path[start_pos] == '/') {
        start_pos++;
    }
    
    // 找到第一個斜線（時間戳資料夾的結尾）
    size_t end_pos = file_path.find('/', start_pos);
    if (end_pos == std::string::npos) {
        return "";
    }
    
    std::string folder_name = file_path.substr(start_pos, end_pos - start_pos);
    if (is_timestamp_folder(folder_name)) {
        std::string folder_path = recordings_dir;
        if (recordings_dir.back() != '/') folder_path += "/";
        folder_path += folder_name;
        return folder_path;
    }
    
    return "";
}

// 遞迴掃描目錄，收集所有 WAV 檔案
static void scan_directory_recursive(const std::string &dir_path, std::vector<std::string> &wav_files) {
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
        
        if (S_ISDIR(path_stat.st_mode)) {
            // 遞迴掃描子目錄
            scan_directory_recursive(full_path, wav_files);
        } else if (S_ISREG(path_stat.st_mode) && is_wav_file(name)) {
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
        ei_printf("音訊長度不足，至少需 %u 個樣本，實際只有 %zu，將自動補零\r\n",
            EI_CLASSIFIER_RAW_SAMPLE_COUNT, out_samples.size());
        out_samples.resize(EI_CLASSIFIER_RAW_SAMPLE_COUNT, 0.0f);
    } else if (out_samples.size() > EI_CLASSIFIER_RAW_SAMPLE_COUNT) {
        ei_printf("音訊長度過長，將截斷到 %u 個樣本\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        out_samples.resize(EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    }
    
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
            ei_printf("  --output / -o         輸出 CSV 檔名（預設: 自動儲存在各時間戳資料夾內）\r\n");
            return 0;
        }
    }

    ei_printf("========================================\r\n");
    ei_printf("批量推論處理（讀取整個 recordings 資料夾）\r\n");
    ei_printf("========================================\r\n");
    ei_printf("錄音資料夾: %s\r\n", config.recordings_dir.c_str());
    if (!config.output_csv.empty()) {
        ei_printf("CSV 輸出: %s（統一輸出）\r\n", config.output_csv.c_str());
    } else {
        ei_printf("CSV 輸出: 自動儲存在各時間戳資料夾內\r\n");
    }
    ei_printf("\r\n");

    // 掃描所有 WAV 檔案
    std::vector<std::string> wav_files;
    scan_directory_recursive(config.recordings_dir, wav_files);

    if (wav_files.empty()) {
        ei_printf("未找到任何 WAV 檔案\r\n");
        return 1;
    }

    ei_printf("找到 %zu 個 WAV 檔案，開始處理...\r\n\r\n", wav_files.size());

    std::vector<inference_result_t> results;
    size_t success_count = 0;
    size_t fail_count = 0;

    // 用於追蹤每個時間戳資料夾的 CSV 路徑
    std::map<std::string, std::string> folder_csv_map;
    
    for (size_t i = 0; i < wav_files.size(); i++) {
        std::string input_path = wav_files[i];
        ei_printf("[%zu/%zu] 處理: %s\r\n", i + 1, wav_files.size(), input_path.c_str());

        inference_result_t result = process_single_file(input_path);
        results.push_back(result);

        if (result.success) {
            success_count++;
            ei_printf("  ✓ %s: %s (%.4f)\r\n", result.filename.c_str(), result.label.c_str(), result.confidence);
            
            // 決定 CSV 輸出路徑
            std::string csv_path;
            if (!config.output_csv.empty()) {
                // 如果指定了輸出 CSV，使用指定的路徑
                csv_path = config.output_csv;
            } else {
                // 自動判斷：如果檔案在時間戳資料夾內，使用該資料夾的 CSV
                std::string timestamp_folder = get_timestamp_folder_from_path(input_path, config.recordings_dir);
                if (!timestamp_folder.empty()) {
                    // 檢查是否已經為這個資料夾建立 CSV 路徑
                    if (folder_csv_map.find(timestamp_folder) == folder_csv_map.end()) {
                        folder_csv_map[timestamp_folder] = timestamp_folder + "/inference_results.csv";
                    }
                    csv_path = folder_csv_map[timestamp_folder];
                } else {
                    // 不在時間戳資料夾內，使用預設路徑
                    if (csv_path.empty()) {
                        csv_path = config.recordings_dir;
                        if (config.recordings_dir.back() != '/') csv_path += "/";
                        csv_path += "inference_results_batch.csv";
                    }
                }
            }
            
            // 立即寫入 CSV
            append_result_to_csv(csv_path, result);
        } else {
            fail_count++;
            ei_printf("  ✗ 處理失敗\r\n");
        }
        ei_printf("\r\n");
    }

    ei_printf("========================================\r\n");
    ei_printf("批量處理完成：成功 %zu 個，失敗 %zu 個\r\n", success_count, fail_count);
    if (!config.output_csv.empty()) {
        ei_printf("結果已寫入: %s\r\n", config.output_csv.c_str());
    } else if (!folder_csv_map.empty()) {
        ei_printf("結果已寫入以下資料夾的 CSV：\r\n");
        for (const auto &pair : folder_csv_map) {
            ei_printf("  - %s\r\n", pair.second.c_str());
        }
    }
    ei_printf("========================================\r\n");

    return (fail_count > 0) ? 1 : 0;
}

