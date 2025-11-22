# Inferencing V3 - 新模型推論程式

## 📁 資料夾結構

```
inferencing_v3/
├── source/
│   ├── main_batch.cpp    # 讀取整個 recordings 資料夾
│   └── main_latest.cpp   # 只讀取最新時間戳資料夾
├── sdk/                  # Edge Impulse SDK（新模型 833329）
│   ├── edge-impulse-sdk/
│   ├── model-parameters/
│   └── tflite-model/
├── build/                # 編譯輸出
├── output/               # 結果輸出
├── Makefile              # 編譯設定
├── build.sh              # 編譯腳本
└── README.md             # 本檔案
```

## 🚀 快速開始

### 1. 編譯

```bash
cd /home/richlin0308/114/ros2_ws/sound/inference/inferencing_v3
bash build.sh
```

### 2. 使用

#### 方式一：讀取整個 recordings 資料夾（批量處理）

```bash
./build/inference_batch
```

或指定參數：

```bash
./build/inference_batch --recordings-dir ../../recordings --output ../../recordings/inference_results_batch.csv
```

**功能**：
- 遞迴掃描 `recordings` 資料夾內的所有 WAV 檔案
- 處理所有子資料夾內的音訊
- 結果儲存到 `inference_results_batch.csv`

#### 方式二：只讀取最新時間戳資料夾

```bash
./build/inference_latest
```

或指定參數：

```bash
./build/inference_latest --recordings-dir ../../recordings --output ../../recordings/inference_results_latest.csv
```

**功能**：
- 自動找到最新的時間戳資料夾（格式：`YYYYMMDD_HHMMSS`）
- 只處理該資料夾內的 WAV 檔案
- 結果儲存到 `inference_results_latest.csv`

## 📊 CSV 格式

兩個程式都使用相同的 CSV 格式：

```csv
timestamp,filepath,label,confidence,dsp_time_ms,classification_time_ms,total_time_ms
2025-11-21 10:00:00,../../recordings/20241121_100530/test.wav,GOOD,0.8766,15,3,18
2025-11-21 10:00:05,../../recordings/20241121_100530/test2.wav,BAD,0.1234,14,2,16
```

**欄位說明**：
- `timestamp`: 推論時間
- `filepath`: WAV 檔案完整路徑
- `label`: 分類結果（BAD 或 GOOD）
- `confidence`: 信心度（0.0-1.0）
- `dsp_time_ms`: DSP 處理時間（毫秒）
- `classification_time_ms`: 分類時間（毫秒）
- `total_time_ms`: 總處理時間（毫秒）

## ✨ 特點

- ✅ 使用新模型（Project ID: 833329）
- ✅ 直接引用 `sdk/` 資料夾內的參數
- ✅ 自動將每筆結果儲存到 CSV
- ✅ 支援兩種處理模式（批量 / 最新資料夾）
- ✅ 即時寫入 CSV（每處理一個檔案就寫入）

## 🔧 模型資訊

- **Project ID**: 833329
- **專案名稱**: MFE
- **取樣率**: 44100 Hz
- **特徵向量大小**: 4416 維
- **原始樣本數**: 61740 個（約 1.4 秒）
- **標籤數量**: 2 個（BAD, GOOD）
- **模型類型**: float32 TFLite

## 📝 注意事項

1. **音訊長度要求**：模型需要至少 61740 個樣本（約 1.4 秒），音訊太短會自動補零，太長會截斷
2. **取樣率要求**：WAV 檔案必須是 44100 Hz
3. **時間戳資料夾格式**：必須符合 `YYYYMMDD_HHMMSS` 格式（例如：`20241121_100530`）

