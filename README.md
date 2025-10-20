# 自動化敲擊錄音系統

## 📋 系統概述

本系統整合了視覺辨識、硬體控制、音訊錄製功能，實現自動化磁磚敲擊和錄音。

### 系統組件
- **視覺辨識**: RealSense 相機 + OpenCV 影像處理
- **硬體控制**: ESP32 + 步進馬達 + 限位開關
- **音訊錄製**: PyAudio 即時錄音
- **通訊**: ROS2 + micro-ROS

## 🏗️ 系統架構

```
LattePanda 3 (Linux)
├── 視覺辨識節點 (realsence_auto_hit.py)
├── 音訊錄製節點 (sound_record_1.py)
├── micro-ROS Agent
└── ESP32 硬體控制 (bothit_keyboard.ino)
```

## 📁 目錄結構

```
114/
├── ros2_ws/                    # ROS2 工作空間
│   ├── realsence/             # 視覺辨識
│   │   ├── realsence_auto_hit.py
│   │   ├── realsence.py
│   │   └── 修正指南.md
│   └── sound/                 # 音訊錄製
│       ├── sound_record_1.py
│       ├── recordings/        # 錄音檔案目錄
│       └── README.MD
├── hit/                       # ESP32 控制
│   ├── bothit_keyboard.ino
│   ├── keyboard_sender_loop.py
│   └── README.MD
├── start_system.sh            # 系統啟動腳本
├── logs/                      # 系統日誌
└── README.md                  # 本檔案
```

## 🚀 快速開始

### 1. 環境準備
```bash
# 安裝 ROS2 Humble
sudo apt install ros-humble-desktop

# 安裝音訊處理套件
sudo apt install python3-pyaudio

# 設定 USB 權限
sudo usermod -a -G dialout $USER
# 重新登入後生效
```

### 2. 啟動系統
```bash
chmod +x start_system.sh
./start_system.sh
```

### 3. 監控系統
```bash
# 監控系統狀態和位置
ros2 topic echo /moveknock_locate_node

# 監控視覺座標
ros2 topic echo /World_Coordinates

# 監控視覺狀態
ros2 topic echo /vision_status
```

### 4. 錄音檔案位置
```bash
# 錄音檔案儲存在 ros2_ws/sound/recordings/ 目錄下
ls -la ros2_ws/sound/recordings/

# 查看最新錄音資料夾
ls -la ros2_ws/sound/recordings/ | tail -5

# 播放錄音檔案
aplay ros2_ws/sound/recordings/20241201_143022/tile_1_left_001.wav
```

## 🔧 硬體配置

### ESP32 接線
- **X軸步進馬達**: GPIO 16 (STEP), GPIO 17 (DIR), GPIO 18 (EN)
- **Y軸步進馬達**: GPIO 26 (STEP), GPIO 25 (DIR), GPIO 27 (EN)
- **限位開關**: GPIO 33 (X軸右限), GPIO 32 (Y軸下限)
- **敲擊馬達**: GPIO 21, GPIO 22
- **敲擊限位**: GPIO 13

### 座標系統
- **X軸**: 0.0mm (最左) → 350.0mm (最右)
- **Y軸**: 0.0mm (最下) → 140.0mm (最上)
- **初始化**: 通電後自動向右向下移動到限位開關，然後退回5mm

## 📊 系統參數

### 視覺辨識
- **解析度**: 640×480 (優化後)
- **座標精度**: X軸 0.547mm/pixel, Y軸 0.292mm/pixel
- **偵測參數**: 面積 6000-250000, 矩形度檢查已關閉

### 硬體控制
- **步進馬達**: 200步/圈, 1/16細分 (3200步/圈)
- **移動速度**: 可調 (100-2000μs)
- **限位保護**: 硬體限位開關
- **移動範圍**: X軸 0.0-350.0mm, Y軸 0.0-140.0mm (軟體範圍檢查)

### 音訊錄製
- **取樣率**: 44100 Hz
- **格式**: WAV 16-bit
- **自動命名**: 
  - 敲擊序列: `tile_1_left.wav`, `tile_1_mid.wav`, `tile_1_right.wav`
  - 單次敲擊: `00001.wav`, `00002.wav`, `00003.wav`
- **儲存位置**: `ros2_ws/sound/recordings/YYYYMMDD_HHMMSS/`

## 🎮 控制指令

### 鍵盤控制
| 指令 | 功能 | 說明 |
|------|------|------|
| `w` | Y軸向上 10mm | 向上移動 |
| `s` | Y軸向下 10mm | 向下移動 |
| `a` | X軸向左 10mm | 向左移動 |
| `d` | X軸向右 10mm | 向右移動 |
| `h` | 單次敲擊 | 敲擊機構動作 |
| `r` | 回到原點 | 移動到 (0,0) |
| `p` | 查詢位置 | 回報當前座標 |
| `e` | 查詢錯誤 | 回報最後錯誤的詳細資訊 |
| `i` | 重新初始化 | 重新 homing |
| `c` | 清除錯誤狀態 | 從 ERROR 狀態回到 READY |
| `t` | 開始敲擊序列 | 啟動自動敲擊 |
| `n` | 下一個敲擊位置 | 序列中下一個 |
| `q` | 結束序列 | 結束敲擊序列，進入下一個磁磚 |
| `x` | 緊急停止 | 立即停止所有動作 |
| `+` | 加速移動 | 減少延遲時間 |
| `-` | 減速移動 | 增加延遲時間 |

### ROS2 指令
```bash
# 移動指令
ros2 topic pub /keyboard_control std_msgs/String "data: 'w'"

# 敲擊指令
ros2 topic pub /keyboard_control std_msgs/String "data: 'h'"

# 查詢位置
ros2 topic pub /keyboard_control std_msgs/String "data: 'p'"

# 查詢錯誤
ros2 topic pub /keyboard_control std_msgs/String "data: 'e'"

# 清除錯誤狀態
ros2 topic pub /keyboard_control std_msgs/String "data: 'c'"
```

## 📡 系統回報

### 狀態回報
系統每5秒回報一次狀態和位置：
```
status:ready, pos:X:345.00,Y:5.00
status:moving, pos:X:335.00,Y:5.00
status:hitting, pos:X:335.00,Y:5.00
status:error, pos:X:335.00,Y:5.00
```

### 位置回報
- **定期回報**: 每5秒包含在狀態中
- **移動完成**: 立即回報新位置
- **手動查詢**: 按 `p` 鍵立即回報

### 錄音控制
```
start_recording:tile_1:left
stop_recording
```

## 🚨 錯誤回報系統

### 錯誤分類

#### A. Homing 錯誤
- **HOMING_TIMEOUT**: 單軸 homing 超時 (30秒)
- **HOMING_MAX_STEPS**: 單軸 homing 步數超限 (50,000步)
- **SIMULTANEOUS_HOMING_TIMEOUT**: 同時 homing 超時
- **SIMULTANEOUS_HOMING_MAX_STEPS**: 同時 homing 步數超限

#### B. 狀態機錯誤
- **STATE_TRANSITION_BLOCKED**: 狀態轉換被鎖定
- **INVALID_STATE_TRANSITION**: 無效的狀態轉換
- **STATE_TIMEOUT**: 狀態轉換超時 (5秒)

#### C. 系統錯誤
- **MEMORY_ALLOCATION_FAILED**: 記憶體分配失敗
- **HIT_TIMEOUT**: 敲擊超時 (5秒)

### 錯誤回報格式

#### 序列埠日誌
```
ERROR[1234567]: HOMING_TIMEOUT - Axis: X, Steps: 25000, Timeout: 30000ms
ERROR[1234568]: STATE_TIMEOUT - State timeout! Current: MOVING, Timeout: 5000ms
ERROR[1234569]: HIT_TIMEOUT - Hit timeout after 6000ms, Timeout limit: 5000ms, Limit count: 2
```

#### ROS2 話題回報
```
error:HOMING_TIMEOUT:Axis: X, Steps: 25000, Timeout: 30000ms
error:STATE_TIMEOUT:State timeout! Current: MOVING, Timeout: 5000ms
error:HIT_TIMEOUT:Hit timeout after 6000ms, Timeout limit: 5000ms, Limit count: 2
```

### 錯誤查詢
```bash
# 查詢最後錯誤
ros2 topic pub /keyboard_control std_msgs/String "data: 'e'"

# 回應範例
data: "Last error: ERROR[1234567]: HOMING_TIMEOUT - Axis: X, Steps: 25000, Timeout: 30000ms (Time: 5000ms ago)"
```

## 🔍 故障排除

### 常見問題

#### 1. ESP32 連接失敗
- **檢查 USB 連接**: 確認 ESP32 正確連接
- **確認權限設定**: `sudo usermod -a -G dialout $USER`
- **檢查 micro-ROS agent**: 確認 agent 正在運行
- **重啟系統**: 重新執行 `start_system.sh`

#### 2. Homing 失敗
- **檢查限位開關**: 確認限位開關正常觸發
- **檢查馬達接線**: 確認步進馬達接線正確
- **檢查電源**: 確認馬達驅動器有足夠電源
- **手動復位**: 使用 `i` 指令重新 homing
- **清除錯誤**: 使用 `c` 指令清除錯誤狀態

#### 3. 移動異常
- **檢查座標**: 使用 `p` 指令查詢當前位置
- **檢查狀態**: 確認系統在 `ready` 狀態
- **檢查錯誤**: 使用 `e` 指令查詢最後錯誤
- **緊急停止**: 使用 `x` 指令立即停止

#### 4. 視覺辨識失敗
- **檢查相機連接**: 確認 RealSense 相機正常
- **調整光照條件**: 確保充足且均勻的光照
- **檢查參數設定**: 參考 `修正指南.md`
- **重啟視覺節點**: 重新執行視覺程式

#### 5. 音訊錄製失敗
- **檢查音訊裝置**: `arecord -l` 列出音訊裝置
- **確認權限設定**: 檢查音訊裝置權限
- **檢查錄音目錄**: 確認 `recordings` 目錄存在
- **檢查磁碟空間**: 確認有足夠儲存空間

### 日誌檔案
- **系統日誌**: `logs/system_YYYYMMDD_HHMMSS.log`
- **錄音檔案**: `ros2_ws/sound/recordings/YYYYMMDD_HHMMSS/`
- **序列埠日誌**: ESP32 序列埠輸出

### 監控指令
```bash
# 監控系統狀態
ros2 topic echo /moveknock_locate_node

# 監控錯誤
ros2 topic echo /moveknock_locate_node | grep "error:"

# 查看系統日誌
tail -f logs/system_*.log

# 檢查系統負載
htop
```

## 📈 效能監控

### 系統負載目標
- **CPU使用率**: <55% (為 ORB-SLAM 預留)
- **記憶體使用**: <550MB
- **處理頻率**: 30fps 視覺處理
- **回報頻率**: 5秒間隔狀態回報

### 監控指令
```bash
# CPU 使用率
top -p [PID列表]

# 記憶體使用
ps -p [PID列表] -o pid,ppid,cmd,%mem,%cpu

# 系統負載
uptime

# 網路連接
ss -tuln

# 即時監控
watch -n 1 'ps aux | grep python'
```

## 🔮 未來擴展

### ORB-SLAM 整合
- 預留 45% CPU 資源
- 預留 7GB 記憶體
- 即時定位與建圖
- 高精度視覺定位

### 系統優化
- 多線程處理
- 硬體加速
- 參數自動調整
- 機器學習優化

### 功能擴展
- 多相機支援
- 3D 重建
- 自動品質檢測
- 遠端監控

## 📞 技術支援

### 相關檔案
- **詳細修正指南**: `ros2_ws/realsence/修正指南.md`
- **音訊系統說明**: `ros2_ws/sound/README.MD`
- **ESP32 控制說明**: `hit/README.MD`
- **鍵盤控制說明**: `hit/keyboard_sender_loop.py`

### 系統要求
- **作業系統**: Linux (Ubuntu 22.04+)
- **硬體**: LattePanda 3 Delta
- **相機**: Intel RealSense D435i
- **控制器**: ESP32
- **軟體**: ROS2 Humble

### 支援資訊
- **錯誤回報**: 使用 `e` 指令查詢詳細錯誤
- **狀態監控**: 使用 `p` 指令查詢當前位置
- **系統重啟**: 使用 `i` 指令重新初始化
- **錯誤恢復**: 使用 `c` 指令清除錯誤狀態
- **緊急停止**: 使用 `x` 指令立即停止

---
*最後更新: 2024年12月*