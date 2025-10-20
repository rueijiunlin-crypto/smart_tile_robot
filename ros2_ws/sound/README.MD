# 音訊錄製系統安裝指南

## 安裝音訊處理套件

# 方式 A（建議，最省事）
sudo apt install python3-pyaudio

# 若你偏好 pip，先裝開發套件再 pip：
# sudo apt install portaudio19-dev
# pip install pyaudio

## 檢查音訊裝置
arecord -l    # 看得到你的音訊裝置就好

## 檢查 USB 裝置權限
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
# 若你不是 dialout 組：
sudo usermod -aG dialout $USER
# 重新登入（或重開機）後生效

## 啟動系統
source /opt/ros/humble/setup.bash
python3 sound_record_1.py

## 監控錄音狀態
ros2 topic echo /moveknock_locate_node

## 注意事項
- 錄音檔案會自動以日期時間命名
- 錄音資料夾會自動創建
- 系統會自動處理錄音開始/停止