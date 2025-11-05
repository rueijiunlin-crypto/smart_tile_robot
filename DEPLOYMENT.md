# 🚀 部署指南：在另一台電腦上啟動系統

## 📋 前置需求

### 1. 系統環境
- **作業系統**: Ubuntu 20.04 / 22.04 或相容的 Linux 發行版
- **Python**: Python 3.8 或以上
- **ROS2**: ROS2 Humble 或兼容版本

### 2. 硬體需求
- RealSense 相機（用於視覺辨識）
- 音訊輸入裝置（麥克風）
- ESP32 開發板（硬體控制，透過 USB 連接）
- USB 序列埠權限（用戶需在 `dialout` 群組）

---

## 📦 安裝步驟

### 步驟 1: 安裝 ROS2

```bash
# Ubuntu 22.04 (Jammy)
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-humble-desktop

# 設定環境變數（加入 ~/.bashrc）
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 步驟 2: 安裝 ROS2 開發工具

```bash
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential

sudo rosdep init
rosdep update
```

### 步驟 3: 安裝 Python 依賴

```bash
# 確保 pip 是最新的
python3 -m pip install --upgrade pip

# 安裝 Python 套件（注意 NumPy 版本）
pip3 install --user -r requirements.txt

# 如果缺少某些系統依賴，可能需要安裝：
sudo apt install -y \
    portaudio19-dev \
    python3-dev \
    libasound2-dev \
    libopencv-dev
```

### 步驟 4: 安裝 TensorFlow Lite Runtime

```bash
# 根據你的 CPU 架構選擇（x86_64 範例）
python3 -m pip install --user https://github.com/google-coral/pycoral/releases/download/v2.0.0/tflite_runtime-2.5.0-cp310-cp310-linux_x86_64.whl

# 或使用官方版本（如果可用）
# python3 -m pip install --user tflite-runtime
```

### 步驟 5: 複製專案檔案

```bash
# 在新電腦上建立專案目錄
mkdir -p ~/114
cd ~/114

# 從原電腦複製以下目錄和檔案：
# - ros2_ws/
# - start_system.sh
# - requirements.txt
# - README.md
# - DEPLOYMENT.md

# 可以使用 scp、rsync 或 git（如果專案在 git 中）
# 例如：
# scp -r user@原電腦IP:/home/richlin0308/114/ros2_ws ~/114/
# scp user@原電腦IP:/home/richlin0308/114/start_system.sh ~/114/
```

### 步驟 6: 設定 USB 裝置權限

```bash
# 將用戶加入 dialout 群組（允許存取 USB 序列埠）
sudo usermod -a -G dialout $USER

# 登出後重新登入，或執行：
newgrp dialout

# 檢查 USB 裝置
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "尚未連接 USB 裝置"
```

### 步驟 7: 建立 ROS2 工作空間結構

```bash
cd ~/114/ros2_ws

# 建立 package.xml 和 setup.py（如果沒有）
# 視覺辨識套件
mkdir -p realsence/realsence
# 音訊套件
mkdir -p sound/sound

# 建立 setup.py（如果沒有）
# 或使用 colcon 自動建立
```

### 步驟 8: 編譯 ROS2 工作空間

```bash
cd ~/114/ros2_ws

# 安裝依賴
rosdep install --from-paths src --ignore-src -r -y

# 編譯
colcon build --packages-select realsence sound

# 載入環境
source install/local_setup.bash
```

### 步驟 9: 設定模型檔案

```bash
# 確認模型檔案存在
ls -lh ~/114/ros2_ws/sound/tflite-model/model.tflite

# 如果不存在，從原電腦複製：
# scp user@原電腦IP:/home/richlin0308/114/ros2_ws/sound/tflite-model/model.tflite ~/114/ros2_ws/sound/tflite-model/
```

### 步驟 10: 檢查硬體連接

```bash
# 檢查 RealSense 相機
rs-enumerate-devices 2>/dev/null || echo "RealSense SDK 未安裝或相機未連接"

# 檢查音訊裝置
arecord -l

# 檢查 USB 序列埠
lsusb | grep -i "serial\|ch340\|cp210\|ft232"
```

---

## 🚀 啟動系統

### 方法 1: 使用啟動腳本（推薦）

```bash
cd ~/114
chmod +x start_system.sh
bash start_system.sh
```

### 方法 2: 手動啟動各個節點

```bash
# Terminal 1: 啟動 micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6

# Terminal 2: 啟動音訊錄製節點
cd ~/114/ros2_ws
source install/local_setup.bash
python3 sound/sound_record_1.py

# Terminal 3: 啟動視覺辨識節點
cd ~/114/ros2_ws
source install/local_setup.bash
python3 realsence/realsence_auto_hit.py

# Terminal 4: 啟動聲音推論節點
cd ~/114/ros2_ws
source install/local_setup.bash
python3 sound/sound_inference_node.py
```

---

## 🔧 故障排除

### 問題 1: NumPy 版本不相容

```bash
# 檢查版本
python3 -c "import numpy; print(numpy.__version__)"

# 如果版本 >= 2.0，降級：
pip3 install --user "numpy<2.0"
```

### 問題 2: OpenCV 無法載入

```bash
# 重新安裝 OpenCV
pip3 uninstall opencv-python opencv-contrib-python
pip3 install --user opencv-python
```

### 問題 3: RealSense 相機無法偵測

```bash
# 安裝 RealSense SDK
# 參考：https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

# 或使用 pip 安裝 Python 綁定
pip3 install --user pyrealsense2
```

### 問題 4: USB 序列埠權限不足

```bash
# 臨時解決（每次開機後執行）
sudo chmod 666 /dev/ttyUSB0

# 永久解決（加入 udev 規則）
sudo nano /etc/udev/rules.d/99-usb-serial.rules
# 加入：
# KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"
sudo udevadm control --reload-rules
```

### 問題 5: ROS2 環境未設定

```bash
# 檢查 ROS2 環境
echo $ROS_DISTRO

# 如果為空，設定：
source /opt/ros/humble/setup.bash
source ~/114/ros2_ws/install/local_setup.bash

# 加入 ~/.bashrc 使其永久生效
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/114/ros2_ws/install/local_setup.bash" >> ~/.bashrc
```

### 問題 6: 模型檔案找不到

```bash
# 檢查路徑
ls -lh ~/114/ros2_ws/sound/tflite-model/model.tflite

# 如果不存在，從原電腦複製
# 或修改 sound_inference_node.py 中的 MODEL_PATH
```

---

## 📝 檢查清單

部署前確認：
- [ ] ROS2 Humble 已安裝並設定
- [ ] Python 3.8+ 已安裝
- [ ] 所有 Python 依賴已安裝（`pip3 install -r requirements.txt`）
- [ ] NumPy 版本 < 2.0
- [ ] RealSense 相機可偵測
- [ ] 音訊裝置可偵測
- [ ] USB 序列埠權限已設定
- [ ] ROS2 工作空間已編譯
- [ ] 模型檔案已複製到正確位置
- [ ] 啟動腳本有執行權限

---

## 🔗 相關資源

- [ROS2 官方文件](https://docs.ros.org/en/humble/)
- [RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [TensorFlow Lite](https://www.tensorflow.org/lite/guide/python)

---

## 📞 支援

如遇到問題，請檢查：
1. 日誌檔案：`~/114/logs/system_*.log`
2. ROS2 節點狀態：`ros2 node list`
3. ROS2 Topic：`ros2 topic list`
4. 系統資源：`htop` 或 `top`

