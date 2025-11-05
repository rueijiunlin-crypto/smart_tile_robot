#!/bin/bash
# 新機器快速設定腳本

set -e

echo "=== 自動化敲擊錄音系統 - 新機器設定 ==="

# 檢查是否為 root（不應使用 root）
if [ "$EUID" -eq 0 ]; then 
    echo "請不要使用 root 執行此腳本"
    exit 1
fi

# 檢查 ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "警告：ROS2 環境未設定"
    echo "請先安裝 ROS2 Humble，然後執行："
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS2 環境：$ROS_DISTRO"

# 安裝系統依賴
echo "安裝系統依賴..."
sudo apt update
sudo apt install -y \
    portaudio19-dev \
    python3-dev \
    libasound2-dev \
    python3-pip \
    build-essential

# 安裝 Python 依賴
echo "安裝 Python 依賴..."
pip3 install --user -r requirements.txt

# 設定 USB 權限
echo "設定 USB 權限..."
sudo usermod -a -G dialout $USER

# 檢查工作空間
if [ ! -d "ros2_ws" ]; then
    echo "錯誤：找不到 ros2_ws 目錄"
    exit 1
fi

# 編譯工作空間
echo "編譯 ROS2 工作空間..."
cd ros2_ws
colcon build --packages-select realsence sound
source install/local_setup.bash
cd ..

echo ""
echo "=== 設定完成 ==="
echo ""
echo "請執行以下步驟："
echo "1. 登出後重新登入（讓 USB 權限生效）"
echo "2. 連接硬體裝置（RealSense、ESP32、麥克風）"
echo "3. 執行：bash start_system.sh"
