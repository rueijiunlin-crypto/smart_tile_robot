#!/bin/bash
# 環境檢查腳本

echo "=== 環境檢查 ==="
echo ""

# 檢查 ROS2
echo "1. 檢查 ROS2 環境..."
if [ -z "$ROS_DISTRO" ]; then
    echo "   ❌ ROS2 環境未設定"
    echo "   請執行：source /opt/ros/humble/setup.bash"
else
    echo "   ✅ ROS2 版本：$ROS_DISTRO"
fi

# 檢查 Python
echo ""
echo "2. 檢查 Python..."
PYTHON_VERSION=$(python3 --version 2>&1)
echo "   ✅ $PYTHON_VERSION"

# 檢查 Python 依賴
echo ""
echo "3. 檢查 Python 依賴套件..."
python3 -c "
import sys
errors = []
try:
    import numpy
    ver = numpy.__version__
    if float(ver.split('.')[0]) >= 2:
        errors.append(f'❌ NumPy 版本 {ver} >= 2.0（需要 < 2.0）')
    else:
        print(f'   ✅ NumPy: {ver}')
except ImportError:
    errors.append('❌ NumPy 未安裝')

try:
    import cv2
    print(f'   ✅ OpenCV: {cv2.__version__}')
except ImportError:
    errors.append('❌ OpenCV (cv2) 未安裝')

try:
    import soundfile
    print(f'   ✅ soundfile: {soundfile.__version__}')
except ImportError:
    errors.append('❌ soundfile 未安裝')

try:
    import pyaudio
    print(f'   ✅ pyaudio: 已安裝')
except ImportError:
    errors.append('❌ pyaudio 未安裝')

try:
    import rclpy
    print(f'   ✅ rclpy: 已安裝')
except ImportError:
    errors.append('❌ rclpy 未安裝（ROS2 未正確設定）')

try:
    from tflite_runtime.interpreter import Interpreter
    print(f'   ✅ tflite_runtime: 已安裝')
except ImportError:
    errors.append('❌ tflite_runtime 未安裝')

try:
    import pyrealsense2 as rs
    print(f'   ✅ pyrealsense2: 已安裝')
except ImportError:
    errors.append('❌ pyrealsense2 未安裝')

for err in errors:
    print(f'   {err}')

if errors:
    sys.exit(1)
" 2>&1

DEP_CHECK=$?

# 檢查檔案
echo ""
echo "4. 檢查專案檔案..."
if [ -f "ros2_ws/sound/tflite-model/model.tflite" ]; then
    echo "   ✅ 模型檔案存在"
else
    echo "   ❌ 模型檔案不存在：ros2_ws/sound/tflite-model/model.tflite"
fi

if [ -f "ros2_ws/sound/sound_record_1.py" ]; then
    echo "   ✅ sound_record_1.py 存在"
else
    echo "   ❌ sound_record_1.py 不存在"
fi

if [ -f "ros2_ws/realsence/realsence_auto_hit.py" ]; then
    echo "   ✅ realsence_auto_hit.py 存在"
else
    echo "   ❌ realsence_auto_hit.py 不存在"
fi

# 檢查硬體
echo ""
echo "5. 檢查硬體..."
if ls /dev/ttyUSB* /dev/ttyACM* >/dev/null 2>&1; then
    echo "   ✅ USB 序列埠：$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"
else
    echo "   ⚠️  未偵測到 USB 序列埠"
fi

if command -v arecord >/dev/null 2>&1; then
    if arecord -l >/dev/null 2>&1; then
        echo "   ✅ 音訊裝置：已偵測"
    else
        echo "   ⚠️  未偵測到音訊裝置"
    fi
else
    echo "   ⚠️  arecord 未安裝"
fi

# 檢查 USB 權限
echo ""
echo "6. 檢查 USB 權限..."
if groups | grep -q dialout; then
    echo "   ✅ 用戶在 dialout 群組中"
else
    echo "   ❌ 用戶不在 dialout 群組中"
    echo "   請執行：sudo usermod -a -G dialout $USER"
    echo "   然後登出並重新登入"
fi

echo ""
if [ $DEP_CHECK -ne 0 ]; then
    echo "=== 發現問題！請先解決上述錯誤 ==="
    exit 1
else
    echo "=== 環境檢查通過 ==="
fi
