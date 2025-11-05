#!/bin/bash
# 快速修復 NumPy 和 OpenCV 相容性問題

set -e

echo "=== 修復 NumPy 和 OpenCV 相容性問題 ==="
echo ""

# 檢查 Python
echo "檢查 Python 環境..."
python3 --version

# 卸載有問題的套件
echo ""
echo "卸載舊版 NumPy 和 OpenCV..."
pip3 uninstall -y numpy opencv-python opencv-contrib-python 2>/dev/null || true

# 安裝正確版本
echo ""
echo "安裝 NumPy < 2.0..."
pip3 install --user "numpy<2.0"

echo ""
echo "安裝 OpenCV..."
pip3 install --user opencv-python

# 驗證
echo ""
echo "驗證安裝..."
python3 -c "
import numpy
import cv2
print(f'✅ NumPy: {numpy.__version__}')
print(f'✅ OpenCV: {cv2.__version__}')

# 測試基本功能
import numpy as np
arr = np.array([1, 2, 3])
print(f'✅ NumPy 功能正常')

img = cv2.imread('/dev/null') 2>/dev/null || True
print(f'✅ OpenCV 載入正常')
"

echo ""
echo "=== 修復完成 ==="
echo "請重新執行啟動腳本：bash start_system.sh"

