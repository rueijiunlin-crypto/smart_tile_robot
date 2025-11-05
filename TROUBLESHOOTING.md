# 🔧 快速修復指南

## 常見錯誤與解決方案

### 錯誤 1: NumPy 版本不相容

**錯誤訊息：**
```
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6
AttributeError: _ARRAY_API not found
```

**解決方法：**
```bash
# 檢查當前版本
python3 -c "import numpy; print(numpy.__version__)"

# 降級 NumPy
pip3 install --user "numpy<2.0" --force-reinstall

# 驗證
python3 -c "import numpy; print('NumPy:', numpy.__version__)"
```

---

### 錯誤 2: OpenCV (cv2) 無法載入

**錯誤訊息：**
```
ImportError: numpy.core.multiarray failed to import
AttributeError: _ARRAY_API not found
```

**解決方法：**
```bash
# 方法 1: 重新安裝 OpenCV（先降級 NumPy）
pip3 install --user "numpy<2.0" --force-reinstall
pip3 install --user opencv-python --force-reinstall

# 方法 2: 如果方法 1 無效，完全移除後重裝
pip3 uninstall opencv-python opencv-contrib-python
pip3 install --user "numpy<2.0"
pip3 install --user opencv-python
```

---

### 錯誤 3: tflite_runtime 未安裝或無法載入

**錯誤訊息：**
```
ModuleNotFoundError: No module named 'tflite_runtime'
```

**解決方法：**
```bash
# 檢查 Python 版本
python3 --version

# 下載適合的 wheel 檔案（以 Python 3.10 x86_64 為例）
cd /tmp
wget https://github.com/google-coral/pycoral/releases/download/v2.0.0/tflite_runtime-2.5.0-cp310-cp310-linux_x86_64.whl

# 安裝
pip3 install --user tflite_runtime-2.5.0-cp310-cp310-linux_x86_64.whl

# 驗證
python3 -c "from tflite_runtime.interpreter import Interpreter; print('✅ tflite_runtime 安裝成功')"

# 其他版本請參考：
# https://github.com/google-coral/pycoral/releases
```

---

### 錯誤 4: ROS2 環境未設定

**錯誤訊息：**
```
錯誤：ROS 2 環境未設定！
請執行：source /opt/ros/humble/setup.bash
```

**解決方法：**
```bash
# 臨時設定（當前終端）
source /opt/ros/humble/setup.bash

# 永久設定（加入 ~/.bashrc）
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/114/ros2_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 錯誤 5: rclpy.shutdown() 重複呼叫

**錯誤訊息：**
```
RCLError: rcl_shutdown already called
```

**解決方法：**
- 此問題已在最新版本修復
- 確保使用最新版本的程式碼
- 如果仍有問題，檢查程式碼中的 `finally` 區塊是否包含 `if rclpy.ok():` 檢查

---

### 錯誤 6: USB 序列埠權限不足

**錯誤訊息：**
```
Permission denied: /dev/ttyUSB0
```

**解決方法：**
```bash
# 臨時解決（每次開機後執行）
sudo chmod 666 /dev/ttyUSB0

# 永久解決
sudo usermod -a -G dialout $USER
# 登出後重新登入

# 或建立 udev 規則
echo 'KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"' | sudo tee /etc/udev/rules.d/99-usb-serial.rules
sudo udevadm control --reload-rules
```

---

### 錯誤 7: 模型檔案找不到

**錯誤訊息：**
```
ValueError: Could not open '.../model.tflite'
```

**解決方法：**
```bash
# 檢查檔案是否存在
ls -lh ~/114/ros2_ws/sound/tflite-model/model.tflite

# 如果不存在，從原電腦複製
# scp user@原電腦IP:/home/richlin0308/114/ros2_ws/sound/tflite-model/model.tflite ~/114/ros2_ws/sound/tflite-model/

# 或確認路徑設定正確
# 檢查 sound_inference_node.py 中的 MODEL_PATH
```

---

### 錯誤 8: pyaudio 安裝失敗

**錯誤訊息：**
```
ERROR: Failed building wheel for pyaudio
```

**解決方法：**
```bash
# 安裝系統依賴
sudo apt update
sudo apt install -y portaudio19-dev python3-dev libasound2-dev

# 重新安裝
pip3 install --user pyaudio
```

---

## 🔍 完整診斷流程

### 步驟 1: 執行環境檢查

```bash
cd ~/114
bash check_environment.sh
```

### 步驟 2: 根據檢查結果修復

如果檢查發現問題，按照上述對應的解決方案修復。

### 步驟 3: 重新檢查

```bash
bash check_environment.sh
```

### 步驟 4: 測試單個節點

```bash
# 測試視覺節點
cd ~/114/ros2_ws
source install/local_setup.bash
python3 realsence/realsence_auto_hit.py

# 測試音訊節點
python3 sound/sound_record_1.py

# 測試推論節點
python3 sound/sound_inference_node.py
```

### 步驟 5: 如果單個節點正常，啟動完整系統

```bash
cd ~/114
bash start_system.sh
```

---

## 📝 一鍵修復腳本

如果遇到 NumPy/OpenCV 問題，執行：

```bash
#!/bin/bash
# 快速修復 NumPy 和 OpenCV 問題

echo "修復 NumPy 和 OpenCV 相容性問題..."

# 卸載有問題的套件
pip3 uninstall -y numpy opencv-python opencv-contrib-python

# 安裝正確版本
pip3 install --user "numpy<2.0"
pip3 install --user opencv-python

# 驗證
python3 -c "import numpy, cv2; print(f'✅ NumPy: {numpy.__version__}, OpenCV: {cv2.__version__}')"

echo "修復完成！"
```

儲存為 `fix_numpy_opencv.sh`，執行 `bash fix_numpy_opencv.sh`

---

## 💡 建議

1. **先執行診斷**：使用 `check_environment.sh` 檢查環境
2. **依序修復**：按照錯誤訊息逐一解決
3. **測試單個節點**：確認每個節點都能單獨運行
4. **查看日誌**：檢查 `logs/system_*.log` 獲取詳細錯誤訊息

---

## 📞 需要協助？

如果以上方法都無法解決，請提供：
1. 完整的錯誤訊息（從終端複製）
2. `check_environment.sh` 的輸出結果
3. `logs/system_*.log` 的內容

