#!/bin/bash

# 自動化敲擊錄音系統啟動腳本
# 整合視覺識別、硬體控制、音訊錄製

echo "=== 自動化敲擊錄音系統啟動 ==="

# 創建日誌目錄
LOG_DIR="logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/system_$(date +%Y%m%d_%H%M%S).log"

echo "日誌檔案：$LOG_FILE"
echo "開始時間：$(date)" | tee -a "$LOG_FILE"

# 檢查 ROS 2 環境
if [ -z "$ROS_DISTRO" ]; then
    echo "錯誤：ROS 2 環境未設定！"
    echo "請執行：source /opt/ros/humble/setup.bash"
    exit 1
fi

# 設定工作目錄
cd "$(dirname "$0")"

# 檢查並進入 ROS 2 工作空間
if [ -d "ros2_ws" ]; then
    echo "進入 ROS 2 工作空間..."
    cd ros2_ws
    
    # 編譯 ROS 2 工作空間
    echo "編譯 ROS 2 工作空間..."
    colcon build --packages-select realsence sound
    
    if [ $? -ne 0 ]; then
        echo "錯誤：編譯失敗！"
        exit 1
    fi
    
    # 載入編譯結果
    source install/local_setup.bash
    cd ..
else
    echo "錯誤：找不到 ros2_ws 目錄！"
    exit 1
fi

# 檢查 micro-ROS agent
echo "檢查 micro-ROS agent..."
if ! pgrep -f "micro_ros_agent" > /dev/null; then
    echo "啟動 micro-ROS agent..."
    
    # 檢查 microros_ws 目錄
    if [ -d "microros_ws" ]; then
        echo "使用現有的 microros_ws..."
        cd microros_ws
        source install/local_setup.bash
        cd ..
    else
        echo "創建新的 micro-ROS agent..."
        source install/local_setup.bash
        ros2 run micro_ros_setup create_agent_ws.sh
        ros2 run micro_ros_setup build_agent.sh
        source install/local_setup.bash
    fi
    
    # 在背景啟動 micro-ROS agent
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6 &
    MICRO_ROS_PID=$!
    echo "micro-ROS agent PID: $MICRO_ROS_PID"
    sleep 3
else
    echo "micro-ROS agent 已在運行"
    MICRO_ROS_PID=$(pgrep -f "micro_ros_agent")
fi

# 檢查攝影機
echo "檢查攝影機..."
if ! ls /dev/video* > /dev/null 2>&1; then
    echo "警告：未檢測到攝影機！"
else
    echo "檢測到攝影機：$(ls /dev/video*)"
fi

# 檢查音訊裝置
echo "檢查音訊裝置..."
if ! arecord -l > /dev/null 2>&1; then
    echo "警告：未檢測到音訊裝置！"
else
    echo "檢測到音訊裝置："
    arecord -l | grep -E "card|device"
fi

# 檢查 USB 裝置權限
echo "檢查 USB 裝置權限..."
if ls /dev/ttyUSB* > /dev/null 2>&1; then
    for device in /dev/ttyUSB*; do
        if [ ! -r "$device" ] || [ ! -w "$device" ]; then
            echo "警告：$device 權限不足！"
            echo "請執行：sudo chmod 666 $device"
            echo "或將用戶加入 dialout 群組：sudo usermod -a -G dialout $USER"
        else
            echo "$device 權限正常"
        fi
    done
fi

# 檢查系統資源
echo "檢查系統資源..."
echo "可用記憶體：$(free -h | grep '^Mem:' | awk '{print $7}')"
echo "可用磁碟空間：$(df -h . | tail -1 | awk '{print $4}')"

# 錄音資料夾由 sound_record_1.py 自動建立在 ros2_ws/sound/recordings/ 目錄下 (以日期時間命名)

echo "=== 啟動系統組件 ==="

# 跳過配置載入器（目前沒有配置檔案）
echo "跳過配置載入器（目前沒有配置檔案）"

# 啟動音訊錄製節點
echo "啟動音訊錄製節點..."
python3 ros2_ws/sound/sound_record_1.py &
SOUND_PID=$!
echo "音訊錄製節點 PID: $SOUND_PID"

# 檢查音訊節點是否成功啟動
sleep 2
if ! kill -0 $SOUND_PID 2>/dev/null; then
    echo "錯誤：音訊錄製節點啟動失敗！"
    exit 1
fi

# 啟動視覺識別節點
echo "啟動視覺識別節點..."
# 先檢查 Python 依賴
python3 -c "import cv2; import numpy; print(f'✅ OpenCV: {cv2.__version__}, NumPy: {numpy.__version__}')" 2>&1
if [ $? -ne 0 ]; then
    echo "錯誤：Python 依賴檢查失敗！請確認 cv2 和 numpy 已正確安裝。" | tee -a "$LOG_FILE"
    echo "建議執行：pip3 install 'numpy<2' opencv-python" | tee -a "$LOG_FILE"
    kill $SOUND_PID 2>/dev/null
    exit 1
fi
python3 ros2_ws/realsence/realsence_auto_hit.py >> "$LOG_FILE" 2>&1 &
VISION_PID=$!
echo "視覺識別節點 PID: $VISION_PID"

# 檢查視覺節點是否成功啟動
sleep 2
if ! kill -0 $VISION_PID 2>/dev/null; then
    echo "錯誤：視覺識別節點啟動失敗！請檢查日誌：$LOG_FILE" | tee -a "$LOG_FILE"
    kill $SOUND_PID 2>/dev/null
    exit 1
fi

# 啟動聲音推論節點（ros2_ws/sound/sound_inference_node.py）
echo "啟動聲音推論節點..."
# 檢查模型檔是否存在
SOUND_DIR="ros2_ws/sound"
MODEL_FILE="$SOUND_DIR/tflite-model/model.tflite"
if [ ! -f "$MODEL_FILE" ]; then
    echo "警告：找不到模型檔 $MODEL_FILE，推論節點仍會啟動但可能無法運作。" | tee -a "$LOG_FILE"
fi
python3 ros2_ws/sound/sound_inference_node.py &
INFER_PID=$!
echo "聲音推論節點 PID: $INFER_PID"

# 檢查推論節點是否成功啟動
sleep 2
if ! kill -0 $INFER_PID 2>/dev/null; then
    echo "錯誤：聲音推論節點啟動失敗！"
    # 不中斷整體系統，但提示使用者
fi

echo "=== 系統啟動完成 ==="
echo "系統狀態："
echo "  音訊節點：PID $SOUND_PID"
echo "  視覺節點：PID $VISION_PID"
echo "  聲音推論：PID ${INFER_PID:-N/A}"
echo "  micro-ROS：PID $MICRO_ROS_PID"
echo ""
echo "可用指令："
echo "  監控座標：ros2 topic echo /World_Coordinates"
echo "  監控狀態：ros2 topic echo /vision_status"
echo "  監控錄音：ros2 topic echo /moveknock_locate_node"
echo "  監控推論：ros2 topic echo /tile_sound_result"
echo "  鍵盤控制：python3 hit/keyboard_sender_loop.py"
echo "  系統配置：ros2 topic echo /system_config"
echo "  ESP32狀態：ros2 topic echo /move_hit_initialized"
echo ""
echo "系統監控："
echo "  CPU使用率：top -p $SOUND_PID,$VISION_PID,$MICRO_ROS_PID"
echo "  記憶體使用：ps -p $SOUND_PID,$VISION_PID,$MICRO_ROS_PID -o pid,ppid,cmd,%mem,%cpu"
echo "  系統負載：uptime"
echo "  即時監控：watch -n 1 'ps -p $SOUND_PID,$VISION_PID,$MICRO_ROS_PID -o pid,ppid,cmd,%mem,%cpu'"
echo "  系統狀態：htop"
echo "  網路狀態：ss -tuln"
echo ""
echo "日誌檔案：$LOG_FILE"
echo "按 Ctrl+C 停止系統"

# 等待用戶中斷
trap 'echo "正在停止系統..."; echo "結束時間：$(date)" | tee -a "$LOG_FILE"; kill $SOUND_PID $VISION_PID ${INFER_PID:-} $MICRO_ROS_PID 2>/dev/null; echo "系統已停止" | tee -a "$LOG_FILE"; exit 0' INT TERM

# 保持腳本運行並監控系統狀態
while true; do
    # 檢查所有進程是否還在運行
    if ! kill -0 $SOUND_PID 2>/dev/null; then
        echo "警告：音訊節點已停止！" | tee -a "$LOG_FILE"
    fi
    
    if ! kill -0 $VISION_PID 2>/dev/null; then
        echo "警告：視覺節點已停止！" | tee -a "$LOG_FILE"
    fi
    if [ -n "${INFER_PID:-}" ] && ! kill -0 $INFER_PID 2>/dev/null; then
        echo "警告：聲音推論節點已停止！" | tee -a "$LOG_FILE"
    fi
    
    if ! kill -0 $MICRO_ROS_PID 2>/dev/null; then
        echo "警告：micro-ROS agent 已停止！" | tee -a "$LOG_FILE"
    fi
    
    sleep 10
done
