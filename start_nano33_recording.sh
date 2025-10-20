#!/bin/bash

# NANO33 錄音啟動腳本
# 整合 micro-ROS Agent + NANO33 串列錄音（serial_wav_recorder.py）

echo "=== NANO33 錄音啟動 ==="

# 創建日誌目錄
LOG_DIR="logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/nano33_$(date +%Y%m%d_%H%M%S).log"

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

# 參數解析：--esp-port=... --nano-port=...
ESP_PORT=""
NANO_PORT_ARG=""
for arg in "$@"; do
  case "$arg" in
    --esp-port=*) ESP_PORT="${arg#*=}" ;;
    --nano-port=*) NANO_PORT_ARG="${arg#*=}" ;;
  esac
done

# 檢查並進入 ROS 2 工作空間
if [ -d "ros2_ws" ]; then
    echo "進入 ROS 2 工作空間..."
    cd ros2_ws
    
    # 編譯 ROS 2 工作空間（沿用 realsence/sound，若不存在會跳過）
    echo "編譯 ROS 2 工作空間..."
    colcon build --packages-select realsence sound || true
    
    # 載入編譯結果
    source install/local_setup.bash || true
    cd ..
fi

# 檢查 micro-ROS agent
echo "檢查 micro-ROS agent..."
if ! pgrep -f "micro_ros_agent" > /dev/null; then
    echo "啟動 micro-ROS agent..."
    
    # 檢查或建立 microros_ws
    if [ -d "microros_ws" ]; then
        echo "使用現有的 microros_ws..."
        (
          cd microros_ws
          # shellcheck disable=SC1091
          source install/local_setup.bash || true
        )
    else
        echo "創建新的 micro-ROS agent 工作空間..."
        # shellcheck disable=SC1091
        source /opt/ros/$ROS_DISTRO/setup.bash
        ros2 run micro_ros_setup create_agent_ws.sh || true
        ros2 run micro_ros_setup build_agent.sh || true
        (
          cd microros_ws 2>/dev/null || true
          # shellcheck disable=SC1091
          source install/local_setup.bash || true
        )
    fi
    
    # 選擇 ESP32 埠（可用 --esp-port 指定；否則挑第一個 ttyUSB*）
    if [ -z "$ESP_PORT" ]; then
        if ls /dev/ttyUSB* > /dev/null 2>&1; then
            ESP_PORT=$(ls /dev/ttyUSB* | head -n1)
        else
            echo "警告：找不到 ESP32 埠 (ttyUSB*)，預設使用 /dev/ttyUSB0" | tee -a "$LOG_FILE"
            ESP_PORT="/dev/ttyUSB0"
        fi
    fi
    echo "micro-ROS agent 使用埠：$ESP_PORT" | tee -a "$LOG_FILE"
    # 在背景啟動 micro-ROS agent
    ros2 run micro_ros_agent micro_ros_agent serial --dev "$ESP_PORT" -b 115200 -v6 >> "$LOG_FILE" 2>&1 &
    MICRO_ROS_PID=$!
    echo "micro-ROS agent PID: $MICRO_ROS_PID"
    sleep 3
else
    echo "micro-ROS agent 已在運行"
    MICRO_ROS_PID=$(pgrep -f "micro_ros_agent" | head -n1)
fi

# 檢查 USB 裝置權限（NANO33）
echo "檢查 NANO33 序列裝置..."
NANO_PORT="$NANO_PORT_ARG"
if [ -z "$NANO_PORT" ]; then
    if ls /dev/ttyACM* > /dev/null 2>&1; then
        for d in /dev/ttyACM*; do
            if [ "$d" != "$ESP_PORT" ]; then NANO_PORT="$d"; break; fi
        done
    fi
fi
if [ -z "$NANO_PORT" ]; then
    if ls /dev/ttyUSB* > /dev/null 2>&1; then
        for d in /dev/ttyUSB*; do
            if [ "$d" != "$ESP_PORT" ]; then NANO_PORT="$d"; break; fi
        done
    fi
fi

if [ -z "$NANO_PORT" ]; then
    echo "錯誤：找不到 NANO33 序列裝置 (避免與 ESP32 埠相同)。可用 --nano-port 指定。" | tee -a "$LOG_FILE"
    exit 1
fi

if [ "$NANO_PORT" = "$ESP_PORT" ]; then
    echo "錯誤：NANO33 與 ESP32 使用同一序列埠（$NANO_PORT）。請接到不同埠或用參數指定。" | tee -a "$LOG_FILE"
    exit 1
fi

echo "使用 NANO33 埠：$NANO_PORT"
if [ ! -r "$NANO_PORT" ] || [ ! -w "$NANO_PORT" ]; then
    echo "警告：$NANO_PORT 權限不足！" | tee -a "$LOG_FILE"
    echo "請執行：sudo chmod 666 $NANO_PORT" | tee -a "$LOG_FILE"
    echo "或將用戶加入 dialout 群組：sudo usermod -a -G dialout $USER" | tee -a "$LOG_FILE"
fi

# ========= 啟動 NANO33 錄音節點 =========
echo "=== 啟動錄音組件 ==="

echo "啟動 NANO33 串列錄音節點..."
python3 nano33/serial_wav_recorder.py --port "$NANO_PORT" --baud 2000000 --sample-rate 22050 >> "$LOG_FILE" 2>&1 &
REC_PID=$!
echo "錄音節點 PID: $REC_PID"

# 確認啟動
sleep 2
if ! kill -0 $REC_PID 2>/dev/null; then
    echo "錯誤：錄音節點啟動失敗！"
    exit 1
fi

echo "=== 啟動完成 ==="
echo "系統狀態："
echo "  錄音節點：PID $REC_PID"
echo "  micro-ROS：PID $MICRO_ROS_PID"
echo ""
echo "可用指令："
echo "  開始錄音：ros2 topic pub /moveknock_locate_node std_msgs/String \"data: 'start_recording:tile_1:left'\""
echo "  停止錄音：ros2 topic pub /moveknock_locate_node std_msgs/String \"data: 'stop_recording'\""
echo "  監控錄音：ros2 topic echo /moveknock_locate_node"
echo ""
echo "日誌檔案：$LOG_FILE"
echo "按 Ctrl+C 停止系統"

# 等待用戶中斷
trap 'echo "正在停止系統..."; echo "結束時間：$(date)" | tee -a "$LOG_FILE"; kill $REC_PID $MICRO_ROS_PID 2>/dev/null; echo "系統已停止" | tee -a "$LOG_FILE"; exit 0' INT TERM

# 保持腳本運行並監控系統狀態
while true; do
    if ! kill -0 $REC_PID 2>/dev/null; then
        echo "警告：錄音節點已停止！" | tee -a "$LOG_FILE"
    fi
    if ! kill -0 $MICRO_ROS_PID 2>/dev/null; then
        echo "警告：micro-ROS agent 已停止！" | tee -a "$LOG_FILE"
    fi
    sleep 10
done