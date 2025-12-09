#!/bin/bash

# 上機構啟動腳本（僅啟動上機構相關元件，不影響現有下機構流程）
# 功能：
# 1) 啟動 micro-ROS UDP Agent（埠 2024）
# 2) 載入 ROS 2 環境
# 3) 可選：啟動鍵盤指令橋接節點（/keyboard_control -> /keyboard_input）

set -euo pipefail

safe_source() {
  # 在 set -u 環境下，colcon 生成的 local_setup 會依賴 COLCON_CURRENT_PREFIX
  # 先暫時關閉 -u，source 後再恢復
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$SCRIPT_DIR/../ros2_ws"
MICROROS_WS="$SCRIPT_DIR/../microros_ws"
AGENT_PORT=2024

echo "=== 啟動上機構 ==="

if [ ! -f "$ROS_WS/install/local_setup.bash" ]; then
  echo "錯誤：找不到 ROS 2 環境，請先在 ros2_ws 執行 colcon build。"
  exit 1
fi

if [ ! -f "$MICROROS_WS/install/local_setup.bash" ]; then
  echo "錯誤：找不到 microros_ws，請先建立 micro-ROS Agent 工作區。"
  exit 1
fi

echo "載入 ROS 2 環境：$ROS_WS/install/local_setup.bash"
safe_source "$ROS_WS/install/local_setup.bash"

echo "載入 micro-ROS Agent 環境：$MICROROS_WS/install/local_setup.bash"
safe_source "$MICROROS_WS/install/local_setup.bash"

echo "啟動 micro-ROS UDP Agent (port=$AGENT_PORT)..."
if pgrep -f "micro_ros_agent .*udp4 .*${AGENT_PORT}" >/dev/null 2>&1; then
  echo "micro-ROS UDP Agent 已在執行，略過。"
else
  ros2 run micro_ros_agent micro_ros_agent udp4 --port "$AGENT_PORT" -v6 &
  AGENT_PID=$!
  echo "micro-ROS UDP Agent PID: $AGENT_PID"
  sleep 2
fi

# 選用：啟動鍵盤橋接節點，將 /keyboard_control 轉發到 /keyboard_input
# 如需自動從 vision 節點帶動上機構，建議啟用。
if [ "${ENABLE_KEYBOARD_BRIDGE:-1}" = "1" ]; then
  echo "啟動鍵盤橋接節點..."
  python3 "$SCRIPT_DIR/keyboard_bridge.py" &
  BRIDGE_PID=$!
  echo "鍵盤橋接節點 PID: $BRIDGE_PID"
fi

echo "=== 上機構啟動完成 ==="
echo "提示："
echo "  - 發送指令測試: ros2 topic pub /keyboard_input std_msgs/msg/String \"data: 'd'\""
echo "  - 監看座標:      ros2 topic echo /top_base_Coordinates"
echo "  - WiFi/UDP 設定需與 Arduino 相同 (set_microros_wifi_transports)"

