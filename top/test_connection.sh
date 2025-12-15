#!/bin/bash

# 上機構連接測試腳本

echo "=== 上機構連接測試 ==="
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$SCRIPT_DIR/../ros2_ws"
MICROROS_WS="$SCRIPT_DIR/../microros_ws"

# 載入環境
set +u
source "$ROS_WS/install/local_setup.bash" 2>/dev/null
source "$MICROROS_WS/install/local_setup.bash" 2>/dev/null
set -u

echo "1. 檢查 ROS 2 節點："
NODES=$(ros2 node list 2>/dev/null)
echo "$NODES"
echo ""

if echo "$NODES" | grep -q "micro_ros_agent"; then
  echo "   ✓ 找到 /micro_ros_agent 節點"
else
  echo "   ✗ 未找到 /micro_ros_agent 節點"
fi

if echo "$NODES" | grep -q "move_node"; then
  echo "   ✓ 找到 /move_node 節點（Arduino 已連接！）"
else
  echo "   ✗ 未找到 /move_node 節點（Arduino 未連接）"
fi

if echo "$NODES" | grep -c "keyboard_bridge" | grep -q "^[2-9]"; then
  echo "   ⚠ 發現多個 keyboard_bridge 節點（重複），建議執行 ./start_top.sh 清理"
fi
echo ""

echo "2. 檢查 Topics："
TOPICS=$(ros2 topic list 2>/dev/null)
if echo "$TOPICS" | grep -q "top_base"; then
  echo "   ✓ 找到上機構 topics："
  echo "$TOPICS" | grep "top_base" | sed 's/^/      /'
else
  echo "   ✗ 未找到上機構 topics"
fi
echo ""

echo "3. 測試座標發布（等待 3 秒）："
timeout 3 ros2 topic echo /top_base_Coordinates 2>&1 | head -5 || echo "   未收到座標訊息"
echo ""

echo "=== 測試完成 ==="
echo ""
echo "如果 Arduino 未連接，請："
echo "1. 確認 Arduino 已上電並運行（查看串口監視器）"
echo "2. 確認 WiFi 連接成功（Arduino 串口應顯示 WiFi 相關訊息）"
echo "3. 重新啟動 Agent：./start_top.sh"
echo "4. 查看 Agent 日誌：tail -f logs/agent_*.log"

