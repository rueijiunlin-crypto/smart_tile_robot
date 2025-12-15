#!/bin/bash

# Arduino 連接詳細診斷腳本

echo "=== Arduino 連接詳細診斷 ==="
echo ""

# 1. 檢查 Agent 狀態
echo "1. 檢查 micro-ROS Agent 狀態："
if pgrep -f "micro_ros_agent.*udp4.*2024" >/dev/null 2>&1; then
    echo "   ✓ Agent 進程正在運行"
    AGENT_PID=$(pgrep -f "micro_ros_agent.*udp4.*2024" | head -1)
    echo "   PID: $AGENT_PID"
else
    echo "   ✗ Agent 未運行！請執行 ./start_top.sh"
    exit 1
fi

# 2. 檢查 UDP 端口
echo ""
echo "2. 檢查 UDP 2024 端口："
if ss -tuln | grep -q ":2024.*udp"; then
    echo "   ✓ UDP 2024 正在監聽"
    ss -tuln | grep ":2024.*udp"
else
    echo "   ✗ UDP 2024 未監聽"
fi

# 3. 檢查 Agent 日誌
echo ""
echo "3. 檢查 Agent 日誌（最近 10 行）："
LATEST_LOG=$(ls -t ~/114/top/logs/agent_*.log 2>/dev/null | head -1)
if [ -n "$LATEST_LOG" ]; then
    echo "   日誌檔案: $LATEST_LOG"
    tail -10 "$LATEST_LOG" | sed 's/^/   /'
    if grep -q "Client connected\|New client\|client connected" "$LATEST_LOG" 2>/dev/null; then
        echo "   ✓ 發現客戶端連接訊息"
    else
        echo "   ✗ 未發現客戶端連接訊息"
    fi
else
    echo "   ✗ 找不到日誌檔案"
fi

# 4. 檢查 ROS 2 節點
echo ""
echo "4. 檢查 ROS 2 節點："
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$SCRIPT_DIR/../ros2_ws"
MICROROS_WS="$SCRIPT_DIR/../microros_ws"

if [ -f "$ROS_WS/install/local_setup.bash" ] && [ -f "$MICROROS_WS/install/local_setup.bash" ]; then
    set +u
    source "$ROS_WS/install/local_setup.bash" 2>/dev/null
    source "$MICROROS_WS/install/local_setup.bash" 2>/dev/null
    set -u
    
    NODES=$(ros2 node list 2>/dev/null)
    if echo "$NODES" | grep -q "move_node"; then
        echo "   ✓ 找到 /move_node（Arduino 已連接！）"
    else
        echo "   ✗ 未找到 /move_node（Arduino 未連接）"
    fi
    
    if echo "$NODES" | grep -q "micro_ros_agent"; then
        echo "   ✓ 找到 /micro_ros_agent 節點"
    else
        echo "   ⚠ /micro_ros_agent 節點未出現在 ROS 2 graph（可能正常，等待客戶端連接）"
    fi
fi

# 5. 檢查 Topics
echo ""
echo "5. 檢查 Topics："
if [ -f "$ROS_WS/install/local_setup.bash" ] && [ -f "$MICROROS_WS/install/local_setup.bash" ]; then
    TOPICS=$(ros2 topic list 2>/dev/null)
    if echo "$TOPICS" | grep -q "top_base"; then
        echo "   ✓ 找到上機構 topics："
        echo "$TOPICS" | grep "top_base" | sed 's/^/      /'
    else
        echo "   ✗ 未找到上機構 topics"
    fi
fi

# 6. 網路設定檢查
echo ""
echo "6. 網路設定檢查："
HOST_IP=$(ip -4 a | grep "inet " | grep -v "127.0.0.1" | awk '{print $2}' | cut -d'/' -f1 | head -1)
echo "   主機 IP: $HOST_IP"
echo "   Arduino 設定 IP: 172.31.153.161"
if [ "$HOST_IP" = "172.31.153.161" ]; then
    echo "   ✓ IP 地址匹配"
else
    echo "   ⚠ IP 地址不匹配！"
    echo "      請確認 Arduino 程式碼中的 IP 與主機實際 IP 一致"
fi

# 7. 診斷建議
echo ""
echo "=== 診斷建議 ==="
echo ""
echo "如果 Arduino 仍未連接，請檢查："
echo ""
echo "【Arduino 端檢查】"
echo "1. 串口監視器（115200 baud）是否顯示："
echo "   - '=== 上機構啟動 ==='"
echo "   - '設定 WiFi 連線...'"
echo "   - 'WiFi transport 設定完成，等待連線...'"
echo "   - 'Loop running... (ROS executor active)'"
echo ""
echo "2. 如果串口沒有輸出："
echo "   - 確認 Arduino 已上電"
echo "   - 確認 COM 埠選擇正確"
echo "   - 確認波特率設定為 115200"
echo "   - 重新燒錄程式碼"
echo ""
echo "3. 如果停在 '設定 WiFi 連線...'："
echo "   - WiFi SSID/密碼可能錯誤"
echo "   - WiFi 可能不是 2.4GHz"
echo "   - 等待更長時間（WiFi 連接可能需要 10-30 秒）"
echo ""
echo "【網路檢查】"
echo "4. 確認 Arduino 與主機在同一 WiFi 網段："
echo "   - Arduino 取得的 IP 應該是 172.31.x.x"
echo "   - 如果 Arduino IP 是 192.168.x.x 或其他，表示不在同一網段"
echo ""
echo "5. 檢查防火牆："
echo "   - WSL2 用戶：檢查 Windows 防火牆是否允許 UDP 2024"
echo "   - Linux 用戶：檢查 ufw/iptables 設定"
echo ""
echo "【測試步驟】"
echo "6. 建議測試順序："
echo "   a) 先啟動 Agent: ./start_top.sh"
echo "   b) 等待 3 秒讓 Agent 完全啟動"
echo "   c) 再上電 Arduino（拔插 USB 或按 Reset）"
echo "   d) 觀察 Agent 日誌: tail -f ~/114/top/logs/agent_*.log"
echo "   e) 等待 30 秒，觀察是否有 'Client connected' 訊息"
echo ""
echo "=== 診斷完成 ==="

