#!/bin/bash

# 上機構連接診斷腳本

echo "=== 上機構連接診斷 ==="
echo ""

# 1. 檢查 Agent 進程
echo "1. 檢查 micro-ROS Agent 進程："
if pgrep -f "micro_ros_agent .*udp4 .*2024" >/dev/null 2>&1; then
    echo "   ✓ Agent 進程正在運行"
    ps -ef | grep "micro_ros_agent.*udp4.*2024" | grep -v grep | head -1
else
    echo "   ✗ Agent 進程未運行！請先執行 ./start_top.sh"
    exit 1
fi
echo ""

# 2. 檢查 UDP 端口
echo "2. 檢查 UDP 2024 端口："
if ss -tuln | grep -q ":2024.*udp"; then
    echo "   ✓ UDP 2024 端口正在監聽"
    ss -tuln | grep ":2024.*udp"
else
    echo "   ✗ UDP 2024 端口未監聽！"
fi
echo ""

# 3. 載入 ROS 環境
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$SCRIPT_DIR/../ros2_ws"
MICROROS_WS="$SCRIPT_DIR/../microros_ws"

if [ -f "$ROS_WS/install/local_setup.bash" ] && [ -f "$MICROROS_WS/install/local_setup.bash" ]; then
    set +u
    source "$ROS_WS/install/local_setup.bash" 2>/dev/null
    source "$MICROROS_WS/install/local_setup.bash" 2>/dev/null
    set -u
    
    # 4. 檢查 ROS 2 節點
    echo "3. 檢查 ROS 2 節點："
    NODES=$(ros2 node list 2>/dev/null)
    if echo "$NODES" | grep -q "micro_ros_agent"; then
        echo "   ✓ 找到 /micro_ros_agent 節點"
    else
        echo "   ⚠ /micro_ros_agent 節點未出現在 ROS 2 graph 中（可能正常，等待客戶端連接）"
    fi
    
    if echo "$NODES" | grep -q "move_node"; then
        echo "   ✓ 找到 /move_node 節點（Arduino 已連接！）"
    else
        echo "   ✗ 未找到 /move_node 節點（Arduino 未連接）"
    fi
    echo ""
    
    # 5. 檢查 Topics
    echo "4. 檢查 Topics："
    TOPICS=$(ros2 topic list 2>/dev/null)
    if echo "$TOPICS" | grep -q "top_base"; then
        echo "   ✓ 找到上機構相關 topics："
        echo "$TOPICS" | grep "top_base" | sed 's/^/      /'
    else
        echo "   ✗ 未找到上機構相關 topics（Arduino 未連接或未發布）"
    fi
    echo ""
    
    # 6. 檢查 Agent 日誌（如果可能）
    echo "5. Agent 連接狀態提示："
    echo "   請檢查執行 start_top.sh 的終端機，查看是否有以下訊息："
    echo "   - 'Client connected' 或 'New client connected' → Arduino 已連接"
    echo "   - 'Waiting for client...' → 等待 Arduino 連接"
    echo "   - 沒有任何訊息 → 可能 Agent 輸出被重定向"
    echo ""
    
    # 7. Arduino 檢查建議
    echo "6. Arduino 檢查建議："
    echo "   a) 確認板子已上電"
    echo "   b) 打開 Arduino IDE 的序列埠監視器（115200 baud）"
    echo "   c) 查看是否有以下訊息："
    echo "      - 'WiFi connected' → WiFi 連接成功"
    echo "      - 'IP address: xxx.xxx.xxx.xxx' → 取得 IP"
    echo "      - 'Connecting to agent at 172.31.153.161:2024' → 嘗試連接 Agent"
    echo "      - 'Agent connected' 或類似訊息 → 連接成功"
    echo "      - 錯誤訊息 → 請檢查 WiFi SSID/密碼/IP/Port"
    echo ""
    
    # 8. 網路檢查
    echo "7. 網路設定檢查："
    HOST_IP=$(ip -4 a | grep "inet " | grep -v "127.0.0.1" | awk '{print $2}' | cut -d'/' -f1 | head -1)
    echo "   主機 IP: $HOST_IP"
    echo "   Arduino 設定 IP: 172.31.153.161"
    if [ "$HOST_IP" = "172.31.153.161" ]; then
        echo "   ✓ IP 地址匹配"
    else
        echo "   ⚠ IP 地址不匹配！請確認 Arduino 設定的 IP 與主機實際 IP 一致"
    fi
    echo ""
    
    echo "=== 診斷完成 ==="
    echo ""
    echo "如果 Arduino 仍未連接，請："
    echo "1. 確認 Arduino 已燒錄最新程式碼（WiFi SSID/密碼/IP/Port 正確）"
    echo "2. 確認 Arduino 與主機在同一 WiFi 網段（2.4GHz）"
    echo "3. 確認主機防火牆允許 UDP 2024"
    echo "4. 重啟 Arduino 板子（拔插 USB 或重啟電源）"
    echo "5. 先啟動 Agent，再上電 Arduino（建議順序）"
    
else
    echo "錯誤：找不到 ROS 2 或 micro-ROS 環境"
fi

