#!/bin/bash

# 網路連通性檢查腳本

echo "=== 網路連通性檢查 ==="
echo ""

# 1. 檢查主機 IP
echo "1. 主機 IP 地址："
HOST_IP=$(ip -4 a | grep "inet " | grep -v "127.0.0.1" | awk '{print $2}' | cut -d'/' -f1 | head -1)
echo "   主機 IP: $HOST_IP"
echo "   Arduino 設定 IP: 172.31.153.161"
if [ "$HOST_IP" = "172.31.153.161" ]; then
    echo "   ✓ IP 地址匹配"
else
    echo "   ✗ IP 地址不匹配！"
    echo "      請更新 Arduino 程式碼中的 IP 為: $HOST_IP"
fi
echo ""

# 2. 檢查 UDP 2024 監聽
echo "2. UDP 2024 端口監聽狀態："
if ss -tuln | grep -q ":2024.*udp"; then
    echo "   ✓ UDP 2024 正在監聽"
    ss -tuln | grep ":2024.*udp" | sed 's/^/      /'
else
    echo "   ✗ UDP 2024 未監聽"
fi
echo ""

# 3. 檢查 Agent 進程
echo "3. micro-ROS Agent 進程："
if pgrep -f "micro_ros_agent.*udp4.*2024" >/dev/null 2>&1; then
    echo "   ✓ Agent 正在運行"
    pgrep -f "micro_ros_agent.*udp4.*2024" | head -1 | xargs ps -p | tail -1 | sed 's/^/      /'
else
    echo "   ✗ Agent 未運行"
fi
echo ""

# 4. 檢查防火牆（如果可能）
echo "4. 防火牆檢查："
if command -v ufw >/dev/null 2>&1; then
    UFW_STATUS=$(sudo ufw status 2>/dev/null | head -1)
    if echo "$UFW_STATUS" | grep -q "Status: active"; then
        echo "   ⚠ 防火牆已啟用"
        echo "      請確認 UDP 2024 是否允許："
        echo "      sudo ufw allow 2024/udp"
    else
        echo "   ✓ 防火牆未啟用或未安裝 ufw"
    fi
else
    echo "   ⚠ 無法檢查防火牆（需要 ufw 或 root 權限）"
    echo "      WSL2 用戶：請檢查 Windows 防火牆設定"
fi
echo ""

# 5. 網路介面資訊
echo "5. 網路介面資訊："
ip -4 a | grep "inet " | grep -v "127.0.0.1" | while read line; do
    INTERFACE=$(echo "$line" | awk '{print $NF}')
    IP=$(echo "$line" | awk '{print $2}' | cut -d'/' -f1)
    echo "   介面: $INTERFACE, IP: $IP"
done
echo ""

# 6. 建議
echo "=== 診斷建議 ==="
echo ""
echo "如果 Arduino 無法連接，請檢查："
echo ""
echo "1. WiFi 設定："
echo "   - SSID: RayG"
echo "   - 密碼: n6n6um7i"
echo "   - 確認 WiFi 是 2.4GHz（部分 ESP32 不支援 5GHz）"
echo ""
echo "2. 網路連通性："
echo "   - Arduino 與主機必須在同一 WiFi 網段"
echo "   - Arduino 取得的 IP 應該是 172.31.x.x（與主機同網段）"
echo "   - 如果 Arduino IP 是 192.168.x.x 或其他，表示不在同一網段"
echo ""
echo "3. 防火牆："
echo "   - WSL2 用戶：檢查 Windows 防火牆是否允許 UDP 2024"
echo "   - Linux 用戶：確認 ufw/iptables 允許 UDP 2024"
echo ""
echo "4. 測試步驟："
echo "   a) 確認 Agent 正在運行：ps aux | grep micro_ros_agent"
echo "   b) 確認 UDP 2024 正在監聽：ss -tuln | grep 2024"
echo "   c) 重新啟動 Arduino（拔插 USB 或按 Reset）"
echo "   d) 觀察 Agent 日誌：tail -f ~/114/top/logs/agent_*.log"
echo "   e) 等待 30-60 秒，觀察是否有連接嘗試"
echo ""
echo "5. 如果仍然無法連接："
echo "   - 檢查 Arduino 串口監視器是否有 WiFi 相關錯誤訊息"
echo "   - 嘗試使用不同的 WiFi 網路測試"
echo "   - 確認 Arduino 板子的 WiFi 模組正常工作"
echo ""

