# ESP32 敲擊控制系統

## micro-ROS Agent 設定

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6

usbipd attach --wsl --busid 1-1


## 鍵盤控制指令

| 動作        | 指令範例                                                           |
| --------- | -------------------------------------------------------------- |
| 移動上方 10mm | `ros2 topic pub /keyboard_control std_msgs/String "data: 'w'"` |
| 移動下方 10mm | `ros2 topic pub /keyboard_control std_msgs/String "data: 's'"` |
| 移動左方 10mm | `ros2 topic pub /keyboard_control std_msgs/String "data: 'a'"` |
| 移動右方 10mm | `ros2 topic pub /keyboard_control std_msgs/String "data: 'd'"` |
| 敲擊一次      | `ros2 topic pub /keyboard_control std_msgs/String "data: 'h'"` |
| 歸零回原點     | `ros2 topic pub /keyboard_control std_msgs/String "data: 'r'"` |
| 軸重新初始化    | `ros2 topic pub /keyboard_control std_msgs/String "data: 'i'"` |
| 清除錯誤狀態   | `ros2 topic pub /keyboard_control std_msgs/String "data: 'c'"` |
| 查詢當前位置   | `ros2 topic pub /keyboard_control std_msgs/String "data: 'p'"` |
| 查詢最後錯誤   | `ros2 topic pub /keyboard_control std_msgs/String "data: 'e'"` |
| 開始敲擊序列   | `ros2 topic pub /keyboard_control std_msgs/String "data: 't'"` |
| 下一個敲擊位置 | `ros2 topic pub /keyboard_control std_msgs/String "data: 'n'"` |
| 結束敲擊序列   | `ros2 topic pub /keyboard_control std_msgs/String "data: 'q'"` |
| 緊急停止     | `ros2 topic pub /keyboard_control std_msgs/String "data: 'x'"` |
| 加速移動     | `ros2 topic pub /keyboard_control std_msgs/String "data: '+'"` |
| 減速移動     | `ros2 topic pub /keyboard_control std_msgs/String "data: '-'"` |

## 系統狀態監控

ros2 topic echo /moveknock_locate_node  # 監控系統狀態
ros2 topic echo /World_Coordinates      # 監控座標
ros2 topic echo /vision_status         # 監控視覺狀態

## 硬體配置

- X軸右限位開關：GPIO 33
- Y軸下限位開關：GPIO 32
- 座標系統：X軸 0-350mm，Y軸 0-140mm
- 初始化：通電後自動向右向下移動到限位開關

## 注意事項

- 確保 ESP32 已正確連接
- 檢查 USB 裝置權限
- 系統會自動初始化到原點
python3 keyboard_sender_loop.py

ros2 topic echo /keyboard_control

# 即時監控 ESP32 狀態
ros2 topic echo /moveknock_locate_node

# 監控視覺系統狀態
ros2 topic echo /vision_status

# 監控座標發布
ros2 topic echo /World_Coordinates