#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardSender(Node):
    def __init__(self):
        super().__init__('keyboard_sender')
        self.publisher_ = self.create_publisher(String, '/keyboard_control', 10)

    def send_once(self, key_char):
        msg = String()
        msg.data = key_char
        self.publisher_.publish(msg)
        self.get_logger().info(f'送出指令: {msg.data}')
    
    def show_help(self):
        """顯示完整功能說明"""
        print("\n" + "="*60)
        print("敲擊機構控制系統 - 完整功能說明")
        print("="*60)
        print("\n基本移動控制:")
        print("  w  - Y軸向上移動 10mm")
        print("  s  - Y軸向下移動 10mm") 
        print("  a  - X軸向左移動 10mm")
        print("  d  - X軸向右移動 10mm")
        print("  r  - 回到原點 (0,0)")
        
        print("\n敲擊控制:")
        print("  h  - 單次敲擊")
        print("  i  - 重新初始化 (歸零)")
        print("  c  - 清除錯誤狀態 (從 ERROR 回到 READY)")
        print("  x  - 緊急停止 (停止所有動作)or狀態重製")
        
        print("\n速度控制:")
        print("  +  - 加速 (減少延遲時間)")
        print("  -  - 減速 (增加延遲時間)")
        
        print("\n敲擊序列控制:")
        print("  t  - 啟動敲擊序列 (開始第1塊磁磚)")
        print("  n  - 下一個敲擊位置 (left → mid → right)")
        print("  q  - 結束敲擊序列 (進入下一塊磁磚)")
        
        print("\n錄音檔命名規則:")
        print("  單次敲擊: 00001.wav, 00002.wav, ...")
        print("  敲擊序列: 1_left.wav, 1_mid.wav, 1_right.wav, ...")
        
        print("\n使用範例:")
        print("  1. 手動移動: w → a → s → d")
        print("  2. 單次敲擊: h")
        print("  3. 敲擊序列: t → h → n → h → n → h → q")
        print("  4. 速度調整: + → + → + (加速)")
        print("  5. 緊急停止: x (任何時候)")
        
        print("\n硬體配置:")
        print("  X軸範圍: 0.0mm → 350.0mm (右限位開關)")
        print("  Y軸範圍: 0.0mm → 140.0mm (下限位開關)")
        print("  初始化: 通電後自動向右向下移動到限位開關")
        
        print("\n查詢指令:")
        print("  p  - 查詢當前位置")
        print("  e  - 查詢最後錯誤")
        print("  m  - 敲擊馬達硬體測試")
        print("  M  - 移動馬達硬體測試")
        
        print("\n其他指令:")
        print("  help - 顯示此說明")
        print("  q    - 離開程式")
        print("="*60)

def main():
    rclpy.init()
    node = KeyboardSender()

    try:
        while True:
            key = input("輸入控制指令 (w/a/s/d/h/r/i/c/p/e/m/M/x/+/-/t/n/q/help)：").strip()
            if key == 'q':
                break
            elif key == 'help':
                node.show_help()
            elif key in ['w', 'a', 's', 'd', 'h', 'r', 'i', 'c', 'p', 'e', 'm', 'M', 'x', '+', '-', 't', 'n']:
                node.send_once(key)
            else:
                print("❌ 請輸入有效指令：w/a/s/d/h/r/i/c/p/e/m/M/x/+/-/t/n/q/help")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
