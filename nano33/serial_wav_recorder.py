#!/usr/bin/env python3
import os
import wave
import serial
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime


class Nano33SerialRecorder(Node):
    """
    從 Nano33 串列接收音訊封包並寫入 WAV；
    透過 /moveknock_locate_node 的 start_recording/stop_recording 控制。
    封包格式：b"AUD0" + uint16(len) + 原始 16-bit little-endian PCM
    """

    HEADER = b"AUD0"

    def __init__(self,
                 serial_port: str = "/dev/ttyACM0",
                 baud: int = 2000000,
                 sample_rate: int = 22050):
        super().__init__('nano33_serial_recorder')

        self.serial_port = serial_port
        self.baud = baud
        self.sample_rate = sample_rate

        # 錄音狀態
        self.is_recording = False
        self.wave_file = None
        self.ser = None

        # 建立錄音資料夾（沿用 sound_record_1.py 的路徑慣例）
        now = datetime.now()
        folder_name = now.strftime("%Y%m%d_%H%M%S")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # 儲存在 ros2_ws/sound/recordings 下，若不存在則回退到當前目錄 recordings
        sound_dir = os.path.join(os.path.dirname(script_dir), 'ros2_ws', 'sound', 'recordings')
        base_dir = sound_dir if os.path.isdir(sound_dir) or self._safe_makedirs(sound_dir) else os.path.join(script_dir, 'recordings')
        self.output_folder = os.path.join(base_dir, folder_name)
        os.makedirs(self.output_folder, exist_ok=True)
        self.get_logger().info(f'Recording folder: {self.output_folder}')

        # 訂閱觸發話題
        self.subscription_knock = self.create_subscription(
            String,
            '/moveknock_locate_node',
            self.knock_callback,
            10
        )

        # 以 timer 輪詢串列資料（非阻塞）
        self.timer = self.create_timer(0.005, self.poll_serial)

    def _safe_makedirs(self, path: str) -> bool:
        try:
            os.makedirs(path, exist_ok=True)
            return True
        except Exception:
            return False

    def open_serial(self):
        if self.ser and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=0)
            self.get_logger().info(f'Opened serial {self.serial_port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Open serial failed: {e}')
            self.ser = None

    def close_serial(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def start_recording(self, filename_hint: str = None):
        if self.is_recording:
            return
        self.open_serial()
        if not self.ser:
            self.get_logger().error('Serial not available; cannot start recording')
            return

        # 建立檔名
        if filename_hint and filename_hint.endswith('.wav'):
            filename = os.path.join(self.output_folder, filename_hint)
        else:
            # 預設名
            filename = os.path.join(self.output_folder, 'nano33_record.wav')

        try:
            self.wave_file = wave.open(filename, 'wb')
            self.wave_file.setnchannels(1)
            self.wave_file.setsampwidth(2)  # 16-bit
            self.wave_file.setframerate(self.sample_rate)

            # 清空輸入緩衝
            self.ser.reset_input_buffer()
            # 發 START
            self.ser.write(b'START\n')
            self.is_recording = True
            self.get_logger().info(f'Start recording -> {filename}')
        except Exception as e:
            self.get_logger().error(f'Open wave failed: {e}')
            self.wave_file = None

    def stop_recording(self):
        if not self.is_recording:
            return
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b'STOP\n')
        except Exception:
            pass
        finally:
            self.is_recording = False
            if self.wave_file:
                try:
                    self.wave_file.close()
                except Exception:
                    pass
                self.wave_file = None
            self.get_logger().info('Recording stopped')

    def poll_serial(self):
        if not (self.is_recording and self.ser and self.ser.is_open):
            return
        try:
            # 嘗試讀 header + 長度
            if self.ser.in_waiting < 6:
                return
            hdr = self.ser.read(6)
            if len(hdr) != 6 or hdr[:4] != self.HEADER:
                # 同步丟失，嘗試丟到下一個 'A' 開頭
                return
            length = hdr[4] | (hdr[5] << 8)
            if length <= 0 or length > 4096:
                return
            payload = self.ser.read(length)
            if len(payload) != length:
                return
            if self.wave_file:
                self.wave_file.writeframes(payload)
        except Exception as e:
            self.get_logger().warn(f'serial read warning: {e}')

    def knock_callback(self, msg: String):
        data = msg.data
        if 'start_recording' in data:
            # 解析檔名提示（若有）
            filename = None
            parts = data.split(':')
            if len(parts) >= 3 and parts[0] == 'start_recording' and parts[1].startswith('tile_'):
                try:
                    tile_index = int(parts[1].split('_')[1])
                    position = parts[2]
                    if position in ('left', 'mid', 'right'):
                        filename = f"tile_{tile_index}_{position}.wav"
                except Exception:
                    pass
            self.start_recording(filename)
        elif 'stop_recording' in data:
            self.stop_recording()


def main():
    parser = argparse.ArgumentParser(description='Nano33 Serial WAV Recorder')
    parser.add_argument('--port', dest='port', default=os.environ.get('NANO_PORT', '/dev/ttyACM0'), help='Serial port, e.g., /dev/ttyACM0')
    parser.add_argument('--baud', dest='baud', type=int, default=int(os.environ.get('NANO_BAUD', 2000000)), help='Baud rate (default 2000000)')
    parser.add_argument('--sample-rate', dest='sample_rate', type=int, default=int(os.environ.get('NANO_SR', 22050)), help='Sample rate (default 22050)')
    # 允許 ROS2 自身參數繼續傳遞
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Nano33SerialRecorder(serial_port=args.port, baud=args.baud, sample_rate=args.sample_rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


