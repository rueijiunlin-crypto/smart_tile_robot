#!/usr/bin/env python3
# 引入必要的函式庫
import os                  # 用於檔案和目錄操作
import rclpy              # ROS 2 Python 客戶端函式庫
from rclpy.node import Node    # ROS 2 節點類別
from std_msgs.msg import String # ROS 2 字串訊息類型
import pyaudio            # 音訊處理函式庫
import wave               # WAV 檔案處理函式庫
import threading          # 執行緒處理函式庫
from datetime import datetime  # 用於日期時間處理

class AudioRecorderNode(Node):
    """
    音訊錄製節點類別
    只負責接收敲擊位置訊息並進行音訊錄製
    """

    def __init__(self):
        """
        初始化函數
        設定節點名稱、訂閱者、發布者和相關變數
        """
        super().__init__('sound_record_node')

        # 音檔計數器（從 1 開始），每錄一段自動遞增
        self.audio_index = 1

        # 錄音狀態變數
        self.is_recording = False        # 是否正在錄音
        self.recording_thread = None     # 錄音執行緒
        
        # 敲擊序列追蹤
        self.tile_index = 1              # 磁磚編號（從 1 開始）
        self.hit_position = None         # 敲擊位置：left/mid/right
        self.hit_sequence_active = False # 是否在敲擊序列中

        # 建立以日期時間命名的錄音資料夾
        now = datetime.now()
        folder_name = now.strftime("%Y%m%d_%H%M%S")  # 格式：20241209_143022
        
        # 獲取當前腳本所在目錄（ros2_ws/sound），在 sound 資料夾內創建 recordings
        script_dir = os.path.dirname(os.path.abspath(__file__))  # ros2_ws/sound 目錄
        
        self.output_folder = os.path.join(script_dir, 'recordings', folder_name)
        os.makedirs(self.output_folder, exist_ok=True)  # 如果資料夾不存在則建立
        self.get_logger().info(f'Recording folder created: {self.output_folder}')

        # 訂閱 /moveknock_locate_node 話題，收到 start/stop recording 指令
        self.subscription_knock = self.create_subscription(
            String,
            '/moveknock_locate_node',    # 訂閱敲擊位置訊息
            self.listener_knock_callback,
            10)                          # 佇列大小為 10

        # 初始化音訊參數
        self.FORMAT = pyaudio.paInt16    # 音訊格式：16位元整數
        self.CHANNELS = 1                # 單聲道
        self.RATE = 16000                # 取樣率：16kHz（與模型一致）
        self.CHUNK = 1024                # 緩衝區大小
<<<<<<< HEAD
=======
        self.RECORD_SECONDS = 1.0        # 固定錄音時長（秒，與模型一致）
>>>>>>> 413f2bd (1104更新)
        self.audio = pyaudio.PyAudio()   # 初始化PyAudio

    def start_recording(self):
        """
        開始錄音函數
        建立新的執行緒來處理錄音，避免阻塞主程式
        """
        if not self.is_recording:
            self.is_recording = True
            self.recording_thread = threading.Thread(target=self._record_audio)
            self.recording_thread.start()
            self.get_logger().info('Start recording...')

    def stop_recording(self):
        """
        停止錄音函數
        等待錄音執行緒結束，確保檔案正確寫入
        """
        if self.is_recording:
            self.is_recording = False
            if self.recording_thread:
                self.recording_thread.join()
            self.get_logger().info('Recording stopped')

    def _record_audio(self):
        """
        實際執行錄音的函數
        在獨立的執行緒中執行，避免阻塞主程式
        錄音檔名根據磁磚編號和敲擊位置命名
        """
        # 根據敲擊序列狀態決定檔名
        if self.hit_sequence_active and self.hit_position:
            filename = os.path.join(self.output_folder, f"{self.tile_index}_{self.hit_position}.wav")
        else:
            # 如果不在敲擊序列中，使用舊的命名方式
            filename = os.path.join(self.output_folder, f"{self.audio_index:05}.wav")
            self.audio_index += 1

        try:
            stream = self.audio.open(format=self.FORMAT,
                                     channels=self.CHANNELS,
                                     rate=self.RATE,
                                     input=True,
                                     frames_per_buffer=self.CHUNK)

            frames = []
            while self.is_recording:
                data = stream.read(self.CHUNK, exception_on_overflow=False)
                frames.append(data)

            stream.stop_stream()
            stream.close()

            wave_file = wave.open(filename, 'wb')
            wave_file.setnchannels(self.CHANNELS)
            wave_file.setsampwidth(self.audio.get_sample_size(self.FORMAT))
            wave_file.setframerate(self.RATE)
            wave_file.writeframes(b''.join(frames))
            wave_file.close()

            self.get_logger().info(f'Recording saved as {filename}')

        except Exception as e:
            self.get_logger().error(f"Recording error: {e}")

    def listener_knock_callback(self, msg):
        """
        處理敲擊位置訊息的回調函數
        收到 'start_recording' 字串時開始錄音，'stop_recording' 時停止
        同時解析敲擊序列狀態和位置信息
        """
        self.get_logger().info(f'Received message: {msg.data}')

        if 'start_recording' in msg.data:
            # 解析敲擊序列信息
            self._parse_hit_sequence(msg.data)
            self.start_recording()
        elif 'stop_recording' in msg.data:
            self.stop_recording()
    
    def _parse_hit_sequence(self, message):
        """
        解析敲擊序列信息，從訊息中提取磁磚編號和敲擊位置
        預期格式：start_recording:tile_1:left 或 start_recording:tile_2:mid 等
        """
        try:
            parts = message.split(':')
            if len(parts) >= 3:
                # 格式：start_recording:tile_X:position
                tile_part = parts[1]  # tile_1, tile_2, etc.
                position_part = parts[2]  # left, mid, right
                
                # 提取磁磚編號
                if tile_part.startswith('tile_'):
                    self.tile_index = int(tile_part.split('_')[1])
                
                # 設定敲擊位置
                if position_part in ['left', 'mid', 'right']:
                    self.hit_position = position_part
                    self.hit_sequence_active = True
                    self.get_logger().info(f'Recording for tile {self.tile_index}, position {self.hit_position}')
                else:
                    self.hit_sequence_active = False
            else:
                # 舊格式，不在敲擊序列中
                self.hit_sequence_active = False
                self.hit_position = None
        except Exception as e:
            self.get_logger().warn(f'Failed to parse hit sequence: {e}')
            self.hit_sequence_active = False
            self.hit_position = None

    def __del__(self):
        """
        解構函數
        確保正確關閉PyAudio，釋放資源
        """
        self.audio.terminate()

def main(args=None):
    rclpy.init(args=args)
    audio_recorder_node = AudioRecorderNode()
    try:
        rclpy.spin(audio_recorder_node)
    except KeyboardInterrupt:
        pass
    finally:
        audio_recorder_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()