#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave
import threading
from datetime import datetime


class AudioRecorderNode(Node):

    def __init__(self):
        super().__init__('sound_record_node')

        self.audio_index = 1
        self.is_recording = False
        self.recording_thread = None

        self.tile_index = 1
        self.hit_position = None
        self.hit_sequence_active = False

        # 建立啟動時間資料夾
        now = datetime.now()
        folder_name = now.strftime("%Y%m%d_%H%M%S")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_folder = os.path.join(script_dir, 'recordings', folder_name)
        os.makedirs(self.output_folder, exist_ok=True)

        self.get_logger().info(f"Recording folder created: {self.output_folder}")

        # ROS2 訂閱
        self.subscription_knock = self.create_subscription(
            String,
            '/moveknock_locate_node',
            self.listener_knock_callback,
            10
        )

        # 音訊參數
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 44100
        self.CHUNK = 1024
        self.RECORDING_DURATION = 1.4  # 錄音時長（秒）- 對應模型要求的 61740 個樣本
        self.audio = pyaudio.PyAudio()

    # -------------------------------------------------------------
    #  開始 & 停止錄音
    # -------------------------------------------------------------
    def start_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.recording_thread = threading.Thread(target=self._record_audio)
            self.recording_thread.start()
            self.get_logger().info("Start recording...")

    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            if self.recording_thread:
                self.recording_thread.join()
            self.get_logger().info("Recording stopped")

    # -------------------------------------------------------------
    #  實際錄音
    # -------------------------------------------------------------
    def _record_audio(self):
        # 檔名決定方式
        if self.hit_sequence_active and self.hit_position:
            filename = os.path.join(self.output_folder, f"{self.tile_index}_{self.hit_position}.wav")
        else:
            filename = os.path.join(self.output_folder, f"{self.audio_index:05}.wav")
            self.audio_index += 1

        try:
            stream = self.audio.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK
            )

            frames = []
            # 計算需要錄製的 CHUNK 數量
            total_chunks = int(self.RATE / self.CHUNK * self.RECORDING_DURATION)
            # 計算剩餘樣本數（用於最後一個不完整的 CHUNK）
            remaining_samples = int(self.RATE * self.RECORDING_DURATION) % self.CHUNK
            
            self.get_logger().info(f"開始錄音，目標時長: {self.RECORDING_DURATION} 秒 ({total_chunks} 個完整 CHUNK + {remaining_samples} 個樣本)")
            self.get_logger().info(f"注意：錄音將固定錄製 {self.RECORDING_DURATION} 秒，不受 stop_recording 訊息影響")

            # 錄製完整的 CHUNK（固定時長，不受 is_recording 標誌影響）
            for i in range(total_chunks):
                data = stream.read(self.CHUNK, exception_on_overflow=False)
                frames.append(data)

            # 錄製剩餘的樣本（如果有的話）
            if remaining_samples > 0:
                data = stream.read(remaining_samples, exception_on_overflow=False)
                frames.append(data)
            
            # 錄音完成後，重置 is_recording 標誌
            self.is_recording = False

            stream.stop_stream()
            stream.close()

            # 存成原始 WAV
            with wave.open(filename, 'wb') as wf:
                wf.setnchannels(self.CHANNELS)
                wf.setsampwidth(self.audio.get_sample_size(self.FORMAT))
                wf.setframerate(self.RATE)
                wf.writeframes(b''.join(frames))

            # 計算實際錄製的樣本數
            total_samples = sum(len(frame) // 2 for frame in frames)  # 每個樣本是 2 bytes (Int16)
            actual_duration = total_samples / self.RATE
            
            self.get_logger().info(f"錄音完成: {filename}, 實際時長: {actual_duration:.3f} 秒 ({total_samples} 個樣本)")

        except Exception as e:
            self.get_logger().error(f"Recording error: {e}")

    # -------------------------------------------------------------
    #  ROS2 訊息處理
    # -------------------------------------------------------------
    def listener_knock_callback(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")

        if 'start_recording' in msg.data:
            self._parse_hit_sequence(msg.data)
            self.start_recording()

        elif 'stop_recording' in msg.data:
            self.stop_recording()

    # -------------------------------------------------------------
    #  敲擊序列解析
    # -------------------------------------------------------------
    def _parse_hit_sequence(self, message):
        try:
            parts = message.split(':')
            if len(parts) >= 3:
                tile_part = parts[1]
                position = parts[2]

                if tile_part.startswith("tile_"):
                    self.tile_index = int(tile_part.split('_')[1])

                if position in ['left', 'mid', 'right']:
                    self.hit_position = position
                    self.hit_sequence_active = True
                    self.get_logger().info(f"Recording for tile {self.tile_index}, position {self.hit_position}")
                else:
                    self.hit_sequence_active = False
            else:
                self.hit_sequence_active = False
                self.hit_position = None

        except Exception as e:
            self.get_logger().warn(f"Failed to parse hit sequence: {e}")
            self.hit_sequence_active = False
            self.hit_position = None

    def __del__(self):
        self.audio.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = AudioRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
