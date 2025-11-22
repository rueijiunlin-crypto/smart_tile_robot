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
            while self.is_recording:
                data = stream.read(self.CHUNK, exception_on_overflow=False)
                frames.append(data)

            stream.stop_stream()
            stream.close()

            # 存成原始 WAV
            with wave.open(filename, 'wb') as wf:
                wf.setnchannels(self.CHANNELS)
                wf.setsampwidth(self.audio.get_sample_size(self.FORMAT))
                wf.setframerate(self.RATE)
                wf.writeframes(b''.join(frames))

            self.get_logger().info(f"Recording saved as {filename}")

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
