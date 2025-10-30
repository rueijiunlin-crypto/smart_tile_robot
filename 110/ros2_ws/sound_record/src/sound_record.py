#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave

class AudioRecorderNode(Node):

    def __init__(self):
        super().__init__('sound_record_node')

        # Initialize variables for filename construction
        self.x1 = 1
        self.x2 = -1  # Start at -1 so the first message makes it 0
        self.x3 = 1   # Start at 1
        self.s_count = 0  # Counter for consecutive 's' messages

        # Subscription to /moveknock_locate_node
        self.subscription_knock = self.create_subscription(
            String,
            '/moveknock_locate_node',
            self.listener_knock_callback,
            10)

        # Subscription to /move2
        self.subscription_move = self.create_subscription(
            String,
            '/move2',
            self.listener_move_callback,
            10)

        self.publisher = self.create_publisher(String, '/recording_status', 10)

    def listener_move_callback(self, msg):
        # Check for 's' messages and update x1 after 6 consecutive 's' messages
        if msg.data == 's':
            self.s_count += 1
            if self.s_count == 6:
                self.x1 += 1
                self.s_count = 0  # Reset count after reaching 8
        else:
            self.s_count = 0  # Reset count if any other message is received

    def listener_knock_callback(self, msg):
        # Increment x2 and handle reset and increment of x3
        if self.x2 == 2:  # Reset x2 after it reaches 2
            self.x2 = 0
            self.x3 += 1
        else:
            self.x2 += 1

        # Log the received message and initiate recording
        self.get_logger().info(f'Received message: {msg.data}')
        self.record_audio()

    def record_audio(self):
        self.get_logger().info('Starting audio recording...')
        
        # PyAudio setup
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 44100
        CHUNK = 1024
        RECORD_SECONDS = 3
        WAVE_OUTPUT_FILENAME = f"{self.x1}_{self.x3}_{self.x2}.wav"  # Custom naming logic
        
        audio = pyaudio.PyAudio()

        stream = audio.open(format=FORMAT, channels=CHANNELS,
                            rate=RATE, input=True,
                            frames_per_buffer=CHUNK)

        frames = []

        for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK)
            frames.append(data)

        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # Save the recording
        waveFile = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        waveFile.setnchannels(CHANNELS)
        waveFile.setsampwidth(audio.get_sample_size(FORMAT))
        waveFile.setframerate(RATE)
        waveFile.writeframes(b''.join(frames))
        waveFile.close()

        self.get_logger().info(f'Recording finished and saved as {WAVE_OUTPUT_FILENAME}.')

        # Publish "Recording finished" message
        self.publish_finished_message()

    def publish_finished_message(self):
        msg = String()
        msg.data = 'Recording finished'
        self.publisher.publish(msg)
        self.get_logger().info('Published: "Recording finished"')

def main(args=None):
    rclpy.init(args=args)
    audio_recorder_node = AudioRecorderNode()
    rclpy.spin(audio_recorder_node)
    audio_recorder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
