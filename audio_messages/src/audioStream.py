# Standard libraries
import os, sys
import sounddevice as sd
import soundfile as sf
import numpy as np
assert np
from datetime import datetime
import time
from queue import Queue
from loguru import logger
import uuid

# ROS2 libraries
import rclpy
from rclpy.node import Node
from audio_messages.msg import AudioInfo, AudioData


class AudioPublisher(Node):

    def __init__(self):
        super().__init__('audio_publisher')
        self.publisher_ = self.create_publisher(AudioData, 'audio_data', 10) # Publish audio data
        self.info_publisher_ = self.create_publisher(AudioInfo, 'audio_info', 10) # Publish audio info
        timer_period = 5 # every 5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.audio_queue = Queue() # Initialize a queue to store audio data
        self.DURATION:float = 5 # Define the duration for each recording window in seconds
        self.CHANNELS:int = 1
        self.SAMPLE_RATE:int = 16000

        # Configure loguru to write logs to a file
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        logger.add(f"logfile_{current_time}.txt")


        # Get the default microphone input device ID
        self.default_input_device = sd.default.device[0]
        logger.debug(f"Default input device: {self.default_input_device}")

        # Indicate the creation of the node
        logger.debug("Node created")

    def sounddevice_callback(self, indata:np.ndarray, frames:int, time, status) -> None:
        try:
            if status:
                logger.debug(status)
                logger.debug(f"data type: {type(indata)}")
            # print(indata)

            # Put the recorded audio data into the queue
            self.audio_queue.put(indata.copy())


            # Check if the queue contains enough data for X seconds
            total_samples = sum([len(data) for data in self.audio_queue.queue])
            if total_samples >= self.DURATION * self.SAMPLE_RATE:
                # Concatenate the audio data in the queue
                recorded_data = np.concatenate(list(self.audio_queue.queue), axis=0)

                # Generate a filename with a timestamp
                current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                unique_id = uuid.uuid4()  # Generate a random UUID
                filename = "your_audiofile_{current_time}_{unique_id}.wav"

                # Save the recorded audio to a .wav file
                sf.write(filename, recorded_data, self.SAMPLE_RATE)
                logger.debug(f"Recording saved to {filename}")

                # Clear the queue for the next recording
                self.audio_queue.queue.clear()

                # Publish the audio info
                info_msg = AudioInfo()
                info_msg.num_channels = int(self.CHANNELS)
                info_msg.sample_rate = int(self.SAMPLE_RATE)
                info_msg.subtype = 'float32'
                info_msg.uuid = str(unique_id)
                self.info_publisher_.publish(info_msg)
                
                time.sleep(0.1)

                # Publish the recorded audio data
                msg = AudioData()
                msg.data = recorded_data.astype(np.float32).tolist()
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published audio data identified by time: {current_time}")
        except Exception as e:
            logger.error(f"Error at sounddevice callback: {e}")

    def timer_callback(self):
        try:
            # Start recording
            with sd.InputStream(device=self.default_input_device, channels=self.CHANNELS, callback=self.sounddevice_callback):
                logger.debug(f"Recording for {self.DURATION} seconds")
                sd.sleep(int(self.DURATION * 1000))
        except Exception as e:
            logger.error(f"Error at timer callback: {e}")
            
def main(args=None):
    try:
        rclpy.init(args=args)
        audio_publisher = AudioPublisher()
        rclpy.spin(audio_publisher)
        audio_publisher.destroy_node()
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        rclpy.shutdown()    

class AudioSubscriber(Node):

    def __init__(self):
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(AudioData, 
                                                     'audio_data', 
                                                     self.audio_callback, 
                                                     10)
        self.subscription_info = self.create_subscription(AudioInfo, 
                                                          'audio_info', 
                                                          self.audio_info_callback, 
                                                          10)
        self.subscription # prevent unused variable warning

        # Configure Loguru to set default verbosity to INFO
        logger.remove()  # Remove the default handler
        logger.add(sys.stderr, level="INFO")  # Add a handler to stderr with INFO level


        # Configure loguru to write logs to a file
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        logger.add(f"logfile_{current_time}.txt")
        
        # Sleep 2 seconds for launch file
        time.sleep(5)
        logger.info("Wait 2 seconds for launch file")
        

    def audio_callback(self, msg:AudioData) -> None:
        logger.debug(f"Received audio data: {msg.data}")
        self.get_logger().info(f"Received audio data")

    def audio_info_callback(self, msg:AudioInfo) -> None:
        logger.info(f"Received audio info: {msg.uuid}")
        self.get_logger().info(f"Received audio info: {msg.uuid}")

def main(args=None):
    rclpy.init(args=args)

    audio_subscriber = AudioSubscriber()

    rclpy.spin(audio_subscriber)

    audio_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()