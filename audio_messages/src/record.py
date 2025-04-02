# Standard libraries
import os, sys
import sounddevice as sd
import soundfile as sf
import numpy as np
assert np
from datetime import datetime
from queue import Queue
from loguru import logger
import uuid

# Initialize a queue to store audio data
audio_queue = Queue()

# Define the duration for each recording window
# Set the parameters
DURATION = 10  # seconds
CHANNELS = 1
SAMPLE_RATE = 44100

# Get the default microphone input device ID
default_input_device = sd.default.device[0]

def callback(indata:np.ndarray, frames:int, time, status) -> None:
    if status:
        print(status)
        logger.debug(f"data type: {type(indata)}")
    # print(indata)

    # Put the recorded audio data into the queue
    audio_queue.put(indata.copy())


    # Check if the queue contains enough data for 10 seconds
    total_samples = sum([len(data) for data in audio_queue.queue])
    if total_samples >= DURATION * SAMPLE_RATE:
        # Concatenate the audio data in the queue
        recorded_data = np.concatenate(list(audio_queue.queue), axis=0)

        # Generate a filename with a timestamp
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"/DATA_TEST/audio_record/recording_{current_time}.wav"
        
        # Save the recorded audio to a .wav file
        sf.write(filename, recorded_data, SAMPLE_RATE)
        logger.debug(f"Recording saved to {filename}")

        # Clear the queue for the next recording
        audio_queue.queue.clear()


for i in range(10):
    print(f"Recording {i+1}...")
    with sd.InputStream(channels=CHANNELS, samplerate=SAMPLE_RATE, callback=callback):
        sd.sleep(int(DURATION * 1000))