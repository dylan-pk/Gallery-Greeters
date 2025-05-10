import pvporcupine as wake
import sounddevice as sd
import struct

AR_access_key = "8HEM095Qo29k5b/OQ01LPFlr+FfiUHVRi0k1N1rYnUQ2ZvZuig2zdA=="

print(wake.KEYWORDS)
handle = wake.create(access_key=AR_access_key, keywords=['jarvis'])
print("reached here, error not in creation")

audioStream = sd.RawInputStream(samplerate=handle.sample_rate, blocksize=handle.frame_length, dtype='int16', channels=1)
audioStream.start()

count = 0

def get_next_audio_frame():
    # print("in function which is just pass")
    pcm = audioStream.read(handle.frame_length)[0]
    pcm = struct.unpack_from("h" * handle.frame_length, pcm)

    return pcm

try:
    while True:
        keyword_index = handle.process(get_next_audio_frame())
        # print("after function")
        if keyword_index >= 0:
            # Insert detection event callback here
            count += 1
            print(f"detection number {count}")
            pass
        if count > 10:
            break
finally:
    audioStream.stop()
    audioStream.close()
    handle.delete()
