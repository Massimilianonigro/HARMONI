import pyaudio
import wave
 
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 48000
DOWNSAMPLED_RATE = 16000
CHUNK = 1024*3
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "recordedFile.wav"
device_index = 0
audio = pyaudio.PyAudio()

devinfo = audio.get_device_info_by_index(0)  # Or whatever device you care about.
if audio.is_format_supported(RATE,  # Sample rate
                         input_device=devinfo['index'],
                         input_channels=devinfo['maxInputChannels'],
                         input_format=pyaudio.paInt16):
  print ('Yay!')



print("----------------------record device list---------------------")
info = audio.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')
for i in range(0, numdevices):
        if (audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            print("Input Device id ", i, " - ", audio.get_device_info_by_host_api_device_index(0, i).get('name'))

print("-------------------------------------------------------------")

index = int(input())
print("recording via index "+str(index))

stream = audio.open(format=FORMAT, channels=CHANNELS,
                rate=RATE, input=True,input_device_index = index,
                frames_per_buffer=CHUNK)
print ("recording started")
Recordframes = []

for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK, exception_on_overflow = False)
    # print(len(data))
    Recordframes.append(data)
print ("recording stopped")
 
stream.stop_stream()
stream.close()
audio.terminate()

print(audio.get_sample_size(FORMAT))

waveFile = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
waveFile.setnchannels(CHANNELS)
waveFile.setsampwidth(audio.get_sample_size(FORMAT))
waveFile.setframerate(RATE)
waveFile.writeframes(b''.join(Recordframes))
waveFile.close()

# downsampling to 16k sample_rate
from pydub import AudioSegment as am
sound = am.from_file("recordedFile", format='wav', frame_rate=RATE)
sound = sound.set_frame_rate(DOWNSAMPLED_RATE)
sound.export("recordedFile_16k.wav", format='wav')
