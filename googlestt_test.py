
# Imports the Google Cloud client library
from google.cloud import speech
import os

os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/root/.gcp/private-keys.json"


# Instantiates a client
client = speech.SpeechClient()

# The name of the audio file to transcribe
gcs_uri = "/root/harmoni_catkin_ws/src/HARMONI/gstt_test.wav"

audio = speech.RecognitionAudio(uri=gcs_uri)

config = speech.RecognitionConfig(
    encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
    sample_rate_hertz=48000,
    language_code="en-US",
)

# Detects speech in the audio file
response = client.recognize(config=config, audio=audio)

for result in response.results:
    print("Transcript: {}".format(result.alternatives[0].transcript))