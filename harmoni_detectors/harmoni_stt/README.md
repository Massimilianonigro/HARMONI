# HARMONI STT
This is a module for transcribing the speech from audio files and audio streaming. 
You can run your STT in the `harmoni_full` container.

## Usage

### Local DeepSpeech STT
Using the DeepSpeech service:
To set up the local STT service, first run `sh harmoni_detectors/harmoni_stt/get_deepspeech_models.sh` 
from the HARMONI directory in order to place the models in a parallel directory.


The API for Local DeepSpeech STT has:
- Request Name: ActionType: REQUEST
- Body: None (the STT is already listening from the microphone)
- Response:
    - response (int): SUCCESS, or FAILURE 
    - message (str): text transcribed from the streaming audio or audio file


The local DeepSpeech speech-to-text service can be launched with `roslaunch harmoni_stt stt_deepspeech_service.launch` or `roslaunch harmoni_stt stt_service.launch service_to_launch:=deepspeech`.
Transcriptions are only published by the DeepSpeech service when the client determines the text as final based on the `t_wait` parameter (the default is 0.5s).

### Google STT

The API for Google STT has:
- Request Name: ActionType: REQUEST
- Body: None (the STT is already listening from the microphone)
- Response:
    - response (int): SUCCESS, or FAILURE 
    - message (str): text transcribed from the streaming audio or audio file


You can run the service with the following command:
```roslaunch harmoni_stt stt_google_service.launch```

## Parameters

### Local DeepSpeech STT
Parameters input for the local STT service:
| Parameters           | Definition | Values |
|----------------------|------------|--------|
|model_file_path       |   path of the local STT model         |  str; e.g., "$(find harmoni_models)/stt/deepspeech-0.9.3-models.pbmm"      |
|scorer_path           |  path of the scorer for the deepspeech model          |  str; e.g., "$(find harmoni_models)/stt/deepspeech-0.9.3-models.scorer"       |
|lm_alpha              |   parameters of the deepspeech model         |   int; 0.75      |
|lm_beta               |   parameters of the deepspeech model           |   int; 1.85     |
|beam_width            |    width of them beam        | int; 700       |
|t_wait                |    seconds to wait of silence before stoping transcribing        | int; 3s       |
|subscriber_id         |    id of the subscriber        |  e.g., "default"      |

### Google STT

Parameters input for the Google STT service:

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|language_id       |  language of the audio file    |  str; "en-US"    |
|sample_rate           |  sample rate of the audio file (it should match with the microphone one in case of streaming)        |  int; e.g., 48000, or 44100     |
|audio_channel              |   number of audio channels   |   int; 1      |
|max_duration               |   maximum duration of empty streaming (seconds)           |   int; 30   |
|waiting_time            |   time of silence to wait after stopping the transcription (seconds)     | int; 2       |
|credential_path                |   path where private keys are mounted        | str; "$(env HOME)/.gcp/private-keys.json/private-keys.json"      |
|subscriber_id         |    id of the subscriber        |  e.g., "default"      |


## Testing

Local DeepSpeech STT module can be tested using

```  bash
rostest harmoni_stt deepspeech.test
```

Online Gooogle STT module can be tested using

```  bash
rostest harmoni_stt google.test

## References
[Documentation](https://harmoni20.readthedocs.io/en/latest/packages/harmoni_stt.html)

https://trac.ffmpeg.org/wiki/Capture/ALSA
