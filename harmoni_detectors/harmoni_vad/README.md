# HARMONI VAD


VAD (Voice Activity Detection) is a packege which used the state-of-the-art VAD service sneakers4/silero-vad


## Setup

In order to use Voice Activity Detector import the python library by running the following commands:

```bash 
$ pip install torchaudio
$ pip install tensorflow==2.4.0
``` 


## Usage

Run the following commands in order to run microphone service and VAD service in two different terminals:

```  bash
roslaunch harmoni_sensors microphone_service.launch
roslaunch harmoni_vad vad_service.launch
```

It publish at /harmoni/detecting/vad/default a boolean (1: VAD present, 0: VAD absent)

## Testing

The module can be tested using

```  bash
rostest harmoni_vad vad.test
```

The mock audio used is harmoni_stt/test_data/hello.wav
It can be changed using the param test_vad_input