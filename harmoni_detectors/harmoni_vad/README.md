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

microphone will open and one frame will be captured and put as input of the FER model. Once the output of the VAD service arrives another frame will be captured and again used as input for FER model and so on. 
