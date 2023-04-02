# HARMONI FER


FER (facial expression recognition) is a packege which used the state-of-the-art FaceChannel service to detect Arousal and Valence, find more details in: https://facechannel.readthedocs.io/en/latest/


## Setup

In order to use FaceChannel import the python library by running the following commands:

```bash 
$ pip install facechannel
``` 


## Usage

Run the following commands in order to run camera service and FER service in two different terminals:

```  bash
roslaunch harmoni_sensors camera_service.launch
roslaunch harmoni_fer fer_service.launch
```

It is a detector that subscribe from a camera sensor (either the robots or an external camera) (Image) data, and it publish two set of String:
- /harmoni/detecting/fer/default: returning arousal, valence 
- /harmoni/detecting/fer/default/baseline: retuning valence max and valence min of the first 60 seconds of the interaction (to get a baseline)

## Testing

Camera will open and one frame will be captured and put as input of the FER model. Once the output of the FER service arrives another frame will be captured and again used as input for FER model and so on. 
