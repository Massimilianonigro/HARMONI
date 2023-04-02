# HARMONI Custom Detector



## Setup

Dowload the custom model from the HARMONI path:
```  bash
sh ./harmoni_detectors/harmoni_detcustom/get_ir_custom_model.sh
```

## Usage

Run the following commands in order to run camera service and custom service :

```  bash
roslaunch harmoni_detcustom detcustom_service.launch
```

While in the harmoni_full_detector container, you can run opneface service:

```  bash
roslaunch harmoni_openface openface_service.launch
```

It returns a boolean with interaction rupture dection (1: present, 0: absent )

## Testing


