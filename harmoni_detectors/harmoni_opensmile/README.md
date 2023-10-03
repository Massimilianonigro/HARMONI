# HARMONI opensmile



## Setup
Install opensmile using the following command in harmoni_detectors/harmoni_opensmile
```  bash
pip install -r requirements.txt
```

## Usage

Run the following commands :

```  bash
roslaunch harmoni_opensmile opensmile_service.launch
```

## Testing

The module can be tested using

```  bash
rostest harmoni_opensmile opensmile.test
```

The mock audio used is harmoni_stt/test_data/hello.wav
It can be changed using the param test_opensmile_input

