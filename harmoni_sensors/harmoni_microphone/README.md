# HARMONI Microphone

## Usage
Configure harfware using [the guide](https://harmoni.readthedocs.io/en/latest/configuration/Hardware.html)
Run using

```  bash
roslaunch harmoni_microphone microphone_service.launch
```
## Parameters
Parameters input for the microphone service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|audio_format_width    |            |        |
|chunk_size            |            |        |
|total_channels        |            |        |
|audio_rate            |            |        |
|device_name           |            |        |

## Testing

Module can be tested using

```  bash
rostest harmoni_microphone microphone.test
```

## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_microphone.html)