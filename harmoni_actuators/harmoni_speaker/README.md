# HARMONI Speaker


You can run this module in the `harmoni_full` or/and  `harmoni_hardware` containers.


## Usage
The speaker device is configured in the dockerfiles/config/asoundrc file, whose contents are shown below:

```
pcm.!default {
  type plug
  slave {
    pcm "hw:0,0"
  }
}
ctl.!default {
  type hw
  card 1n
}
```

The speaker device is controlled by the hw values, currently listed as 0,0. To choose a speaker you may need to use the command ```aplay -L ``` to see what devices are available.


The following documentation refers to the spaeker do.

The API for Speaker has:

- Request Name: ActionType: DO
- Body: data(str): 
  - inputting string of a dict that contains audio data {"audo_data": data(AudioData)}, where data is a binary audio data in the format of AudioData from audio_common_messages ROS library 
  - inputting string of a location of .wav file
- Response:
    - response (int): SUCCESS, or FAILURE
    - message (str): the DO action as no response message


You can run the service as follows: 
```roslaunch harmoni_speaker speaker_service.launch```

## Parameters
There are no Parameters input for the speaker service.


## Testing

To test that the speaker has been configured properly, use the command ```rostest harmoni_speaker speaker.test``` which will play a short phrase through the configured device. You may need to experiement with different values to ensure the proper speaker is set up.

## References
[Documentation](https://harmoni20.readthedocs.io/en/latest/packages/harmoni_speaker.html)
