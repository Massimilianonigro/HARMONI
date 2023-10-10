# Launching a Service 

## Basics

Launching services in HARMONI Follows the standard ROS convention:
```bash
roslaunch harmoni_$PACKAGE $PACKAGE_service.launch
```
For example:
```bash
roslaunch harmoni_tts tts_service.launch
```
Launching packages this way will allow you to verify if the package is correctly setup and installed. However to fully test a package, look at the "Running a test" Section. To use a package, you will need to interact with the service it exposes.

## Launching Services in Docker
See the Quickstart Docker section for details on getting started with docker. Once you have docker set up and are inside the container, you can run services as shown above.
Most services can be run in the 'full' container, however detectors often come with conflicting requirements. Therefore we choose to create seperate images for these modules, and run them in their own images.  
If you want to run a detector locally, use the same command but in the corresponding container. For example, for the stt package, use the ros_w2l container for running the script.  
Similarly, if there are seperate computers controlling the robot hardware, you may wish to have a seperate image for running on that computer. This is often named 'harmoni_hardware'.

