# Running Individual Packages

## Running a Package in HARMONI
If you want to run a single package, you can directly launch the node from the command line as shown in the [Launching a Service page](Launching-Services).
```
roslaunch harmoni_microphone microphone_service.launch
```


### Playing a behaviour tree

In harmoni we seek to provide a single point of truth for the configuration of the services. Rather than generating multiple launch files for each configuration set, we use the python api for roslaunch to dynamically launch files based on the current configuration. 


## Running Packages Alone

If you want to run an harmoni_client instantiated as a py_trees leaf, you can use the following command:
```
roslaunch harmoni_pytree microphone_service.launch
```




