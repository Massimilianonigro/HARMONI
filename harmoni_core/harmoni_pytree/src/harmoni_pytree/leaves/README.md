# How to make leaves....grow a tree?
Ciao!!  
Leaves are components of a behaviour tree, where actual functionality of a task is defined and updates regarding an action is maintained. Essentially it is a superclass of a ```pytree``` class, ```py_trees.behaviour.Behaviour```.  
A detailed documentation of ```pytree``` can be found [here](https://py-trees.readthedocs.io/en/devel/).  
Now let's see how can we put to this to our use for creating a decision process.  
In a behaviour tree, we define something called a "tick". When a leaf is "tick"ed, it returns the status of execution od task that it is supposed to do(like a text-to-speech leaf is supposed to send a request to Amazon polly server for converting a text segment to speech). The result of the tick can be SUCCESS, FAILURE, INVALID or RUNNING. The meaning of these status can be looked up [here](https://py-trees.readthedocs.io/en/devel/behaviours.html).  
But how does the "tick"ing return the status?? We need to return the status of the task in an override function called "update". And like this function, there are other functions which are used for defining a task for leaf.  
### ```__init__()```
This function is called when creating an instance of this class. Usually, one-time initializations are included here.  
>Harmoni usually uses this for initialising blackboard and their keys.

### ```setup()```
This function is usually called manually(that is, does not get called by "tick"). Hence delayed one-time initializations are included here.  
>Harmoni uses this for setting up action client which establishes connection with the corresponding harmoni action server, for sending goals to the sever.

### ```initialise()```
Ahhh....again a method initialisation, but this is called whenever the leaf is "tick"ed. We will soon get to the ticking business, but usually those initializations are done which are required before every RUN of the task.  
>Harmoni uses this for....nothing. Yeah, most of the leaves do not use this for anything serious(just used for logging info)

### ```update()```
This is where the main business is carried out. This method is also called when the leaf is "tick"ed, so the goals of the task are sent here. The status of the task(SUCCESS, FAILURE, INVALID or RUNNING) is returned.  
**This method must be non-blocking, since immediate status of the task is required to be returned.**
>Harmoni uses this method to send goals to the action server using the action client. The result callback and feedback callback of the action client are used for updating the state of variables according to the result. Also the state of the task is also maintained by fetching the state of the goal given to the server.

### ```terminate()```
This method is called when a "tick" returns a non-RUNNING state, which are SUCCESS, FAILURE, INVALID.
>Harmoni uses this for setting up new goal for the harmoni action server. All the previous goals sent to the server are cancelled in case there are any pending goals.  

Therefore, to construct a leaf with a specific functionality, the above functions should be defined appropitely in a class having `py_trees.behaviour.Behaviour` as a parent class.

For composing the these basic units(leaves) to make a complex behaviour tree, have a look at `README.md` of subtrees.