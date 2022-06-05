# How to grow trees?

Ciao!!  
Constructing a leaf would certainly not satisfy our needs, and we often need to make complex behaviour trees. This can be achieved by combining leaves in one of the following ways:

1. Sequence: In a sequence composition of leaves, ticks are progressively carried out over the defined sequence untill a RUNNING or FAILURE status is returned. In case of last child, the result is adopted regarless.
2. Selector: This composite is used to prioritize leaves. The childeren are executed untill one of them succeeds. In case it runs out of children, FAILURE is returned.
3. Parallel: Leaves in this composite are ticked in parallel. This is useful for tasks which are required to be done simultaneously, like speaking and facial expressions. The return value depends on the policy used for parallelism. Details can be found [here](https://py-trees.readthedocs.io/en/devel/composites.html#:~:text=Parallels%20will%20return%20FAILURE,of%20children%20return%20SUCCESS)

Using the above composites on leaves yields an instance of `py_trees.behaviour.Behaviour` and hence the composition can be applied again. This way more complicated trees can be constructed.  
After making a composite of leaves, there are other things you can add to the behaviour tree which are not necesssary but make our lives easier, especially in debugging purposes. These are:  

1. [add_post_tick_handler()](https://py-trees.readthedocs.io/en/devel/modules.html#:~:text=instance%20of%20Behaviour-,add_post_tick_handler,-(handler)): Using this you can add a method which would be called upon every time before after a tick is made.
2. [add_pre_tick_handler()](https://py-trees.readthedocs.io/en/devel/modules.html#:~:text=func\)%20%E2%80%93%20function-,add_pre_tick_handler,-(handler)): Similar to the above method, a method can be added to the behaviour tree which would be called before a tick is made.
3. [visitors.append()](https://py-trees.readthedocs.io/en/devel/modules.html#:~:text=behaviour_tree.visitors.append): A snapshot visitor, like `py_trees.visitors.SnapshotVisitor()` visits the ticked part of a tree, checking off the status against the set of status results recorded in the previous tick.  

As a conclusion, in order to make a comple behaviour tree, the relevant leaves ae required to be defined, and then progressively composites are required to be applied for defining the decision making process. Optionally, post_tick method, pre_tick method and snapshot visiter can be used for added functionality.  

For detailed documentation click [here](https://py-trees.readthedocs.io/en/devel/composites.html). 
