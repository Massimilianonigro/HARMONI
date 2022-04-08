For the impementation of waiting for response the deepspeech service is modified to publish an integer to denote the number of times it has processed a chunk to null(empty) word. This published msg is subscribed by a node in harmoni_pytree, namely, local_dialogues. There is also a json file in the subtrees folder(the same directory), which has three simple sentences.
local_dialogue.py fetches the count msg from te rostopic, and if the count is a multiple of 10 then it modifies a variable of blackboard to the sentence(which is to be spoken by bot).

There is a launch file created to run the bot: "first_interaction_htg.launch", present in harmoni_pytree/launch/subtrees.

For speech-to-text translation to properly work, the microphone should preferably work at 16kHz.
