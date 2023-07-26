#!/usr/bin/env python3


PKG = "test_harmoni_mic_stt_bot_tts_speaker"
# Common Imports
import unittest, rospy, roslib, sys
import traceback

# Specific Imports
from harmoni_common_lib.constants import *
import ast
import time

#py_tree
import py_trees
from harmoni_pytree.subtrees.mic_stt_bot_tts_speaker import *


class TestMicSttBotTtsSpeakerPyTree(unittest.TestCase):

    def setUp(self):

        rospy.init_node("test_mic_stt", log_level=rospy.INFO)
        self.instance_id = rospy.get_param("instance_id")
        
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        
        
        blackboard_tts = py_trees.blackboard.Client(name="blackboard_tts", namespace=ActuatorNameSpace.tts.name)
        blackboard_tts.register_key("result", access=py_trees.common.Access.READ) 
        
        blackboard_scene = py_trees.blackboard.Client(name="blackboard_scene", namespace=PyTreeNameSpace.scene.name)
        blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/nlp", access=py_trees.common.Access.READ)
        blackboard_scene.scene.nlp = 1
        blackboard_scene.scene.request = "Hello?"
        blackboard_scene.scene.utterance = "Hello"

        blackboard_bot = py_trees.blackboard.Client(name="blackboard_bot", namespace=DialogueNameSpace.bot.name +"/"+PyTreeNameSpace.trigger.name)
        blackboard_bot.register_key("result", access=py_trees.common.Access.READ) 
        
        blackboard_stt = py_trees.blackboard.Client(name="blackboardProva", namespace=DetectorNameSpace.stt.name)
        blackboard_stt.register_key("result", access=py_trees.common.Access.READ)

        self.root = create_root()
        self.tree = py_trees.trees.BehaviourTree(self.root)
        self.tree.setup(timeout=15)
        self.success = False

        rospy.loginfo("Setup completed....starting test")

   
    def test_mic_stt(self):
        rospy.loginfo(f"Starting to tick")
        try:
            for unused_i in range(0, 10):
                self.tree.tick()
                time.sleep(1)
                print(self.blackboard_stt)
                print("Tick number: ", unused_i)
        
        except Exception:
            assert self.root.status==py_trees.common.Status.SUCCESS
            print(traceback.format_exc())

        assert self.root.status==py_trees.common.Status.SUCCESS

        return
    

def main():
    import rostest
    rospy.loginfo("test_mic_stt started")
    rospy.loginfo("TestMicSttBotTtsSpeaker: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_mic_stt_pytree", TestMicSttBotTtsSpeakerPyTree, sys.argv)


if __name__ == "__main__":
    main()