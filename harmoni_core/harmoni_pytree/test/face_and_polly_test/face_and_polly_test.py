#!/usr/bin/env python3


PKG = "test_harmoni_polly"
# Common Imports
import unittest, rospy, roslib, sys
import traceback

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, State, DialogueNameSpace, DetectorNameSpace
import ast
import time

#py_tree
import py_trees
from harmoni_pytree.subtrees.face_and_polly import *


class TestPollyPyTree(unittest.TestCase):

    def setUp(self):

        rospy.init_node("test_speaker_and_tts", log_level=rospy.INFO)
        self.data = rospy.get_param(
            "test_polly_input"
        ) 
        # self.wav_loc = rospy.get_param(
        #     "test_speaker_input"
        # ) 
        self.data = ast.literal_eval(self.data)
        self.instance_id = rospy.get_param("instance_id")
        
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        

        # blackboard for test-to-speech data, which is updated after result is fetched
        self.blackboard_tts = py_trees.blackboard.Client(name="blackboard_tts", namespace=ActuatorNameSpace.tts.name)
        self.blackboard_tts.register_key("result", access=py_trees.common.Access.WRITE)
        # self.blackboard_tts.result = self.wav_loc

        # blackboard to store data to senf to the action server
        self.blackboard_bot = py_trees.blackboard.Client(name="blackboard_bot", namespace=DialogueNameSpace.bot.name +"/"+PyTreeNameSpace.trigger.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)
        self.blackboard_bot.result = self.data

        self.blackboard_scene = py_trees.blackboard.Client(name="blackboard_scene", namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)

        self.root = create_root()
        self.tree = py_trees.trees.BehaviourTree(self.root)
        self.tree.setup()
        self.success = True

        rospy.loginfo("Setup completed....starting test")

   
    def test_leaf_pytree_tts(self):
        rospy.loginfo(f"The input data is {self.data}")
        try:
            for unused_i in range(0, 10):
                self.root.tick_once()
                time.sleep(1)
                print("Tick number: ", unused_i)
                print("htg: ", self.blackboard_scene)

        except Exception:
            self.success = False
            print(traceback.format_exc())

        assert self.success == True

        return
    

def main():
    import rostest
    rospy.loginfo("test_polly started")
    rospy.loginfo("TestPolly: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_polly_pytree", TestPollyPyTree, sys.argv)


if __name__ == "__main__":
    main()
