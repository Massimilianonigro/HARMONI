#!/usr/bin/env python3


PKG = "test_harmoni_polly"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from std_msgs.msg import String
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, State, DialogueNameSpace, PyTreeNameSpace
from collections import deque
import os, io
import ast
import time
#py_tree
import py_trees
from harmoni_pytree.leaves.aws_tts_service import AWSTtsServicePytree


class TestPollyPyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_tts
        """
        rospy.init_node("test_polly_pytree", log_level=rospy.INFO)
        self.data = rospy.get_param(
            "test_polly_input"
        ) 
        self.data = ast.literal_eval(self.data)
        self.instance_id = rospy.get_param("instance_id")
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        
        # ex namespace: harmoni_tts
        py_trees.logging.level = py_trees.logging.Level.DEBUG
    
        self.blackboard_output_tts = py_trees.blackboard.Client(name="output_tts", namespace=ActuatorNameSpace.tts.name)
        self.blackboard_scene = py_trees.blackboard.Client(name="scene", namespace=PyTreeNameSpace.scene.name)
        self.blackboard_output_tts.register_key("result_data", access=py_trees.common.Access.READ)
        self.blackboard_output_tts.register_key("result_message", access=py_trees.common.Access.READ)
        self.blackboard_input_tts = py_trees.blackboard.Client(name="input_tts", namespace=DialogueNameSpace.bot.name+"/trigger")
        self.blackboard_input_tts.register_key("result", access=py_trees.common.Access.WRITE)
        self.blackboard_input_tts.register_key("result_message", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key = PyTreeNameSpace.scene.name + "/nlp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.scene.nlp = 0
        self.blackboard_input_tts.result_message = State.SUCCESS
        self.blackboard_input_tts.result = self.data
        print(self.blackboard_output_tts)
        print(self.blackboard_scene)
        print(self.blackboard_input_tts)

        additional_parameters = dict([
            (ActuatorNameSpace.tts.name,False)])   
        rospy.loginfo("--------------------"+str(additional_parameters)) 
        self.ttsPyTree =  AWSTtsServicePytree("ttsPyTreeTest")
        self.ttsPyTree.setup(**additional_parameters)

        rospy.loginfo("Testtts: Started up. waiting for tts startup")
        rospy.loginfo("Testtts: Started")

   
    
    def test_leaf_pytree_tts(self):
        rospy.loginfo(f"The input data is {self.data}")
        for unused_i in range(0, 4):
            self.ttsPyTree.tick_once()
            time.sleep(0.5)
            print(self.blackboard_output_tts)
        print("\n")
        return
    

def main():
    import rostest
    rospy.loginfo("test_polly started")
    rospy.loginfo("TestPolly: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_polly_pytree", TestPollyPyTree, sys.argv)


if __name__ == "__main__":
    main()
