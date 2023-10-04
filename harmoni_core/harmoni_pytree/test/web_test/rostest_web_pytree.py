#!/usr/bin/env python3


PKG = "test_harmoni_web"
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
from harmoni_pytree.leaves.web_service import WebServicePytree


class TestWebPyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_web
        """
        rospy.init_node("test_web_pytree", log_level=rospy.INFO)
        self.data = rospy.get_param(
            "test_web_input"
        ) 
        self.instance_id = rospy.get_param("instance_id")
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
    
        self.blackboard_output_web = py_trees.blackboard.Client(name="output_web", namespace=ActuatorNameSpace.web.name)
        self.blackboard_output_web.register_key("result_message", access=py_trees.common.Access.READ)
        self.blackboard_output_web.register_key("result_data", access=py_trees.common.Access.READ)

        self.blackboard_input_web = py_trees.blackboard.Client(name="input_web", namespace="input_web")
        self.blackboard_input_web.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboard_input_web.register_key("result_message", access=py_trees.common.Access.WRITE)

        self.blackboard_scene = py_trees.blackboard.Client(name="scene", namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key("image", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.image =  self.data
        self.blackboard_input_web.result_message = State.SUCCESS
        self.blackboard_input_web.result_data = self.data

        additional_parameters = dict([
            (ActuatorNameSpace.web.name,False)])   
        rospy.loginfo("--------------------"+str(additional_parameters)) 
        self.webPyTree =  WebServicePytree("webPyTreeTest")
        self.webPyTree.setup(**additional_parameters)

        rospy.loginfo("Testweb: Started up. waiting for web startup")
        rospy.loginfo("Testweb: Started")

   
    
    def test_leaf_pytree_web(self):
        rospy.loginfo(f"The input data is {self.data}")
        for unused_i in range(0, 4):
            self.webPyTree.tick_once()
            time.sleep(2)
            print(self.blackboard_output_web)
            print(self.blackboard_input_web)
        print("\n")
        return
    

def main():
    import rostest
    rospy.loginfo("test_web started")
    rospy.loginfo("TestWeb: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_web_pytree", TestWebPyTree, sys.argv)


if __name__ == "__main__":
    main()
