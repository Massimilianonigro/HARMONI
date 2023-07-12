#!/usr/bin/env python3


PKG = "test_harmoni_camera"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from std_msgs.msg import String
from harmoni_common_lib.constants import SensorNameSpace, ActionType, State
from collections import deque
import os, io
import ast
import time
#py_tree
import py_trees
from harmoni_pytree.leaves.camera_service import CameraServicePytree


class TestCameraPyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_camera
        """
        rospy.init_node("test_camera_pytree", log_level=rospy.INFO)
        
        self.instance_id = rospy.get_param("instance_id")
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        
        # ex namespace: harmoni_tts
        self.blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=SensorNameSpace.camera.name)
        self.blackboardProva.register_key("result_message", access=py_trees.common.Access.WRITE)

        self.blackboardProva.result_message = State.SUCCESS

        additional_parameters = dict([
            (SensorNameSpace.camera.name,False)])   
        rospy.loginfo("--------------------"+str(additional_parameters)) 
        self.cameraPyTree =  CameraServicePytree("cameraPyTreeTest")
        self.cameraPyTree.setup(**additional_parameters)

        rospy.loginfo("Testcamera: Started up. waiting for camera startup")
        rospy.loginfo("Testcamera: Started")

   
    
    def test_leaf_pytree_camera(self):
        for unused_i in range(0, 4):
            self.cameraPyTree.tick_once()
            time.sleep(0.5)
            print(self.blackboardProva)
        print("\n")
        return
    

def main():
    import rostest
    rospy.loginfo("test_camera started")
    rospy.loginfo("Testcamera: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_camera_pytree", TestCameraPyTree, sys.argv)


if __name__ == "__main__":
    main()
