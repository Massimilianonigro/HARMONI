#!/usr/bin/env python3


PKG = "test_harmoni_face_mouth"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from std_msgs.msg import String
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, State, PyTreeNameSpace
from collections import deque
import os, io
import ast
import time
import py_trees
from harmoni_pytree.leaves.facial_exp_service import FacialExpServicePytree
from harmoni_pytree.leaves.lip_sync_service import LipSyncServicePytree



class TestFacePyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_face
        """
        
        self.data = rospy.get_param(
            "test_face_input"
        ) 
        self.instance_id = rospy.get_param("instance_id")
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        
        # ex namespace: harmoni_tts

        self.blackboard_scene = py_trees.blackboard.Client(name="scene", namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("scene/nlp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.face_exp = self.data
        self.blackboard_scene.scene.nlp = 0
        print(self.blackboard_scene)
        additional_parameters = dict([
            (ActuatorNameSpace.face.name + "_mouth",False)])   
        rospy.loginfo("--------------------"+str(additional_parameters)) 

        rospy.init_node("face_mouth_default", log_level=rospy.INFO)
        self.lips = LipSyncServicePytree("LipSyncMainActivity")
        self.facePyTree = FacialExpServicePytree("FacePyTreeTest")
        self.lips.setup(**additional_parameters)
        
        rospy.loginfo("TestFace: Started up. waiting for face startup")
        rospy.loginfo("TestFace: Started")

   
    
    def test_leaf_pytree_mouth(self):
        print("****************************** TESTING STARTED")
        rospy.loginfo(f"The input data is {self.data}")
        for unused_i in range(0, 12):
            self.lips.tick_once()
            time.sleep(0.5)
            print(self.blackboard_scene)
        print("\n")
        print("\n")
        return
    

def main():
    import rostest
    rospy.loginfo("test_face_pytree started")
    rospy.loginfo("TestFace: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_face_pytree", TestFacePyTree, sys.argv)


if __name__ == "__main__":
    main()
