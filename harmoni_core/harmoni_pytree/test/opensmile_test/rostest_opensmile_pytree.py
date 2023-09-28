#!/usr/bin/env python3


PKG = "test_opensmile"

# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from std_msgs.msg import String
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, State, DetectorNameSpace
from collections import deque
import os, io
import ast
import time
#py_tree
import py_trees
from harmoni_pytree.leaves.opensmile_service import OpenSmileServicePytree

class TestOpenSmilePyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_opensmile
        """
        rospy.init_node("test_opensmile", log_level=rospy.INFO)
        
        self.instance_id = rospy.get_param("instance_id")
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
    
        self.blackboardProvaIn = py_trees.blackboard.Client(name="blackboardProva", namespace=DetectorNameSpace.opensmile.name)
        self.blackboardProvaIn.register_key("result", access=py_trees.common.Access.WRITE)
        self.blackboardProvaIn.register_key("result_message", access=py_trees.common.Access.WRITE)

        self.blackboardProvaIn.result_message = State.SUCCESS
        print(self.blackboardProvaIn)
        additional_parameters = dict([
            (DetectorNameSpace.opensmile.name,False)])   
        rospy.loginfo("--------------------"+str(additional_parameters)) 
        self.opensmilePyTree =  OpenSmileServicePytree("opensmilePyTreeTest")
        self.opensmilePyTree.setup(**additional_parameters)
        rospy.loginfo("Testopensmile: Started up. waiting for opensmile startup")
        rospy.loginfo("Testopensmile: Started")

   
    
    def test_leaf_pytree_opensmile(self):
        for unused_i in range(0, 3):
            self.opensmilePyTree.tick_once()
            time.sleep(0.5)
            print(self.blackboardProvaIn)
        print("\n")
        return
    

def main():
    import rostest
    rospy.loginfo("test_opensmile started")
    rospy.loginfo("TestOpenSmile: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_opensmile_pytree", TestOpenSmilePyTree, sys.argv)

if __name__ == "__main__":
    main()
