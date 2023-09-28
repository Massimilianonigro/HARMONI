#!/usr/bin/env python3


PKG = "test_fer"

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
from harmoni_pytree.leaves.fer_service import FERServicePytree

class TestFERPyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_fer
        """
        rospy.init_node("test_fer", log_level=rospy.INFO)
        
        self.instance_id = rospy.get_param("instance_id")
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
    
        self.blackboardProvaIn = py_trees.blackboard.Client(name="blackboardProva", namespace=DetectorNameSpace.fer.name)
        self.blackboardProvaIn.register_key("result", access=py_trees.common.Access.WRITE)
        self.blackboardProvaIn.register_key("result_message", access=py_trees.common.Access.WRITE)

        self.blackboardProvaIn.result_message = State.SUCCESS
        print(self.blackboardProvaIn)
        additional_parameters = dict([
            (DetectorNameSpace.fer.name,False)])   
        rospy.loginfo("--------------------"+str(additional_parameters)) 
        self.ferPyTree =  FERServicePytree("ferPyTreeTest")
        self.ferPyTree.setup(**additional_parameters)
        rospy.loginfo("Testfer: Started up. waiting for fer startup")
        rospy.loginfo("Testfer: Started")

   
    
    def test_leaf_pytree_fer(self):
        for unused_i in range(0, 3):
            self.ferPyTree.tick_once()
            time.sleep(0.5)
            print(self.blackboardProvaIn)
        print("\n")
        return
    

def main():
    import rostest
    rospy.loginfo("test_fer started")
    rospy.loginfo("TestFER: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_fer_pytree", TestFERPyTree, sys.argv)

if __name__ == "__main__":
    main()
