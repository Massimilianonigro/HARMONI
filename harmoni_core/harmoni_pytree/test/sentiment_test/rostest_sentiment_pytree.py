#!/usr/bin/env python3


PKG = "test_harmoni_sentiment"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from std_msgs.msg import String
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, State
from collections import deque
import os, io
import ast
import time
#py_tree
import py_trees
from harmoni_pytree.leaves.sentiment_service import SentimentServicePytree


class TestSentimentPyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_sentiment
        """
        rospy.init_node("test_sentiment_pytree", log_level=rospy.INFO)
        self.data = rospy.get_param(
            "test_sentiment_input"
        ) 
        self.instance_id = rospy.get_param("instance_id")
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        
        # ex namespace: harmoni_tts
        self.blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=ActuatorNameSpace.sentiment.name)
        self.blackboardProva.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboardProva.register_key("result_message", access=py_trees.common.Access.WRITE)

        self.blackboardProva.result_message = State.SUCCESS
        self.blackboardProva.result_data = self.data

        additional_parameters = dict([
            (ActuatorNameSpace.sentiment.name,False)])   
        rospy.loginfo("--------------------"+str(additional_parameters)) 
        self.sentimentPyTree =  SentimentServicePytree("sentimentPyTreeTest")
        self.sentimentPyTree.setup(**additional_parameters)

        rospy.loginfo("Testsentiment: Started up. waiting for sentiment startup")
        rospy.loginfo("Testsentiment: Started")

   
    
    def test_leaf_pytree_sentiment(self):
        rospy.loginfo(f"The input data is {self.data}")
        for unused_i in range(0, 4):
            self.sentimentPyTree.tick_once()
            time.sleep(0.5)
            print(self.blackboardProva)
        print("\n")
        return
    

def main():
    import rostest
    rospy.loginfo("test_sentiment started")
    rospy.loginfo("Testsentiment: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_sentiment_pytree", TestSentimentPyTree, sys.argv)


if __name__ == "__main__":
    main()
