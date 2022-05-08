#!/usr/bin/env python3


PKG = "test_lex"

# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from std_msgs.msg import String
from harmoni_common_lib.constants import State, DialogueNameSpace
import time
#py_tree
import py_trees
from harmoni_pytree.leaves.aws_lex_analyzer_service import AWSLexAnalyzerServicePytree
from harmoni_pytree.leaves.aws_lex_trigger_service import AWSLexTriggerServicePytree

class TestLexPyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_bot
        """
        rospy.init_node("test_lex", log_level=rospy.INFO)
        self.data = rospy.get_param(
            "test_bot_input"
        ) 
        self.instance_id = rospy.get_param("instance_id")
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
    
        self.blackboardProvaIn = py_trees.blackboard.Client(name="blackboardProva", namespace=DialogueNameSpace.bot.name)
        self.blackboardProvaIn.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboardProvaIn.register_key("result_message", access=py_trees.common.Access.WRITE)
        self.blackboardProvaOut = py_trees.blackboard.Client(name="blackboardProva", namespace=DialogueNameSpace.bot.name + "output")
        self.blackboardProvaOut.register_key("result_data", access=py_trees.common.Access.READ)
        self.blackboardProvaOut.register_key("result_message", access=py_trees.common.Access.READ)

        self.blackboardProvaIn.result_message = State.SUCCESS
        self.blackboardProvaIn.result_data = self.data
        print(self.blackboardProvaIn)
        print(self.blackboardProvaOut)
        additional_parameters = dict([
            (DialogueNameSpace.bot.name,False)])   
        rospy.loginfo("--------------------"+str(additional_parameters)) 
        self.botPyTree_analyzer = AWSLexAnalyzerServicePytree("botPyTree_analyzer_Test")
        self.botPyTree_analyzer.setup(**additional_parameters)
        self.botPyTree_trigger = AWSLexTriggerServicePytree("botPyTree_trigger_Test")
        self.botPyTree_trigger.setup(**additional_parameters)
        rospy.loginfo("Testbot: Started up. waiting for bot startup")
        rospy.loginfo("Testbot: Started")

   
    
    def test_leaf_pytree_bot(self):
        rospy.loginfo(f"The input data is {self.data}")
        for unused_i in range(0, 3):
            self.botPyTree_analyzer.tick_once()
            time.sleep(0.5)
            print(self.blackboardProvaIn)
            print(self.blackboardProvaOut)
        print("\n")
        return
    

def main():
    import rostest
    rospy.loginfo("test_lex started")
    rospy.loginfo("TestLex: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_lex_pytree", TestLexPyTree, sys.argv)

if __name__ == "__main__":
    main()
