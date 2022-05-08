#!/usr/bin/env python3


PKG = "test_face_pytree"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, State

import time
#py_tree
import py_trees
from harmoni_pytree.leaves.facial_exp_service import FacialExpServicePytree


class TestFacePyTree(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_face
        """
        rospy.init_node("test_face_pytree", log_level=rospy.INFO)
        self.data = rospy.get_param(
            "test_face_input"
        ) 
        self.instance_id = rospy.get_param("instance_id")
        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        
        # ex namespace: harmoni_tts
        self.blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=ActuatorNameSpace.tts.name)
        self.blackboardProva.register_key("result_data", access=py_trees.common.Access.WRITE)
        self.blackboardProva.register_key("result_message", access=py_trees.common.Access.WRITE)

        self.blackboardProva.result_message = State.SUCCESS
        self.blackboardProva.result_data = self.data

        additional_parameters = dict([
            (ActuatorNameSpace.face.name,True)])
        rospy.loginfo("--------------------"+str(additional_parameters)) 
        self.facePyTree =  FacialExpServicePytree("facePyTreeTest")
        self.facePyTree.setup(**additional_parameters)

        self.result = True

        rospy.loginfo("TestFace: Started up. waiting for face startup")
        rospy.loginfo("TestFace: Started")

   
    
    def test_leaf_pytree_mouth(self):
        rospy.loginfo(f"The input data is {self.data}")
        try:
            self.facePyTree.tick_once()
            print(self.blackboardProva)
            print("\n")

        except Exception as e:
            self.result = False

        assert self.result == True
    

def main():
    import rostest
    rospy.loginfo("test_face started")
    rospy.loginfo("TestFace: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_face_pytree", TestFacePyTree, sys.argv)


if __name__ == "__main__":
    main()
