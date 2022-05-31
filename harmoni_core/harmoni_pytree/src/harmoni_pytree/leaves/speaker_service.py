#!/usr/bin/env python3

# Common Imports
import rospy, rospkg, roslib

from harmoni_common_lib.constants import *
from harmoni_common_lib.action_client import HarmoniActionClient
from actionlib_msgs.msg import GoalStatus

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType
import boto3

import time


#py_tree
import py_trees

class SpeakerServicePytree(py_trees.behaviour.Behaviour):
    """
    This class is a child class of behaviour class of pytree module. It sends requests to harmoni action
    client and updates the leaf status according to the goal status.
    """

    def __init__(self, name = "SpeakerServicePytree", test_mode=False, test_input=None):
        # @brief Constructor for initializing blackboard and their keys
        #
        # @param name Name of the pytree
        # 
        # @param test_mode The mode of running the leaf. If set to true, 
        # blackboard keys are given WRITE access for initialization with a value. 
        #
        # @param test_input The input to the blackboard keys for testing the leaf. If None,
        # then deafult value is given to the blackboard keys which will be used as test input.         
        
        # Attribute initialization
        self.name = name
        self.service_client_speaker = None
        self.client_result = None
        self.server_state = None
        self.server_name = None
        self.send_request = True

        # list of blackboards
        self.blackboards = []

        # blackboard to store text-to-speech data
        self.blackboard_tts = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.tts.name)
        
        # initialization of the tts key based in test_mode and test_input values 
        if test_mode:
            self.blackboard_tts.register_key("result", access=py_trees.common.Access.WRITE)
            if test_input is None:
                self.blackboard_tts.result = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"
            else:
                self.blackboard_tts.result = test_input
        else:
            self.blackboard_tts.register_key("result", access=py_trees.common.Access.READ)

        super(SpeakerServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        # @brief Setting up of action client used for sending goals to the action server. Needs
        # to called manually
        #
        # @param **additional_parameters Not used yet
        self.service_client_speaker = HarmoniActionClient(self.name)
        self.server_name = "speaker_default"
        self.service_client_speaker.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
     
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        # initializing the action client used for sending goals to the harmoni action server
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
    
    def update(self):
        # @brief This is called every time the behaviour tree is ticked. Sending of request to the action server is done here.
        # further status of the goal is updated here.

        # if request is to be sent
        if self.send_request:
            self.send_request = False
            self.logger.debug(f"Sending goal to {self.server_name}")

            # sending the goal to action server
            self.service_client_speaker.send_goal(
                action_goal = ActionType["DO"].value,
                optional_data=self.blackboard_tts.result,
                wait=True,
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            new_status = py_trees.common.Status.RUNNING
        else:
            # updating the leaf state based on the goal status
            new_state = self.service_client_speaker.get_state()
            print(new_state)
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                new_status = py_trees.common.Status.SUCCESS
            else:
                new_status = py_trees.common.Status.FAILURE
                raise
        
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status
        

    def terminate(self, new_status):
        # @brief This function is called whenever the behaviour switches to a non-running state(SUCCESS or FAILURE or ....).
        # @param new_status The function is called with this parameter having the status of the behavior tree
        new_state = self.service_client_speaker.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_speaker.cancel_all_goals()
            self.client_result = None
            self.logger.debug(f"Goal cancelled to {self.server_name}")
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

    def _result_callback(self, result):
        # @brief This function is called when the action client receives a reult of the goal sent. Update 
        # of client_result takes place here which is used for updating blackboard key
        #
        # @param result Contains the result of the goal sent by the client from harmoni action server
        #
        """ Recieve and store result with timestamp """
        self.logger.debug("The result of the request has been received")
        self.client_result = result
        return

    def _feedback_callback(self, feedback):
        # @brief This function is called by action client when it receives feedback from the action server
        # @param feeedback Contains the feedback sent by the harmoni action server.
        """ Feedback is currently just logged """
        self.logger.debug("The feedback recieved is %s." % feedback)
        self.server_state = feedback["state"]
        return


def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=ActuatorNameSpace.tts.name)
    blackboardProva.register_key(key="result", access=py_trees.common.Access.WRITE)
    blackboardProva.result = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"
    print(blackboardProva)
    print(blackboardProva.result)
 
    rospy.init_node("speaker_default", log_level=rospy.INFO)
    
    speakerPyTree = SpeakerServicePytree("SpeakerServicePytreeTest")
    speakerPyTree.setup()
    try:
        for unused_i in range(0, 10):
            speakerPyTree.tick_once()
            time.sleep(0.5)
            print(blackboardProva)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()