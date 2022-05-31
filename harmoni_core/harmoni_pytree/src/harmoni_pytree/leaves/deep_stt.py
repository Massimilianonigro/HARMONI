#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import *
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient

# Specific Imports
from harmoni_common_lib.constants import DetectorNameSpace, ActionType
import boto3

#py_tree
import py_trees
import time

import py_trees.console

class DeepSpeechToTextServicePytree(py_trees.behaviour.Behaviour):

    def __init__(self, name = "DeepSpeechToTextServicePytree"):
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
        self.service_client_stt = None
        self.client_result = None
        self.server_state = None
        self.server_name = None
        self.send_request = True
        
        self.blackboards = []
        
        # blackboard for storing microphone data
        self.blackboard_microphone = self.attach_blackboard_client(name=self.name, namespace=SensorNameSpace.microphone.name)
        self.blackboard_microphone.register_key("state", access=py_trees.common.Access.READ)
        
        # blackboard for storing the text obtained from deep speech to text translation
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.WRITE)

        super(DeepSpeechToTextServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        # @brief Setting up of action client used for sending goals to the action server. Needs
        # to called manually
        #
        # @param **additional_parameters Not used yet
        
         # initializing the action client used for sending goals to the harmoni action server
        self.service_client_stt = HarmoniActionClient(self.name)
        
        # name of the server for deep stt model
        self.server_name = "stt_default"
        self.service_client_stt.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))

        self.blackboard_stt.result = "null"

        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        # @brief Does nothing relating to behaviour trees. This runs the first time your behaviour is ticked and anytime the
        # status is not RUNNING thereafter.        
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        # @brief This is called every time the behaviour tree is ticked. Sending of request to the action server is done here.
        # further status of the goal is updated here.
        if self.send_request:
            self.send_request = False
            self.logger.debug(f"Sending goal to {self.server_name}")
            print("htg: ", "deep speech sending goal")
            
            # the goal to send is of type ON
            self.service_client_stt.send_goal(
                action_goal = ActionType["ON"].value,
                optional_data="",
                wait=False,
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            
            # status update
            new_status = py_trees.common.Status.RUNNING
        else:
            # if request is not be send then fetching the new state
            new_state = self.service_client_stt.get_state()
            print(new_state)
            
            # updating the new_status based in new_state
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                if self.client_result is not None:
                    print("htg: ", "changing stt blackboard")
                    
                    # updating the value of blackboard key to store the result of harmoni action server
                    self.blackboard_stt.result = self.client_result
                    
                    # setting the client_result to None again before fetching new value
                    self.client_result = None
                    new_status = py_trees.common.Status.SUCCESS
                else:
                    self.logger.debug(f"Waiting fot the result ({self.server_name})")
                    new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.PENDING:
                # preparing to send new request
                self.send_request = True
                self.logger.debug(f"Cancelling goal to {self.server_name}")
                self.service_client_stt.cancel_all_goals()
                self.client_result = None
                self.logger.debug(f"Goal cancelled to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                # raising exception in case of failure
                new_status = py_trees.common.Status.FAILURE
                print("htg: ")
                raise
        
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status
        
    def terminate(self, new_status):
        # @brief This function is called whenever the behaviour switches to a non-running state(SUCCESS or FAILURE or ....).
        # @param new_status The function is called with this parameter having the status of the behavior tree 
        new_state = self.service_client_stt.get_state()
        print("terminate : ",new_state)
        if new_status == py_trees.common.Status.INVALID:
            if new_state == GoalStatus.ACTIVE:
                self.send_request = True
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_stt.cancel_all_goals()
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
        self.logger.debug(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_result = result["message"]
        print("htg: ", result)
        return

    def _feedback_callback(self, feedback):
        # @brief This function is called by action client when it receives feedback from the action server
        # @param feeedback Contains the feedback sent by the harmoni action server.
        """ Feedback is currently just logged """
        self.logger.debug("The feedback recieved is %s." % feedback)
        self.server_state = feedback["state"]
        return

def main():
    #command_line_argument_parser().parse_args()

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=DetectorNameSpace.stt.name)
    blackboardProva.register_key("result", access=py_trees.common.Access.READ)
    print(blackboardProva)

    rospy.init_node("deep_stt_default", log_level=rospy.INFO)
    
    sttPyTree = DeepSpeechToTextServicePytree("DeepSSTPytreeTest")

    sttPyTree.setup()
    try:
        for unused_i in range(0, 20):
            sttPyTree.tick_once()
            time.sleep(2)
            print(blackboardProva)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()
