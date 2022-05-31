#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import *
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, DialogueNameSpace

#py_tree
import py_trees
import time
import json
import ast

import py_trees.console

class AWSTtsServicePytree(py_trees.behaviour.Behaviour):
    
    def __init__(self, name, test_mode=False, test_input=None):
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
        self.server_state = None
        self.service_client_tts = None
        self.client_result = None
        self.client_result_face_exp = None
        self.send_request = True

        # Initialising the blackboards
        self.blackboards = []

        # blackboard for test-to-speech data, which is updated after result is fetched
        self.blackboard_tts = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.tts.name)
        self.blackboard_tts.register_key("result", access=py_trees.common.Access.WRITE)
        
        # blackboard for facial expressions
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)

        # blackboard to store data to send to the action server(contains result from lex chatbot)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name +"/"+PyTreeNameSpace.trigger.name)
        
        # initialization of the stt keys based in test_mode and test_input values 
        if test_mode:
            self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)
            if test_input is None:
                self.blackboard_bot.result = {"message": "Hi, I'm Kitty"}
            else:
                self.blackboard_bot.result = test_input
        else:
            self.blackboard_bot.register_key("result", access=py_trees.common.Access.READ)

        super(AWSTtsServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        # @brief Setting up of action client used for sending goals to the action server. Needs
        # to called manually
        #
        # @param **additional_parameters Not used yet

        # initializing the action client used for sending goals to the harmoni action server   
        self.service_client_tts = HarmoniActionClient(self.name)
        
        # name of the server for Amazon polly service 
        self.server_name = "tts_default"
        self.service_client_tts.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))

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

            # sending action to the action server
            self.service_client_tts.send_goal(
                action_goal = ActionType["REQUEST"].value,
                optional_data = self.blackboard_bot.result["message"],
                wait=False,
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            
            # status update
            new_status = py_trees.common.Status.RUNNING
        else:
            # getting the new state 
            new_state = self.service_client_tts.get_state()
            print("update : ",new_state)

            # updating the leaf status according to the goal status
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                if self.client_result is not None:
                    
                    # updating the blackboard keys which are used by speaker and facial expression
                    self.blackboard_tts.result = self.client_result
                    self.blackboard_scene.face_exp = self.client_result_face_exp['behavior_data']
                    self.client_result = None
                    new_status = py_trees.common.Status.SUCCESS
                else:
                    self.logger.debug(f"Waiting fot the result ({self.server_name})")
                    new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.PENDING:
                # preparing to send new request
                self.send_request = True
                self.logger.debug(f"Cancelling goal to {self.server_name}")
                self.service_client_tts.cancel_all_goals()
                self.client_result = None
                self.logger.debug(f"Goal cancelled to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                new_status = py_trees.common.Status.FAILURE
                raise

        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

        
    def terminate(self, new_status):
        # @brief This function is called whenever the behaviour switches to a non-running state(SUCCESS or FAILURE or ....).
        # @param new_status The function is called with this parameter having the status of the behavior tree
        new_state = self.service_client_tts.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_tts.cancel_all_goals()
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
        self.client_result_face_exp = ast.literal_eval(result['message'])

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
    
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=DialogueNameSpace.bot.name +"/"+PyTreeNameSpace.trigger.name)
    blackboardProva.register_key("result", access=py_trees.common.Access.WRITE)
    blackboardProva.result = {
                                "message": "Hi, I'm Kitty"
                            }
    blackboardProva2 = py_trees.blackboard.Client(name="blackboardProva2", namespace=ActuatorNameSpace.tts.name)
    blackboardProva2.register_key("result", access=py_trees.common.Access.READ)                        
    print(blackboardProva)
    print(blackboardProva2)

    rospy.init_node("tts_default", log_level=rospy.INFO)
    
    ttsPyTree = AWSTtsServicePytree("AWSTtsServicePytreeTest")
    ttsPyTree.setup()
    try:
        for unused_i in range(0, 10):
            ttsPyTree.tick_once()
            time.sleep(0.5)
            print(blackboardProva)
            print(blackboardProva2)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()