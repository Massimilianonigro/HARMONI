#!/usr/bin/env python3

# Common Imports
import rospy
from harmoni_common_lib.constants import *
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient

# Specific Imports
from harmoni_common_lib.constants import ActionType, DialogueNameSpace

#py_tree
import py_trees
import time

import py_trees.console

class AWSLexTriggerServicePytree(py_trees.behaviour.Behaviour):
    def __init__(self, name):
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
        self.service_client_lex = None
        self.client_result = None
        self.send_request = True

        self.blackboards = []
        
        # blackboard for storing the utterance of the speaker
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.WRITE)
        
        # blackboard for storing the result from lex chatbot    
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name+"/"+PyTreeNameSpace.trigger.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)

        super(AWSLexTriggerServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        # @brief Setting up of action client used for sending goals to the action server. Needs
        # to called manually
        #
        # @param **additional_parameters Not used yet   
        
        # initializing the action client used for sending goals to the harmoni action server
        self.service_client_lex = HarmoniActionClient(self.name)
        
        # name of the server for lex chatbot         
        self.server_name = "bot_default"
        self.service_client_lex.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        
        self.blackboard_bot.result = "null"

        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):              
        # @brief This is called every time the behaviour tree is ticked. Sending of request to the action server is done here.
        # further status of the goal is updated here.        
        if self.send_request:
            self.send_request = False
            self.logger.debug(f"Sending goal to {self.server_name}")
            self.service_client_lex.send_goal(
                action_goal = ActionType["REQUEST"].value,
                optional_data=self.blackboard_scene.utterance,
                wait=False,
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            
            # status update
            new_status = py_trees.common.Status.RUNNING
        else:
            # if request is not be send then fetching the new state
            new_state = self.service_client_lex.get_state()
            print("update : ",new_state)
            
            # updating the new_status based in new_state
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                if self.client_result is not None:
                    #updating the value of the key of backboard
                    self.blackboard_bot.result = eval(self.client_result)
                    
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
                self.service_client_lex.cancel_all_goals()
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
        new_state = self.service_client_lex.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_lex.cancel_all_goals()
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
        self.client_result = result['message']
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
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=PyTreeNameSpace.scene.name)
    blackboardProva.register_key("utterance", access=py_trees.common.Access.WRITE)
    blackboardProva.utterance = "domanda raccolta"
    blackboardProva2 = py_trees.blackboard.Client(name="blackboardProva2", namespace=DialogueNameSpace.bot.name+"/"+PyTreeNameSpace.trigger.name)
    blackboardProva2.register_key("result", access=py_trees.common.Access.READ)                        
    print(blackboardProva)
    print(blackboardProva2)

    rospy.init_node("bot_default", log_level=rospy.INFO)
    
    awslexPyTree = AWSLexTriggerServicePytree("AWSLexTriggerServicePytreeTest")
    awslexPyTree.setup()
    try:
        for unused_i in range(0, 10):
            awslexPyTree.tick_once()
            time.sleep(0.5)
            print(blackboardProva)
            print(blackboardProva2)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()
