#!/usr/bin/env python3

# Common Imports
import rospy
from harmoni_common_lib.constants import *
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient
import harmoni_common_lib.helper_functions as hf
from harmoni_bot.chatgpt_service import ChatGPTService

# Specific Imports
from harmoni_common_lib.constants import ActionType, DialogueNameSpace

#py_tree
import py_trees
import time
import py_trees.console

class ChatGPTServicePytree(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.name = name
        self.server_state = None
        self.service_client_chatgpt = None
        self.client_result = None
        self.send_request = True

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/utterance", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/nlp", access=py_trees.common.Access.READ)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name+"/"+PyTreeNameSpace.trigger.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)

        super(ChatGPTServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        self.service_client_chatgpt = HarmoniActionClient(self.name)
        self.server_name = "bot_default"
        self.service_client_chatgpt.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        
        self.blackboard_bot.result = "null"

        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):   
        if self.blackboard_scene.scene.nlp:           
            if self.send_request:
                self.send_request = False
                rospy.loginfo("The utterance is " + str(self.blackboard_scene.scene.utterance))
                self.logger.debug(f"Sending goal to {self.server_name}")
                self.service_client_chatgpt.send_goal(
                    action_goal = ActionType["REQUEST"].value,
                    optional_data=self.blackboard_scene.scene.utterance,
                    wait=False,
                )
                self.logger.debug(f"Goal sent to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                new_state = self.service_client_chatgpt.get_state()
                print("update : ", new_state)
                if new_state == GoalStatus.ACTIVE:
                    new_status = py_trees.common.Status.RUNNING
                elif new_state == GoalStatus.SUCCEEDED:
                    if self.client_result is not None:
                        rospy.loginfo("________________________The client results is " +str(self.client_result))
                        self.blackboard_bot.result = self.client_result
                        self.blackboard_bot.result  = {
                                                                "message":   self.client_result
                                            }
                        self.client_result = None
                        new_status = py_trees.common.Status.SUCCESS
                    else:
                        self.logger.debug(f"Waiting fot the result ({self.server_name})")
                        new_status = py_trees.common.Status.RUNNING
                elif new_state == GoalStatus.PENDING:
                    self.send_request = True
                    self.logger.debug(f"Cancelling goal to {self.server_name}")
                    self.service_client_chatgpt.cancel_all_goals()
                    self.client_result = None
                    self.logger.debug(f"Goal cancelled to {self.server_name}")
                    new_status = py_trees.common.Status.RUNNING
                else:
                    new_status = py_trees.common.Status.FAILURE
                    raise
        else:
            new_status = py_trees.common.Status.SUCCESS
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

        

    def terminate(self, new_status):
        new_state = self.service_client_chatgpt.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_chatgpt.cancel_all_goals()
            self.client_result = None
            self.logger.debug(f"Goal cancelled to {self.server_name}")
            #self.service_client_chatgpt.stop_tracking_goal()
            #self.logger.debug(f"Goal tracking stopped to {self.server_name}")
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        self.logger.debug("The result of the request has been received")
        self.logger.debug(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_result = result['message']
        return

    def _feedback_callback(self, feedback):
        """ Feedback is currently just logged """
        self.logger.debug("The feedback recieved is %s." % feedback)
        self.server_state = feedback["state"]
        return

