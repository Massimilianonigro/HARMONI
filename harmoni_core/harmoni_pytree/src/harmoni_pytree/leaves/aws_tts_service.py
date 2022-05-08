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

import py_trees.console

class AWSTtsServicePytree(py_trees.behaviour.Behaviour):
    """
    This class is a child class of behaviour class of pytree module. It sends requests to harmoni action
    client and updates the status according to the goal status.
    """
    def __init__(self, name):
        self.name = name
        self.server_state = None
        self.service_client_tts = None
        self.client_result = None
        self.send_request = True

        # Initialising the blackboards
        self.blackboards = []

        # blackboard for test-to-speech data, which is updated after result is fetched
        self.blackboard_tts = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.tts.name)
        self.blackboard_tts.register_key("result", access=py_trees.common.Access.WRITE)
        
        # blackboard to store data to senf to the action server
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name +"/"+PyTreeNameSpace.trigger.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)

        super(AWSTtsServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        self.service_client_tts = HarmoniActionClient(self.name)
        self.server_name = "tts_default"
        self.service_client_tts.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        
        self.blackboard_tts.result = "null"

        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):  
        self.blackboard_bot.result = {"message": "Hi, I'm Kitty"}
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        if self.send_request:
            self.send_request = False
            self.logger.debug(f"Sending goal to {self.server_name}")
            self.service_client_tts.send_goal(
                action_goal = ActionType["REQUEST"].value,
                optional_data = self.blackboard_bot.result["message"],
                wait=False,
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            new_status = py_trees.common.Status.RUNNING
        else:
            new_state = self.service_client_tts.get_state()
            print("update : ",new_state)
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                if self.client_result is not None:
                    self.blackboard_tts.result = self.client_result
                    self.client_result = None
                    new_status = py_trees.common.Status.SUCCESS
                else:
                    self.logger.debug(f"Waiting fot the result ({self.server_name})")
                    new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.PENDING:
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
        """ Recieve and store result with timestamp """
        self.logger.debug("The result of the request has been received")
        self.logger.debug(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_result = result["message"]
        return

    def _feedback_callback(self, feedback):
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