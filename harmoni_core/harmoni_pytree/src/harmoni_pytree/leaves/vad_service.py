#!/usr/bin/env python3

# Common Imports
import rospy

from harmoni_common_lib.constants import *
from harmoni_common_lib.constants import DetectorNameSpace, ActionType
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient

import py_trees
import time

import py_trees.console

class VADServicePytree(py_trees.behaviour.Behaviour):

    def __init__(self, name = "VADServicePytree"):
        self.name = name
        self.server_state = None
        self.service_client_vad = None
        self.client_result = None
        self.server_name = None
        self.send_request = True

        self.blackboards = []
        self.blackboard_vad = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.vad.name)
        self.blackboard_vad.register_key("result", access=py_trees.common.Access.WRITE)
        super(VADServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):    
        self.service_client_vad = HarmoniActionClient(self.name)
        self.server_name = DetectorNameSpace.vad.value + "default"
        self.blackboard_vad.result = str(False)
        self.service_client_vad.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback,
                                            wait = True)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        self.service_client_vad.send_goal(
                action_goal = ActionType.ON,
                optional_data="Setup",
                wait=False,
            )
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        rospy.loginfo("DETECTOR NAME IS" + str(DetectorNameSpace.vad.name))
        if self.send_request:
            self.send_request = False
            self.logger.debug(f"Sending goal to {self.server_name}")
            self.service_client_vad.send_goal(
                action_goal = ActionType["REQUEST"].value,
                wait=False
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            new_status = py_trees.common.Status.RUNNING
        else:
            new_state = self.service_client_vad.get_state()
            rospy.loginfo("VAD SERVICE STATE IS " + str(new_state))
            rospy.loginfo(new_state)
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                self.send_request = True
                self.blackboard_vad.result = self.client_result  
                new_status = py_trees.common.Status.SUCCESS
            elif new_state == GoalStatus.PENDING:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.ABORTED:
                new_status = py_trees.common.Status.SUCCESS
            else:
                new_status = py_trees.common.Status.FAILURE
            self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status
        
    def terminate(self, new_status):
        rospy.loginfo("INTO VAD TERMINATE")
        new_state = self.service_client_vad.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            rospy.loginfo("BEFORE CANCELLING GOALS")
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            #self.service_client_service_client_vad.cancel_all_goals()
            self.service_client_vad.stop_tracking_goal()

            rospy.loginfo("AFTER CANCELLING GOALS")

            self.client_result = None
            self.logger.debug(f"Goal cancelled to {self.server_name}")
            #self.logger.debug(f"Goal tracking stopped to {self.server_name}")
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
        self.server_state = feedback
        return

def main():
    #command_line_argument_parser().parse_args()

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    #rospy init node mi fa diventare un nodo ros
    rospy.init_node("vad_default", log_level=rospy.INFO)

    blackboard_output = py_trees.blackboard.Client(name=DetectorNameSpace.vad.name, namespace=DetectorNameSpace.vad.name)
    blackboard_output.register_key("result", access=py_trees.common.Access.READ)
    print(blackboard_output)

    vadPyTree = VADServicePytree("VADServicePytreeTest")

    additional_parameters = dict([
        ("VADServicePytree_mode",False)])

    vadPyTree.setup(**additional_parameters)
    try:
        for unused_i in range(0, 5):
            vadPyTree.tick_once()
            time.sleep(1)
            print(blackboard_output)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()