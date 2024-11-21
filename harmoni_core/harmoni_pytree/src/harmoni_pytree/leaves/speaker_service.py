#!/usr/bin/env python3

# Common Imports
import rospy

from harmoni_common_lib.constants import *
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, PyTreeNameSpace
from harmoni_common_lib.action_client import HarmoniActionClient
from actionlib_msgs.msg import GoalStatus

import time
import py_trees

class SpeakerServicePytree(py_trees.behaviour.Behaviour):

    def __init__(self, name = "SpeakerServicePytree"):
        self.name = name
        self.service_client_speaker = None
        self.client_result = None
        self.server_state = None
        self.server_name = None
        self.send_request = True

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_input = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.tts.name)
        self.blackboard_input.register_key("result", access=py_trees.common.Access.WRITE)
        self.blackboard_output = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.speaker.name)
        self.blackboard_output.register_key("result", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="nlp", access=py_trees.common.Access.READ)
        super(SpeakerServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        self.service_client_speaker = HarmoniActionClient(self.name)
        self.server_name = "speaker_default"
        self.service_client_speaker.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
     
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
    
    def update(self):
        if self.blackboard_scene.nlp == 2:
            new_status = py_trees.common.Status.SUCCESS
        else:  
            if self.send_request:
                self.send_request = False
                self.logger.debug(f"Sending goal to {self.server_name}")
                rospy.logdebug("\n\n--------------------SENDING REQUESTO TO --------:" + str(self.server_name))

                self.service_client_speaker.send_goal(
                    action_goal = ActionType["DO"].value,
                    optional_data=self.blackboard_input.result,
                    wait=False,
                )
                self.logger.debug(f"Goal sent to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                new_state = self.service_client_speaker.get_state()
                rospy.logdebug("\n\n--------------------Speaker State is --------:" + str(new_state))
                if new_state == GoalStatus.ACTIVE:
                    new_status = py_trees.common.Status.RUNNING
                elif new_state == GoalStatus.SUCCEEDED:
                    new_status = py_trees.common.Status.SUCCESS
                else:
                    new_status = py_trees.common.Status.FAILURE
                    raise
            self.blackboard_output.result = new_status
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status
        

    def terminate(self, new_status):
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
            #self.service_client_speaker.stop_tracking_goal()
            #self.logger.debug(f"Goal tracking stopped to {self.server_name}")
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        self.logger.debug("The result of the request has been received")
        self.client_result = result
        return

    def _feedback_callback(self, feedback):
        """ Feedback is currently just logged """
        self.logger.debug("The feedback recieved is %s." % feedback)
        self.server_state = feedback["state"]
        return

def main():
    #command_line_argument_parser().parse_args()

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    blackboard_scene = py_trees.blackboard.Client(name=PyTreeNameSpace.scene.name, namespace=PyTreeNameSpace.scene.name)
    blackboard_scene.register_key("nlp", access=py_trees.common.Access.WRITE)
    blackboard_scene.nlp = 0
    blackboard_input = py_trees.blackboard.Client(name=ActuatorNameSpace.tts.name, namespace=ActuatorNameSpace.tts.name)
    blackboard_input.register_key("result", access=py_trees.common.Access.WRITE)
    blackboard_input.result = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"
    print(blackboard_input)

    rospy.init_node("speaker_default", log_level=rospy.INFO)
    
    speakerPyTree = SpeakerServicePytree("SpeakerServicePytreeTest")
    speakerPyTree.setup()
    try:
        for unused_i in range(0, 5):
            speakerPyTree.tick_once()
            time.sleep(0.5)
            print(blackboard_input)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()