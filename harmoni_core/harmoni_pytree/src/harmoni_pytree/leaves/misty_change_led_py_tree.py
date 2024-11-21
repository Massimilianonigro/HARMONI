#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import rospy
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import ActionType
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.constants import ActuatorNameSpace



class MistyChangeLEDPyTree(py_trees.behaviour.Behaviour):
    #If start = False, it will not start the behavior at the first time (useful to implement thinking behavior only after the first sentence)
    def __init__(self, name, color={"red":0,"green":0,"blue":0}):
        self.name = name
        self.blackboards = []
        self.service_client_led = None
        self.color = color
        self.blackboard_head = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.led.name)
        super(MistyChangeLEDPyTree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.send_request = True

    def setup(self):
        #this is the name of the json without the extension
        self.service_client_led = HarmoniActionClient(self.name)
        self.server_name = "led_default"
        self.service_client_led.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        
    def initialise(self):
        self.logger.debug("  %s [ScriptService::initialise()]" % self.name)


    def update(self):   
        if self.send_request:
            self.send_request = False
            self.logger.debug(f"Sending goal to {self.server_name}")
            try:
                self.service_client_led.send_goal(
                    action_goal = ActionType["DO"].value,
                    optional_data=str(self.color),
                    wait=False)
                self.logger.debug(f"Goal sent to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            except Exception as e:
                rospy.loginfo("AOOOOOOOOOOOOOOOOO EXCEPTION : " + str(e))
        else:
            rospy.loginfo("CHECKING STATE")
            new_state = self.service_client_led.get_state()
            print("update : ", new_state)
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                new_status = py_trees.common.Status.SUCCESS 
            elif new_state == GoalStatus.PENDING:
                self.send_request = True
                self.logger.debug(f"Cancelling goal to {self.server_name}")
                self.service_client_led.cancel_all_goals()
                self.logger.debug(f"Goal cancelled to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                new_status = py_trees.common.Status.FAILURE
                raise
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

    def terminate(self, new_status):
        new_state = self.service_client_led.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_led.cancel_all_goals()
            self.client_result = None
            self.logger.debug(f"Goal cancelled to {self.server_name}")
            #self.service_client_speaker.stop_tracking_goal()
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