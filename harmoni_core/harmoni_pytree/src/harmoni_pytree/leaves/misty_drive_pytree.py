#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import rospy
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import ActionType
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.constants import PyTreeNameSpace, ActuatorNameSpace
import random
import rospkg
import json

#At the moment Chooses a random drive movement between the list, (used to implement thinking behavior)
#TODO: Make it possible to go through the list of movements in sequence
class MistyDriveMovementServicePyTree(py_trees.behaviour.Behaviour):
    def __init__(self, name, movements=[[]],random=False, script_gesture=False,gesture_filename=""):
        self.name = name
        self.blackboards = []
        self.service_client_drive_movement = None
        self.default_movements = movements
        self.movements = []
        #If script_gesture == True, it will display the drive movements given the gestures in the script
        self.script_gesture = script_gesture
        self.gesture_filename = gesture_filename
        #If random == True, it will display a random gesture between the ones in the movement list
        #If random == False, it will display them in order
        self.random = random
        self.current_mov_idx = -1 
        self.reset_gesture_list = False
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key("drive", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key("drive_list", access=py_trees.common.Access.WRITE)
        super(MistyDriveMovementServicePyTree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.send_request = True

    def setup(self):
        if self.script_gesture:
            rospack = rospkg.RosPack()
            self.gesture_file = json.load(open(rospack.get_path("harmoni_pytree") + "/resources/gestures/"+self.gesture_filename + ".json","r"))
        #this is the name of the json without the extension
        self.service_client_drive_movement = HarmoniActionClient(self.name)
        self.server_name = "drive_default"
        self.service_client_drive_movement.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        
    def initialise(self):
        self.logger.debug("  %s [ScriptService::initialise()]" % self.name)
        

    def get_movements_from_gesture_list(self):
        movements_unrolled = []
        for gesture in self.blackboard_scene.drive_list:
            if gesture != "" and gesture in self.gesture_file.keys():
                gesture_movements = self.gesture_file[gesture]
                for movement in gesture_movements:
                    movements_unrolled.append(movement)
        return movements_unrolled
    
    def _get_movement_from_script(self):
        new_status = None
        if self.blackboard_scene.drive == "" and self.blackboard_scene.drive_list == []:
            new_status = py_trees.common.Status.SUCCESS
        if self.blackboard_scene.drive not in self.gesture_file.keys() and self.blackboard_scene.drive_list == []:
            self.logger.debug("---------------- Gesture not found in the gesture file")
            new_status = py_trees.common.Status.FAILURE
        rospy.loginfo("Gesture found in the gesture file")
        if self.blackboard_scene.drive_list != []:
            rospy.loginfo("OOOOOOOOOOOOOOOOOOO INSIDE BRANCH GESTURE LIST")
            self.movements = self.get_movements_from_gesture_list()
            if self.movements == []:
                new_status = py_trees.common.Status.SUCCESS
            self.reset_gesture_list = True
        else:
            rospy.loginfo("OOOOOOOOOOOOOOOOOOO INSIDE BRANCH GESTURE")
            self.movements = self.gesture_file[self.blackboard_scene.drive]
        rospy.loginfo("OOOOOOOOOO Curr_movement is " + str(self.movements) + " and gesture is " + str(self.blackboard_scene.drive))     
        return new_status          

    def update(self):   
        if self.script_gesture and self.movements == []:
            #Changes self.movement to the list of movements from the script
            new_status = self._get_movement_from_script()
            if new_status != None:
                return new_status
        if self.random and self.movements == []:
            self.movements = random.choice(self.default_movements)
        
        if self.send_request:
            self.send_request = False
            self.logger.debug(f"Sending goal to {self.server_name}")
            try:
                rospy.loginfo("OOOOOOOOOOOOOOO Curr_movement is " + str(self.movements[self.current_mov_idx]))
                rospy.loginfo("OOOOOOOOOOOOOOO Curr MOVEMENT IDX is " + str(self.current_mov_idx) + " and len is " + str(len(self.movements)))
                if self.current_mov_idx < len(self.movements) - 1:
                    self.current_mov_idx += 1
                    movement = self.movements[self.current_mov_idx]
                    self.service_client_drive_movement.send_goal(
                        action_goal = ActionType["DO"].value,
                        optional_data=str(movement),
                        wait=False)
                    self.logger.debug(f"Goal sent to {self.server_name}")
                    new_status = py_trees.common.Status.RUNNING
                else:
                    new_status = py_trees.common.Status.SUCCESS
                self.logger.debug(f"Goal sent to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            except Exception as e:
                rospy.loginfo("AOOOOOOOOOOOOOOOOO EXCEPTION : " + str(e))
        else:
            rospy.loginfo("CHECKING STATE")
            new_state = self.service_client_drive_movement.get_state()
            print("update : ", new_state)
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                if self.current_mov_idx == len(self.movements) - 1:
                    self.send_request = False
                    self.movements = []
                    self.current_mov_idx = -1
                    if self.reset_gesture_list:
                        self.reset_gesture_list = False
                        self.blackboard_scene.drive_list = []
                    new_status = py_trees.common.Status.SUCCESS
                else:
                    new_status = py_trees.common.Status.RUNNING
                    self.send_request = True
            elif new_state == GoalStatus.PENDING:
                self.send_request = True
                self.logger.debug(f"Cancelling goal to {self.server_name}")
                self.service_client_drive_movement.cancel_all_goals()
                self.logger.debug(f"Goal cancelled to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                new_status = py_trees.common.Status.FAILURE
                raise
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status
    
    def terminate(self, new_status):
        new_state = self.service_client_drive_movement.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_drive_movement.cancel_all_goals()
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