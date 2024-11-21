#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf
from actionlib_msgs.msg import GoalStatus

# Specific Imports
from std_msgs.msg import String, Bool
import numpy as np
import requests
from requests.exceptions import Timeout
import ast

class MistyHeadMovementService(HarmoniServiceManager):
    """
    Gesture service
    """

    def __init__(self, name):
        """ Initialization of variables and gesture parameters """
        super().__init__(name)
        self.gesture_done = False
        self.name = name
        self.service_id = hf.get_child_id(self.name)
        self.state = State.INIT
        self.actuation_completed = False
        return


    def stop(self):
        return

    def do(self, data):
        """Do the gesture

        Args:
            data (str): it could be a string of:
             - object containing {"behavior_data": str} (as results of the TTS synthetization)
             - object of: {"name": str, "timing": int}

        Returns:
            response (int): state of the DO action
        """
        rospy.loginfo("AOOOOOOOOO GOAL RECEIVED HEAD MOVEMENT DATA " + str(data))
        self.state = State.REQUEST
        self.actuation_completed = False
        if type(data) == str:
            data = ast.literal_eval(data)
        try:
            if data:
                while not self.gesture_done:
                    self.state = State.REQUEST
                    self.request_gesture_misty(data)
            self.state = State.SUCCESS
            self.gesture_done = False
            self.actuation_completed = True
            self.response_received = True

        except IOError:
            rospy.logwarn("Gesture failed")
            self.state = State.FAILED
            self.actuation_completed = True
            self.gesture_done = False
            self.response_received = True

        return {"response": self.state}

    def request_gesture_misty(self,data):
        pitch= data["pitch"]
        roll = data["roll"]
        yaw = data["yaw"]
        velocity = -1
        duration = -1
        if "velocity" in data.keys():
            velocity = data["velocity"]
        if "duration" in data.keys():
            duration = data["duration"]
        url, payload = self.move_head(pitch, roll, yaw, velocity, duration)
        if duration != -1:
            rospy.sleep(duration)
        print(url)
        print(payload)
        try:
            response = requests.post(url, 
            params = payload,
            timeout = 1)
        except Timeout:
            response = requests.post(url, 
                params = payload,
                timeout = 4)
            rospy.logwarn("Gesture failed: The ip of the robot appears unreachable")
            print(response)
        self.gesture_done = True


    def move_head(self, Pitch=0, Roll=0, Yaw=0, Velocity=-1, Duration=-1):
        payload = {'Pitch': Pitch, 
                    'Roll': Roll,
                    'Yaw': Yaw
                    }
        if Velocity != -1:
            payload['Velocity'] = Velocity
        if Duration != -1:
            payload['Duration'] = Duration
        print("AOOOOOOOOOOOOOOOOOO Sending request to move head with PAYLOAD " +str(payload))
        url = 'http://{}/api/head'.format(rospy.get_param("/robot_ip"))
        return url, payload

   

def main():
    service_name = ActuatorNameSpace.head.name
    instance_id = rospy.get_param("/instance_id")
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name)
        s = MistyHeadMovementService(service_name)
        service_server = HarmoniServiceServer(service_id, s)
        print(service_name)
        print("**********************************************************************************************")
        print(service_id)
        rospy.loginfo(f"{service_name} is ready AOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()