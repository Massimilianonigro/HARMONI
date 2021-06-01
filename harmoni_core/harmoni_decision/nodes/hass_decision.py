#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

import sys
# print(sys.path)
sys.path.insert(0, "/root/harmoni_catkin_ws/src/HARMONI/harmoni_core/harmoni_pattern/nodes/")
# print(sys.path)

from sequential_pattern import SequentialPattern
# from harmoni_pattern.sequential_pattern import SequentialPattern

# Specific Imports
import rospkg
import json
import inspect

from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import DetectorNameSpace, ActionType, ActuatorNameSpace
from collections import deque
from time import time
import threading

# CHECK if needed
import logging
import ast
import os


class HomeAssistantDecisionManager(HarmoniServiceManager):
    """Instantiates behaviors and receives commands/data for them.
    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name,pattern_list, instance_id ,test_input):
        super().__init__(name)
        self.name = name
        self.service_id = instance_id
        
        self.scripted_services = pattern_list
        # setup_service = pattern_list[0]
        # sequence_service = pattern_list[1]
        # interrupting_service = pattern_list[2]

        self.index = 0
        self.class_clients={}
        self._setup_classes()
        self.state = State.INIT

    def _setup_classes(self):
        """
        Set up the pattern classes from the patterns found in pattern parameter in configuration.yaml
        """
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pattern")
        
        for pattern in self.scripted_services:
            pattern_script_path = pck_path + f"/pattern_scripting/{pattern}.json"
            with open(pattern_script_path, "r") as read_file:
                script = json.load(read_file)
            rospy.loginfo(pattern)
            self.class_clients[pattern] = SequentialPattern(pattern, script)
        # self.client_results[pattern] = deque()

        rospy.loginfo("Classes instantiate")
        rospy.loginfo(
            f"{self.name} Decision manager needs these pattern classes: {self.scripted_services}"
        )
        rospy.loginfo("Decision interface classes clients have been set up!")
        return

    def start(self, service="setup"):
        self.index = 0
        self.state = State.START
        self.do_request(self.index, service, 'Ciao')
        return

    def do_request(self, index, service, optional_data=None):
        rospy.loginfo("_____START STEP " + str(index) + " DECISION MANAGER FOR SERVICE " + service + "_______")
        self.state = State.REQUEST

        if optional_data != "":
            optional_data = str(optional_data)

        def daemon():
            rospy.loginfo("Starting")
            self.class_clients[service].reset_init()

            result_msg = self.class_clients[service].request(optional_data)

            result = {"service": service, "message": result_msg}
            rospy.loginfo("Received result from class")

            self._result_callback(result) 

            rospy.loginfo('Exiting')

        d = threading.Thread(target=daemon)
        d.setDaemon(True)
        d.start()
        return

    def _result_callback(self, result):
        """ Receive and store result with timestamp """
        rospy.loginfo("The result of the request has been received")
        rospy.loginfo(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_results[result["service"]].append(
            {"time": time(), "data": result["message"]}
        )
        if isinstance(result["message"], str) and result["message"] != "":
            result_data = ast.literal_eval(result["message"])

        rospy.loginfo(result_data)

        if result['service'] == self.scripted_services[1]:

            # TODO Check if it has an action for hass, otherwise do reverse dialogue again
            rospy.loginfo("Index " + str(self.index))
            service = "reverse_dialogue" # or hass
            self.index += 1

            # Testing: 2 sequences of same pattern but different optional_data
            if(self.index==2):
                self.do_request(self.index, service, optional_data="Sono stanca")

            # self.do_request(self.index, service, optional_data="action to do")
            self.state = State.SUCCESS

        elif result['service'] == self.scripted_services[2]:

            # TODO depending on the code received send a different message to reverse dialogue (ok / not ok)
            # TODO manage multiple consecutive hass requests

            service = "reverse_dialogue"
            self.index += 1
            # self.do_request(self.index, service, optional_data="action success or not")
            self.state = State.SUCCESS

        elif result['service'] == self.scripted_services[0]:
            
            # After setup, start the sequential pattern

            service = self.scripted_services[1]
            self.index += 1
            self.do_request(self.index, service, optional_data="Ciao")
            self.state = State.SUCCESS

        else:
            rospy.loginfo("Error")
            self.state = State.FAILURE

        rospy.loginfo("_____END STEP " + str(self.index) + " DECISION MANAGER_______")
        return


if __name__ == "__main__":
    name = rospy.get_param("/pattern_name/")
    # test = rospy.get_param("/test_" + name + "/")
    # test_input = rospy.get_param("/test_input_" + name + "/")
    test_input = "Input"
    instance_id = rospy.get_param("/instance_id_" + name + "/")

    pattern_list = []

    # ERROR this prints h a s s
    # pattern_dict = rospy.get_param("/pattern_name")
    # for p in pattern_dict:
    # pattern_list.append(p)
    # rospy.loginfo(pattern_list)
    # TODO GET THESE FROM CONFIG FILE

    pattern_list.append("setup")
    pattern_list.append("reverse_dialogue")
    pattern_list.append("hass")

    try:
        rospy.init_node(name + "_decision")
        bc = HomeAssistantDecisionManager(name, pattern_list, instance_id, test_input)
        rospy.loginfo(f"START from the first step of {name} decision.")

        bc.start(service="setup")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
