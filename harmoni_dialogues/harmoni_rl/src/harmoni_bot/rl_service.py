#!/usr/bin/env python3

# Common Imports
import rospy
from dotenv import load_dotenv
from harmoni_common_lib.constants import *
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf
from std_msgs.msg import String, Bool
# Specific Imports
import os
import ast

class RLService(HarmoniServiceManager):
    """This is a class representation of a harmoni_dialogue service
    (HarmoniServiceManager). It is essentially an extended combination of the
    :class:`harmoni_common_lib.service_server.HarmoniServiceServer` and :class:`harmoni_common_lib.service_manager.HarmoniServiceManager` classes

    :param name: Name of the current service
    :type name: str
    :param param: input parameters of the configuration.yaml file
    :type param: from yaml
    """

    def __init__(self, name, param):
        """Constructor method: Initialization of variables and lex parameters + setting up"""
        super().__init__(name)
        """ Initialization of variables and lex parameters """
        self.subscriber_id = param['subscriber_id']
        self.state = State.INIT
        self._fer_sub = rospy.Subscriber(DetectorNameSpace.fer.value + self.subscriber_id, String, self._fer_detector_cb, queue_size=1)
        self._detcustom_sub = rospy.Subscriber(DetectorNameSpace.detcustom.value + self.subscriber_id, Bool, self._detcustom_detector_cb, queue_size=1)
        self._vad_sub = rospy.Subscriber(DetectorNameSpace.vad.value + self.subscriber_id, Bool, self._vad_detector_cb, queue_size=1)
        self._stt_sub = rospy.Subscriber(DetectorNameSpace.stt.value + self.subscriber_id, String, self._stt_detector_cb, queue_size=1)
        return

    def _fer_detector_cb(self, data):
        data = ast.literal_eval(data.data)
        rospy.loginfo("==================== FER DETECTION RECEIVED")
        rospy.loginfo(data)
        if self.state == State.REQUEST:
            self.fer.append(data)
        return

    def _detcustom_detector_cb(self, data):
        data = data.data
        rospy.loginfo("==================== IR DETECTION RECEIVED")
        rospy.loginfo(data)
        if self.state == State.REQUEST:
            self.detcustom.append(data)
        return
    
    def _vad_detector_cb(self, data):
        data = data.data
        rospy.loginfo("==================== VAD DETECTION RECEIVED")
        rospy.loginfo(data)
        if self.state ==  State.REQUEST:
            self.vad.append(data)
        return

    def _stt_detector_cb(self, data):
        data = ast.literal_eval(data.data)
        rospy.loginfo("==================== STT DETECTION RECEIVED")
        rospy.loginfo(data)
        if self.state ==  State.REQUEST:
            self.stt.append(data)
        return

    def request(self, exercise):
        """[summary]

        Args:
            exercise (str): Exercise current state

        Returns:
            object: It containes information about the response received (bool) and response message (str)
                response: bool
                message: str
        """
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        result = {"response": False, "message": None}
        try:
            ####
            self.result_msg = ai_response
            self.response_received = True
            self.state = State.SUCCESS
        except rospy.ServiceException:
            self.state = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_received = True
            self.result_msg = ""
        return {"response": self.state, "message": self.result_msg}

def main():
    """[summary]
    Main function for starting HarmoniRLService service
    """
    service_name = DialogueNameSpace.rl.name
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name, log_level=rospy.DEBUG)
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")
        s = RLService(service_id, params)
        service_server = HarmoniServiceServer(service_id, s)
        #s.request("The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n\nHuman: Hello, who are you?\nAI: I am an AI created by OpenAI. How can I help you today?\nHuman: I want to do a positive psychology exercise")
        print(service_name)
        print("**********************************************************************************************")
        print(service_id)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()