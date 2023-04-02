#!/usr/bin/env python3


PKG = "test_harmoni_vad"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import (
    DetectorNameSpace,
    SensorNameSpace,
    ActionType,
    State,
)
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from harmoni_common_lib.constants import SensorNameSpace
from std_msgs.msg import String, Bool


# from std_msgs.msg import String
import time
import os, io


class TestVoiceDetection_Common(unittest.TestCase):
       
    def setUp(self):
        self.feedback = State.INIT
        self.result = False
        self.detections = []
        rospy.init_node("test_vad", log_level=rospy.INFO)
        self.rate = rospy.Rate(1)
        # provide mock camera
        self.output_sub = rospy.Subscriber(
            DetectorNameSpace.vad.value + "default",
            Bool,
            self._detecting_callback,
        )
        rospy.loginfo(f"Testside-Image source: {SensorNameSpace.camera.value}default")
        rospy.loginfo(
            f"Testside-expected detection: {DetectorNameSpace.vad.value}default"
        )

        # startup vad node
        self.server = "/harmoni/detecting/vad/default"
        self.client = HarmoniActionClient(self.server)
        rospy.loginfo("***********SETTING UP CLIENT")
        self.client.setup_client(
            self.server, self._result_callback, self._feedback_callback, wait=True
        )
        rospy.loginfo("DONE SETTING UP****************")
        rospy.loginfo("TestVoiceDetection: Turning ON vad server")
        self.client.send_goal(
            action_goal=ActionType.ON, optional_data="Setup", wait=False
        )
        rospy.loginfo("TestVoiceDetection: Started up. waiting for face detect startup")

        rospy.loginfo("TestVoiceDetection: publishing vad")


    def _feedback_callback(self, data):
        rospy.loginfo(f"TestVoiceDetection: Feedback: {data}")
        self.feedback = data["state"]

    def _status_callback(self, data):
        rospy.loginfo(f"TestVoiceDetection: Status: {data}")
        # self.result = True

    def _result_callback(self, data):
        rospy.loginfo(f"TestVoiceDetection: Result: {data}")
        # self.result = True

    def text_received_callback(self, data):
        rospy.loginfo(f"TestVoiceDetection: Text back: {data}")
        # self.result = True

    def _detecting_callback(self, data):
        rospy.logdebug(f"TestVoiceDetection: Detecting: {data}")
        print(data)
        self.detections = data.data
        self.result = True



class TestVoiceDetection_Valid(TestVoiceDetection_Common):
    def test_IO(self):
        rospy.loginfo(
            "TestVoiceDetection_Valid[TEST]: basic IO test to ensure data "
            + "(example image) is received and responded to. Waiting for detection..."
        )
        self.output_sub = rospy.Subscriber(
            DetectorNameSpace.vad.value + "default",
            Bool,
            self._detecting_callback,
        )
        rospy.sleep(4)
        assert self.result == True


def main():
    import rostest

    rospy.loginfo("Testvad: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_vad", TestVoiceDetection_Valid, sys.argv)


if __name__ == "__main__":
    main()
