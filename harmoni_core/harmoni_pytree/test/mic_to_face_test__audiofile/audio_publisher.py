#!/usr/bin/env python3

# Common Imports
import io
import rospy
import sys
import unittest

# Specific Imports
import time
import wave
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import ActionType, DetectorNameSpace, SensorNameSpace, State

from audio_common_msgs.msg import AudioData
from std_msgs.msg import String

PKG = "test_harmoni_stt"


class TestDeepSpeech_Common(unittest.TestCase):

    def setUp(self):
        self.feedback = State.INIT
        self.result = False
        self.test_file = rospy.get_param("test_deepspeech_input")
        rospy.init_node("test_deepspeech", log_level=rospy.INFO)
        self.rate = rospy.Rate(20)
        self.output_sub = rospy.Subscriber(
            "/harmoni/detecting/stt/default", String, self._detecting_callback
        )
        # provide mock microphone
        self.audio_pub = rospy.Publisher(
            SensorNameSpace.microphone.value
            + rospy.get_param("stt/default_param/subscriber_id"),
            AudioData,
            queue_size=10,
        )
        rospy.Subscriber(
            DetectorNameSpace.stt.value + "stt_default",
            String,
            self.text_received_callback,
        )

        self.audio_pub_signal = False
        rospy.Subscriber("audio_pub_signal", String, self.audio_signal_callback)

        chunk_size = 1024
        wf = wave.open(self.test_file)
        # read data (based on the chunk size)
        index = 0
        audio_length = wf.getnframes()
        max_repeat_count = 10
        repeat_count = 0
        while True:# index+chunk_size < audio_length:
            if not self.audio_pub_signal:
                continue
            if repeat_count >= max_repeat_count:
                break
            data = wf.readframes(chunk_size)
            self.audio_pub.publish(data)
            index = index+chunk_size
            time.sleep(0.2)
            if index+chunk_size > audio_length:
                repeat_count += 1
                index = 0
                wf = wave.open(self.test_file)
                audio_length = wf.getnframes()

        rospy.loginfo(
            f"TestDeepSpeech: audio subscribed to by #{self.output_sub.get_num_connections()} connections."
        )

    def _feedback_callback(self, data):
        rospy.loginfo(f"TestDeepSpeech: Feedback: {data}")
        self.feedback = data["state"]

    def _status_callback(self, data):
        rospy.loginfo(f"TestDeepSpeech: Status: {data}")
        self.result = True

    def _result_callback(self, data):
        rospy.loginfo(f"TestDeepSpeech: Result: {data}")
        self.result = True

    def text_received_callback(self, data):
        rospy.loginfo(f"TestDeepSpeech: Text back: {data}")
        self.result = True

    def _detecting_callback(self, data):
        rospy.loginfo(f"TestDeepSpeech: Detecting: {data}")
        self.result = True
    
    def audio_signal_callback(self, data):
        rospy.loginfo(f"Received audio publishing signal")
        self.audio_pub_signal = True

def main():
    # TODO combine validity tests into test suite so that setup doesn't have to run over and over.
    import rostest

    rospy.loginfo("test_deepspeech started")
    rospy.loginfo("TestDeepSpeech: sys.argv: %s" % str(sys.argv))
    TestDeepSpeech_Common().setUp()


if __name__ == "__main__":
    main()
