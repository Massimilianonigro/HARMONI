#!/usr/bin/env python3


PKG = "test_harmoni_polly"
# Common Imports
import unittest, rospy, roslib, sys
import traceback

# Specific Imports
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import DetectorNameSpace, SensorNameSpace, ActuatorNameSpace, State, DialogueNameSpace
import ast
import time
import wave

#py_tree
import py_trees
from harmoni_pytree.subtrees.mic_to_face__audiofile import *
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
from audio_publisher import TestDeepSpeech_Common


class TestMiniBotPytree(unittest.TestCase):

    def setUp(self):

        rospy.init_node("test_mic_to_face", log_level=rospy.INFO)
        self.instance_id = rospy.get_param("instance_id")
        
        self.audio_pub_signal = rospy.Publisher(
            "audio_pub_signal",
            String,
            queue_size=10,
        )


        # NOTE currently no feedback, status, or result is received.
        py_trees.logging.level = py_trees.logging.Level.DEBUG

        # stt blackboard
        self.blackboard_stt = py_trees.blackboard.Client(name="blackboard_stt", namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.WRITE)

        # blackboard for test-to-speech data, which is updated after result is fetched
        self.blackboard_tts = py_trees.blackboard.Client(name="blackboard_tts", namespace=ActuatorNameSpace.tts.name)
        self.blackboard_tts.register_key("result", access=py_trees.common.Access.WRITE)

        # blackboard to store data to send to the action server
        self.blackboard_bot = py_trees.blackboard.Client(name="blackboard_bot", namespace=DialogueNameSpace.bot.name+"/"+ PyTreeNameSpace.analyzer.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)

        # blackboard for storing facial expressions
        self.blackboard_scene = py_trees.blackboard.Client(name="blackboard_scene", namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)

        # blackboards for lex and mic

        self.root = create_root()
        self.tree = py_trees.trees.BehaviourTree(self.root)
        self.tree.setup()
        time.sleep(5)
        self.success = True
        self.audio_pub_signal.publish(String("True"))
        rospy.loginfo("Setup completed....starting test")


   
    def test_leaf_pytree_tts(self):
        # rospy.loginfo(f"The speaker data is at {self.wav_loc}")
        try:
            for unused_i in range(0, 20):
                curr_status = self.root.tick_once()
                time.sleep(1)
                print("Tick number: ", unused_i)

        except Exception:
            self.success = False
            print(traceback.format_exc())

        assert self.success == True

        return
    

def main():
    import rostest
    rospy.loginfo("test_polly started")
    rospy.loginfo("TestPolly: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_polly_pytree", TestMiniBotPytree, sys.argv)


if __name__ == "__main__":
    main()
