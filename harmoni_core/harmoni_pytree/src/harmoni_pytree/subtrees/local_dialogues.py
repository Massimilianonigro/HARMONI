#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import *
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.action_client import HarmoniActionClient
import harmoni_common_lib.helper_functions as hf
from harmoni_tts.aws_tts_service import AWSTtsService
# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType, State, DialogueNameSpace
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
from collections import deque 
import soundfile as sf
import numpy as np
import boto3
import re
import ast
import sys

from std_msgs.msg import Int32
from std_msgs.msg import String
import numpy as np

class BotDialogues(py_trees.behaviour.Behaviour):
	def __init__(self, timeout=10):
		self.timeout = timeout
		rospy.init_node('diaglogue_publisher', anonymous=True)
		self.idle_sub = rospy.Subscriber("deepspeech/idle_state", Int32, self.callback)
		# TODO : fill up name of rostopic for publisher
		self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name +"/"+PyTreeNameSpace.trigger.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)

		
		with open('simple_dialogues.json', 'r') as f:
  			data = json.load(f)
  		self.next_diag_idx = 1

  		rospy.spin()


	def callback(self, message):
		if(message.data % self.timeout):
			self.blackboard_bot.result = data[f"statement{next_diag_idx}"]
			if (self.next_diag_idx < len(data)):
				self.next_diag_idx += 1
				rospy.loginfo(self.blackboard_bot.result)
			else:
				self.next_diag_idx = 0
		rospy.loginfo(f"Idle counter : {messsage_data}")

		


if __name__ == '__main__':
	main()