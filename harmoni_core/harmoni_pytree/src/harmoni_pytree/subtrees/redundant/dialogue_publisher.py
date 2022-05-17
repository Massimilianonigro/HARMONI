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

#py_tree
import py_trees
import time


class BotDialogues(py_trees.behaviour.Behaviour):
	def __init__(self, timeout=10):

		self.timeout = timeout
		self.blackboards = []
		# self.blackboard_bot = self.attach_blackboard_client(name="dialogue_pub", namespace=DialogueNameSpace.bot.name +"/"+PyTreeNameSpace.trigger.name)
		# self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)
		self.blackboard_bot = py_trees.blackboard.Client(name="Writer")
		self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)
		while True:
			diag = input("Enter Dialogue: ")
			self.blackboard_bot.result = diag
			print(self.blackboard_bot)


if __name__ == '__main__':
	BotDialogues()