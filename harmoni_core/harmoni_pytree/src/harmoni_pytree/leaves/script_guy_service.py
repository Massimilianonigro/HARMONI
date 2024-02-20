#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import py_trees
import rospkg
import random
import rospy
from random import randrange
from harmoni_common_lib.constants import *

class ScriptGuyService(py_trees.behaviour.Behaviour):
    def __init__(self, name, params):
        self.name = name
        self.user_name=params['user_name']
        self.researcher_name=params['researcher_name']
        self.script_name = params['interaction']
        self.session = params['session']
        self.scene = params['scene']
        self.condition = params['condition']
        self.human_stopper = "\nHuman: "
        self.ai_stopper = "\nAI: "
        self.utterance_to_play = ""
        self.utterance_to_nlp = []
        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(key="gesture", access=py_trees.common.Access.WRITE)
        
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/nlp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/request", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/exercise", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/max_number_scene", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/scene_counter", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/scene_end", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/action", access=py_trees.common.Access.READ)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name+"/"+PyTreeNameSpace.trigger.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_bot.register_key("feeling", access=py_trees.common.Access.READ)
        self.blackboard_bot.register_key("sentiment", access=py_trees.common.Access.READ)
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.READ)
        
        super(ScriptGuyService, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        #this is the name of the json without the extension
        json_name = self.script_name
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pytree")
        pattern_script_path = pck_path + f"/resources/{json_name}.json"
        with open(pattern_script_path, "r") as read_file:
            self.context = json.load(read_file)
        self.blackboard_scene.scene.max_number_scene= len(self.context[self.session])
        self.blackboard_scene.scene.utterance = self.context[self.session][0]["utterance"]
        self.blackboard_scene.scene.nlp = self.context[self.session][0]["nlp"]
        self.blackboard_scene.scene.exercise = self.session[-1]
        self.logger.debug("  %s [ScriptGuyService::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ScriptGuyService::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [ScriptGuyService::update()]" % self.name)
        self.blackboard_scene.scene.nlp = self.context[self.session][self.blackboard_scene.scene.scene_counter]["nlp"]
        if self.blackboard_scene.scene.scene_counter !=0:
            rospy.loginfo(self.blackboard_bot.result["message"])
            previous_bot_response = self.blackboard_bot.result["message"]
        else:
            previous_bot_response = ""
        
        utterance = self.context[self.session][self.blackboard_scene.scene.scene_counter]["utterance"]
        ## the chatGPT leave will handle the NLP == 0 and NLP ==  1 cases
        if self.blackboard_scene.scene.nlp: #NLP ==  1
            
            if len(self.utterance_to_nlp) == 0:
                context = "*system*"
                self.utterance_to_nlp.append( context + self.ai_stopper + self.context[self.session][self.blackboard_scene.scene.scene_counter]["utterance"] + self.human_stopper + self.blackboard_stt.result + self.ai_stopper)
            else:
                context = "*assistant*"
                self.utterance_to_nlp.append(context + previous_bot_response)
                context = "*user*"
                self.utterance_to_nlp.append(context + self.human_stopper + self.blackboard_stt.result + self.ai_stopper)
            utterance = self.utterance_to_nlp
        else: # NLP == 0
            self.utterance_to_play = self.context[self.session][self.blackboard_scene.scene.scene_counter]["utterance"]
            utterance = self.utterance_to_play
          
        gesture = self.context[self.session][self.blackboard_scene.scene.scene_counter]["gesture"]
        username = "USERNAME" 
        researcher = "RESEARCHERNAME"
        if username in utterance:
            utterance = utterance.replace(username, self.user_name)
        if researcher in utterance:
            utterance = utterance.replace(researcher, self.researcher_name)

        if self.blackboard_scene.scene.scene_end == "end":
            return py_trees.common.Status.FAILURE
        else:
            self.blackboard_scene.scene.utterance =utterance
            self.blackboard_scene.gesture = gesture
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        if new_status == py_trees.common.Status.INVALID:
            self.scene_counter = 0
        """
        self.logger.debug("  %s [ScriptGuyService::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
