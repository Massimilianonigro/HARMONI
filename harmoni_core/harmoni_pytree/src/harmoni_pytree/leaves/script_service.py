#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import py_trees
import rospkg
import random
import rospy

from harmoni_common_lib.constants import *

class ScriptService(py_trees.behaviour.Behaviour):
    def __init__(self, name, params):
        self.name = name
        self.script_name = params['interaction']
        self.session = params['session']
        self.scene = params['scene']
        #Conversation will be saved as a list of objects like {"user":"Hi","addresse":"robot"}...
        self.conversation = []
        self.bot_name = "Robot"
        
        self.human_stopper = "\nHuman: "
        self.ai_stopper = "\nAI: "
        self.utterance_to_play = ""
        self.utterance_to_nlp = []

        #This will be used when crafting the prompt for the LLM before any robot sentence
        self.bot_sentence_preview = "Precedentemente hai detto al ADDRESSEE:"
        #What the user said
        self.user_sentence_preview = "Il ADDRESSEE ti ha risposto:" 
        self.save = True

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(key="gesture", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="gesture_list", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="drive", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="drive_list", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="head", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="head_list", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="sound", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="nlp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="rl", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="exercise", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="addressee", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="addressee_name", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="next_addressee", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="max_number_scene", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="continue_to_next_scene", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="previous_continue_to_next_scene",access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="scene_counter", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key="scene_end", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key="action", access=py_trees.common.Access.READ)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_face = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.face.name)
        self.blackboard_face.register_key("face_exp", access=py_trees.common.Access.WRITE)
        super(ScriptService, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        #this is the name of the json without the extension
        json_name = self.script_name
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pytree")
        pattern_script_path = pck_path + f"/resources/scripts/{json_name}.json"
        with open(pattern_script_path, "r") as read_file:
            self.context = json.load(read_file)
        self.blackboard_scene.gesture_list = []
        self.blackboard_scene.head_list = []
        self.blackboard_scene.drive_list = []
        self.blackboard_scene.max_number_scene= len(self.context[self.session])
        self.blackboard_scene.utterance = self.context[self.session][0]["utterance"]
        self.blackboard_scene.nlp = self.context[self.session][0]["nlp"] 
        self.blackboard_scene.rl = self.context[self.session][0]["rl"]
        self.blackboard_scene.sound = self.context[self.session][0]["sound"]
        self.blackboard_scene.gesture = self.context[self.session][0]["gesture"]
        self.blackboard_scene.drive = self.context[self.session][0]["drive"]
        self.blackboard_scene.head = self.context[self.session][0]["head"]
        self.blackboard_scene.previous_continue_to_next_scene = False
        self.blackboard_scene.exercise = self.session[-1]
        self.blackboard_scene.continue_to_next_scene = self.context[self.session][0]["continue_to_next_scene"]
        self.dictonary_session = self.context["dictionary_" + self.session] if ("dictionary_" + self.session) in self.context.keys() else None
        self.logger.debug("  %s [ScriptService::setup()]" % self.name)
        
    def initialise(self):
        self.logger.debug("  %s [ScriptService::initialise()]" % self.name)

    #Updates the blackboard values to the current scene counter
    def _update_blackboard(self):
        self.blackboard_face.face_exp = self.context[self.session][self.blackboard_scene.scene_counter]["face"]
        self.blackboard_scene.sound = self.context[self.session][self.blackboard_scene.scene_counter]["sound"]
        self.blackboard_scene.gesture = self.context[self.session][self.blackboard_scene.scene_counter]["gesture"]
        self.blackboard_scene.drive = self.context[self.session][self.blackboard_scene.scene_counter]["drive"]
        self.blackboard_scene.head = self.context[self.session][self.blackboard_scene.scene_counter]["head"]
        self.blackboard_scene.nlp = self.context[self.session][self.blackboard_scene.scene_counter]["nlp"]
        self.blackboard_scene.addressee = self.context[self.session][self.blackboard_scene.scene_counter]["addressee"]
        self.blackboard_scene.continue_to_next_scene = self.context[self.session][self.blackboard_scene.scene_counter]["continue_to_next_scene"]
        self.blackboard_scene.addressee_name = self.context[self.session][self.blackboard_scene.scene_counter]["addressee_name"]
        if (self.blackboard_scene.scene_counter - 1) >= 0:
            self.blackboard_scene.previous_continue_to_next_scene = self.context[self.session][self.blackboard_scene.scene_counter - 1]["continue_to_next_scene"]
        if self.blackboard_scene.scene_counter < (self.blackboard_scene.max_number_scene - 1):
            rospy.loginfo("SCENE COUNTER IS " + str(self.blackboard_scene.scene_counter) + " MAX NUMBER SCENE IS " + str(self.blackboard_scene.max_number_scene))
            self.blackboard_scene.next_addressee = self.context[self.session][self.blackboard_scene.scene_counter + 1]["addressee"]
        self.blackboard_scene.gesture_list.append(self.blackboard_scene.gesture)
        self.blackboard_scene.drive_list.append(self.blackboard_scene.drive)
        self.blackboard_scene.head_list.append(self.blackboard_scene.head)

    #Updates record of previous conversation
    def _update_script(self):
        #Before updating all the blackboard values, I have the last values of NLP,
        #  continue_to_next_scene, addressee, addressee_name and I can get the last utterance
        #  with the scene_counter -1
        #Skipping on the first scene (it will be updated when scene_counter is 1)
        if self.blackboard_scene.scene_counter == 0:
            return
        #If NLP was true, last sentence was generated by LLM
        if self.blackboard_scene.nlp:
            last_bot_utterance = {
                self.bot_name: self.blackboard_bot.result["message"],
                "addressee": self.blackboard_scene.addressee_name
            }
        else:
            #Otherwise it was written in the .json script
            last_bot_utterance = {
                self.bot_name: self.context[self.session][self.blackboard_scene.scene_counter-1]["utterance"],
                "addressee": self.blackboard_scene.addressee_name
            }
        self.conversation.append(last_bot_utterance)
        #If continue to next scene is false, I should also expect that the user answered
        if not self.blackboard_scene.continue_to_next_scene:
            #And we assuming that the user always talks to the robot
            last_user_utterance = {
                self.blackboard_scene.addressee_name: self.blackboard_stt.result,
                "addressee": self.bot_name
            }
            self.conversation.append(last_user_utterance)


    #Adjusts the prompt sent to the LLM based on previous conversation and prompt written in the .json
    #Adding to the .json prompt the last sentence from the robot and if there is, the last answer from the user
    def _get_llm_input(self):
        prompt = self.context[self.session][self.blackboard_scene.scene_counter]["utterance"]
        rospy.loginfo("SELF CONVERSATION IS " + str(self.conversation))
        if len(self.conversation) != 0:
            last_sentence = self.conversation[-1]
            rospy.loginfo("LAST SENTENCE IS " + str(last_sentence))
            #In the last sentence a user answered the robot
            if self.bot_name not in last_sentence.keys():
                #Assuming the robot was the one that talked before
                robot_sentence = self.conversation[-2]
                rospy.loginfo("INSIDE THE IF LAST SENTENCE IS " + str(last_sentence[robot_sentence["addressee"]]))
                prompt = prompt + self.bot_sentence_preview.replace("ADDRESSEE", robot_sentence["addressee"]) + robot_sentence[self.bot_name] + "."
                prompt = prompt + self.user_sentence_preview.replace("ADDRESSEE", robot_sentence["addressee"]) + last_sentence[robot_sentence["addressee"]] + "."
                #Put this here to make sure chatgpt completes the robot part and not the child part
                prompt = prompt + "Robot:____"
            else:
                prompt = prompt + self.bot_sentence_preview.replace("ADDRESSEE", last_sentence["addressee"]) + last_sentence[self.bot_name] + "."
            rospy.loginfo("PROMPT RETURNED IS " + str(prompt))
        return prompt


    def update(self):
        self.logger.debug("  %s [ScriptService::update()]" % self.name)
        #Recording the last steps of the conversation in self.conversation  
        self._update_script()
        #Updating the blackboard values to the current scene counter
        self._update_blackboard()
        rospy.loginfo("AOOOOOOOOOOOOOOOOOO INTO SCRIPT SERVICE 1")
        if self.blackboard_scene.scene_counter !=0:
            rospy.loginfo(self.blackboard_bot.result["message"])
            self.previous_bot_response = self.blackboard_bot.result["message"]
        else:
            self.previous_bot_response = ""
        rospy.loginfo("AOOOOOOOOOOOOOOOOOO INTO SCRIPT SERVICE 2")
        if self.blackboard_scene.nlp:
            prompt = self._get_llm_input()
            rospy.loginfo("AOOOOOOOOOOOOOOOOOO INTO SCRIPT SERVICE 3")
            context = "*system*"
            utterance = []
            utterance.append(context + prompt)
            rospy.loginfo("AOOOOOOOOOOOOOOOOOO INTO SCRIPT SERVICE 4")
        else:
            utterance = self.context[self.session][self.blackboard_scene.scene_counter]["utterance"]
        rospy.loginfo("AOOOOOOOOOOOOOOOOOO INTO SCRIPT SERVICE 6")

        gesture = self.context[self.session][self.blackboard_scene.scene_counter]["gesture"]
        if self.blackboard_scene.scene_end == "end":
            if self.save:
                #Saving the conversation in a .json file
                self.save_conversation()
            return py_trees.common.Status.FAILURE
        else:
            rospy.loginfo("AOOOOOOOOOOOOOOOOOO INTO SCRIPT SERVICE 7")

            self.blackboard_scene.utterance = utterance
            self.blackboard_scene.gesture = gesture
            return py_trees.common.Status.SUCCESS

    def save_conversation(self):
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pytree")
        pattern_script_path = pck_path + f"/temp_data/last_conversation.json"
        json.dump(self.conversation, open(pattern_script_path, "w"))

    def terminate(self, new_status):
        """
        if new_status == py_trees.common.Status.INVALID:
            self.scene_counter = 0
        """
        self.logger.debug("  %s [ScriptService::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
