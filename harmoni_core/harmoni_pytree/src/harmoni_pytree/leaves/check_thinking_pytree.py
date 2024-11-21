#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random
import rospy


class CheckThinkingPyTree(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.name = name
        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(key="addressee", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key="next_addressee", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key="continue_to_next_scene", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key="scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)
        self.stored_utterance = ""
        super(CheckThinkingPyTree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        
    def setup(self):
        self.logger.debug("  %s [CheckThinking::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckThinking::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [ChecktHINKING::update()]" % self.name)
        #If the addressee is the same and I have to continue talking with them, I start storing and uniting the utterances (I want to tell all of them at once)
        if self.blackboard_scene.continue_to_next_scene and (self.blackboard_scene.addressee == self.blackboard_scene.next_addressee):
            self.stored_utterance = self.stored_utterance + " " + self.blackboard_bot.result["message"]
            self.blackboard_scene.scene_counter = self.blackboard_scene.scene_counter + 1
            return py_trees.common.Status.SUCCESS
        else:
            #If the addressee is not the same I either change the result, if an utterance was cumulated or just return SUCCESS
            #If the utterance was stored, we change the bot.result with the chained utterance that was stored and reset the stored utterance
            if self.stored_utterance != "":
                self.stored_utterance = self.stored_utterance + ". " + self.blackboard_bot.result["message"]
                self.blackboard_bot.result = {"message":self.stored_utterance}
                self.stored_utterance = ""
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckThinking::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
