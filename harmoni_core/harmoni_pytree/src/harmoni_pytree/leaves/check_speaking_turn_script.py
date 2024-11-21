#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random
import rospy
import rospkg
import json

class CheckSpeakingTurnScript(py_trees.behaviour.Behaviour):
    def __init__(self, name, params):
        self.name = name
        self.scene = params['scene']
        self.script_name = params['interaction']
        self.session = params['session']
        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(key="continue_to_next_scene", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key="max_number_scene", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key="scene_end", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="scene_counter", access=py_trees.common.Access.WRITE)
        super(CheckSpeakingTurnScript, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        
    def setup(self):
        json_name = self.script_name
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pytree")
        pattern_script_path = pck_path + f"/resources/scripts/{json_name}.json"
        with open(pattern_script_path, "r") as read_file:
            self.context = json.load(read_file)
        self.blackboard_scene.scene_end = ''
        self.blackboard_scene.scene_counter = self.scene

        self.logger.debug("  %s [CheckSpeakingTurnScript::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckSpeakingTurnScript::initialise()]" % self.name)

    def update(self):

        ### TODO: IMPLEMENT A CHECK OF SENTIMENT, AND UTTERANCE ITSELF.
        self.logger.debug("  %s [CheckSpeakingTurnScript::update()]" % self.name)
        rospy.loginfo("AOOOOOOOOOOO into check speaking turn script")
        if self.blackboard_scene.scene_end == 'call_researcher':
            rospy.sleep(5)
            self.blackboard_scene.scene_end = 'end'
        else:
            rospy.loginfo("AOOOOOOOOOOO continue to next scene is %s" % self.blackboard_scene.continue_to_next_scene)

            if self.blackboard_scene.continue_to_next_scene:
                rospy.loginfo("AOOOOOOOOOOO if of into check speaking turn script")

                if self.blackboard_scene.scene_counter < self.blackboard_scene.max_number_scene -1:                    
                    self.blackboard_scene.scene_counter+=1
                else:
                    self.blackboard_scene.scene_end = 'end'
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckSTTResultErrors::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
