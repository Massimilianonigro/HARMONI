#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random
import rospy


class CheckUtterance(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.name = name
        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/utterance_end", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/utterance_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/utterance_length", access=py_trees.common.Access.READ)
        super(CheckUtterance, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        
    def setup(self):  
        self.blackboard_scene.scene.utterance_counter = 0
        self.blackboard_scene.scene.utterance_end = ''
        self.logger.debug("  %s [CheckUtterance::setup()]" % self.name)




    def initialise(self):
        self.logger.debug("  %s [CheckUtterance::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [CheckUtterance::update()]" % self.name)
        print(self.blackboard_scene.scene.utterance_length)
        print(self.blackboard_scene.scene.utterance_counter)
        if self.blackboard_scene.scene.utterance_counter < self.blackboard_scene.scene.utterance_length-1:
            self.blackboard_scene.scene.utterance_counter += 1
            return py_trees.common.Status.FAILURE
        else:
            self.utterance_end = "end"
            self.blackboard_scene.scene.utterance_counter = 0
            return py_trees.common.Status.SUCCESS
            
       

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckUtterance::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
