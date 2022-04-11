#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random
import rospy


class WaitResults(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.name = name
        super(WaitResults, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        
    def setup(self):
        self.logger.debug("  %s [WaitResults::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [WaitResults::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [WaitResults::update()]" % self.name)
        rospy.sleep(120)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [WaitResults::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
