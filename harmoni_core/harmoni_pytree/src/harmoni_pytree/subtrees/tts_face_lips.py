#!/usr/bin/env python3
##############################################################################
# Imports
##############################################################################

import argparse
import functools
from py_trees.behaviours import dummy
from py_trees.idioms import either_or
import py_trees
import time
import rospy
from random import randint
import subprocess
import operator
import py_trees.console as console

from harmoni_common_lib.constants import *

from harmoni_pytree.leaves.aws_tts_service import AWSTtsServicePytree
from harmoni_pytree.leaves.facial_exp_service import FacialExpServicePytree
from harmoni_pytree.leaves.lip_sync_service import LipSyncServicePytree

##############################################################################
# Classes
##############################################################################


def description(root):
    content = "\n\n"
    content += "\n"
    content += "EVENTS\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Test".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog():
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(create_root()),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-b', '--with-blackboard-variables', default=False, action='store_true', help='add nodes for the blackboard variables')
    group.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
    group.add_argument('-i', '--interactive', action='store_true', help='pause and wait for keypress at each tick')
    return parser


def pre_tick_handler(behaviour_tree):
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def post_tick_handler(snapshot_visitor, behaviour_tree):
    print(
        "\n" + py_trees.display.unicode_tree(
            root=behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited
        )
    )
    print(py_trees.display.unicode_blackboard())


def create_root(name= "TtsAndFaceLips"):

    face = FacialExpServicePytree("FaceMainActivity")
    lips = LipSyncServicePytree("LipSyncMainActivity")
    tts = AWSTtsServicePytree("TtsMainActivity")

    root = py_trees.composites.Sequence(name="TtsAndFaceLips",memory=False)
    parallel = py_trees.composites.Parallel(name = "FaceAndLips")
    parallel.add_children([face, lips])
    root.add_children([tts, parallel])

    return root

##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    root = create_root()
    print(description(root))

    blackboard_tts = py_trees.blackboard.Client(name="blackboard_tts", namespace=ActuatorNameSpace.tts.name)
    blackboard_tts.register_key("result", access=py_trees.common.Access.READ) 
    blackboard_scene = py_trees.blackboard.Client(name="blackboard_scene", namespace=PyTreeNameSpace.scene.name)
    blackboard_scene.register_key(key=PyTreeNameSpace.scene.name+"/nlp", access=py_trees.common.Access.READ)
    # blackboard_scene.scene.nlp = 1
    blackboard_bot = py_trees.blackboard.Client(name="blackboard_bot", namespace=DialogueNameSpace.bot.name +"/"+PyTreeNameSpace.trigger.name)
    blackboard_bot.register_key("result", access=py_trees.common.Access.READ) 

    print(blackboard_tts)
    print(blackboard_scene)
    print(blackboard_bot)
        
    ####################
    # Tree Stewardship
    ####################

    rospy.init_node("tts_and_face_lips", log_level=rospy.INFO)

    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
    behaviour_tree.visitors.append(snapshot_visitor)
    behaviour_tree.setup(timeout=15)

    ####################
    # Tick Tock
    ####################

    for unused_i in range(1, 30):
        try:
            behaviour_tree.tick()
            time.sleep(3)
        except KeyboardInterrupt:
            break
    print("\n")


if __name__ == "__main__":
    main()