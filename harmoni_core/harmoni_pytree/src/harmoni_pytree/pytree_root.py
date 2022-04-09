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
from harmoni_pytree.leaves.aws_lex_trigger_service import AWSLexTriggerServicePytree
from harmoni_pytree.leaves.speaker_service import SpeakerServicePytree
from harmoni_pytree.leaves.lip_sync_service import LipSyncServicePytree
from harmoni_common_lib.constants import ActuatorNameSpace, DialogueNameSpace, State
import argparse
import py_trees
import sys
import time

import py_trees.console as console

##############################################################################
# Classes
##############################################################################

def description(root):
    content = "Demonstrates sequences in action.\n\n"
    content += "A sequence is populated with 2-tick jobs that are allowed to run through to\n"
    content += "completion.\n"

    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Sequences".center(79) + "\n" + console.reset
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
    parser = argparse.ArgumentParser(description=self.description,
                                    epilog=self.epilog,
                                    formatter_class=argparse.RawDescriptionHelpFormatter,
                                    )
    parser.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
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

def create_root():
    root = py_trees.composites.Sequence("Sequence")
    tts = AWSTtsServicePytree("AwsTtsPyTreeTest")
    chatbot = AWSLexTriggerServicePytree("AwsLexPyTreeTest")
    speaker = SpeakerServicePytree("SpeakerPyTreeTest")
    face = LipSyncServicePytree("FacePyTreeTest")
    parall_speaker_face = py_trees.composites.Parallel("Parallel")
    root.add_child(chatbot)
    root.add_child(tts)
    root.add_child(parall_speaker_face)
    parall_speaker_face.add_child(speaker)
    parall_speaker_face.add_child(face)

    return root

 ##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    root =create_root()
    print(description(root))

    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=PyTreeNameSpace.scene.name)
    blackboardProva.register_key("utterance", access=py_trees.common.Access.WRITE)
    blackboardProva.utterance = "Hey"
    #blackboardProvaOutput = py_trees.blackboard.Client(name="blackboardProvaOutput", namespace=DialogueNameSpace.bot.name+"/"+PyTreeNameSpace.trigger.name)
    #blackboardProvaOutput.register_key("result", access=py_trees.common.Access.READ)                        
    print(blackboardProva)
    #print(blackboardProva2)
        
    ####################
    # Tree Stewardship
    ####################

    rospy.init_node("test_default", log_level=rospy.INFO)

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

    for unused_i in range(1, 7):
        try:
            behaviour_tree.tick()
            time.sleep(3)
        except KeyboardInterrupt:
            break
    print("\n")


if __name__ == "__main__":
    main()   


