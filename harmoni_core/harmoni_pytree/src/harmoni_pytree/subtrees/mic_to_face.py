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
import py_trees.console as console

from harmoni_common_lib.constants import *

from harmoni_pytree.leaves.deep_stt import DeepSpeechToTextServicePytree
from harmoni_pytree.leaves.aws_lex_analyzer_service import AWSLexAnalyzerServicePytree
from harmoni_pytree.leaves.aws_tts_service import AWSTtsServicePytree
from harmoni_pytree.leaves.facial_exp_service import FacialExpServicePytree
from harmoni_pytree.leaves.microphone_service import MicrophoneServicePytree
from harmoni_pytree.leaves.speaker_service import SpeakerServicePytree

##############################################################################
# Classes
##############################################################################


def description(root):
    """Function that returns a string having description 
    of the subtree defined in this script

    Args:
        root (py_trees.behaviour.Behaviour): 

    Returns:
        str: string containing the description of the subtree
    """
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
    """ Method for generating the text to display after the argument help 

    Returns:
        str: text that will be displayed after argument help
    """
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    """Method for constructing argument parser

    Returns:
        argparse.ArgumentParser: parser with required arguments added
    """
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
    """Method which would be called before a single tick

    Args:
        behaviour_tree (py_trees.behaviour.Behaviour): The behaviour tree where this method is added
    """
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def post_tick_handler(snapshot_visitor, behaviour_tree):
    """Method which would be called after a single tick

    Args:
        snapshot_visitor (py_trees.visitors.SnapshotVisitor): The snapshot visitor for getting the visited leaves
        behaviour_tree (py_trees.behaviour.Behaviour): The behaviour tree where this method is added 
    """
    print(
        "\n" + py_trees.display.unicode_tree(
            root=behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited
        )
    )
    print(py_trees.display.unicode_blackboard())


def create_root(name= "MicAndSTT"):
    """Method to create the composites of the leaves

    Args:
        name (str, optional): Name of the subtree. Defaults to "Face_Polly".

    Returns:
        py_trees.behaviour.Behaviour: Behaviour subtree
    """
    # microphone = MicrophoneServicePytree("MicrophoneMainActivity")
    deep_stt = DeepSpeechToTextServicePytree("SpeechToTextMainActivity")
    speaker = SpeakerServicePytree("SpeakerActivity")
    tts = AWSTtsServicePytree("TTSActivity")
    chatbot_analyzer = AWSLexAnalyzerServicePytree("AwsLexAnalyzerPyTreeActivity")
    face = FacialExpServicePytree("FaceActivity")
    face_speaker = py_trees.composites.Parallel(name="face_speaker")
    face_speaker.add_children([speaker, face])


    root = py_trees.composites.Sequence(name="mini_bot")
    root.add_children([deep_stt, chatbot_analyzer, tts, face_speaker])

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

    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=DetectorNameSpace.stt.name)
    blackboardProva.register_key("result", access=py_trees.common.Access.READ)
    print(blackboardProva)
        
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

    for unused_i in range(1, 30):
        try:
            behaviour_tree.tick()
            time.sleep(3)
        except KeyboardInterrupt:
            break
    print("\n")


if __name__ == "__main__":
    main()
