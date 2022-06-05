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

from harmoni_pytree.leaves.speaker_service import SpeakerServicePytree
from harmoni_pytree.leaves.aws_tts_service import AWSTtsServicePytree
from harmoni_pytree.leaves.facial_exp_service import FacialExpServicePytree

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
    content += "Tree having sequential children: Text-to-Speech and Speaker\n"
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


def create_root(name= "Face_Polly"):
    """Method to create the composites of the leaves

    Args:
        name (str, optional): Name of the subtree. Defaults to "Face_Polly".

    Returns:
        py_trees.behaviour.Behaviour: Behaviour subtree
    """
    tts = AWSTtsServicePytree("TTSactivity")
    speaker = SpeakerServicePytree("SpeakerActivity")
    face = FacialExpServicePytree("FaceActivity")
    face_speaker = py_trees.composites.Parallel(name="face_speaker")
    face_speaker.add_children([speaker, face])
    root = py_trees.composites.Sequence(name=name)
    root.add_children([tts, face_speaker])

    return root

##############################################################################
# Main
##############################################################################

def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    root = create_root()
    print(description(root))

    blackboard_tts = py_trees.blackboard.Client(name="blackboard_tts", namespace=ActuatorNameSpace.tts.name)
    blackboard_tts.register_key("result", access=py_trees.common.Access.READ)
    print(blackboard_tts)

    blackboard_bot = py_trees.blackboard.Client(name="blackboard_bot", namespace=DialogueNameSpace.bot.name +"/"+PyTreeNameSpace.trigger.name)
    blackboard_bot.register_key("result", access=py_trees.common.Access.READ)
    print(blackboard_bot)

    blackboard_scene = py_trees.blackboard.Client(name="blackboard_scene", namespace=PyTreeNameSpace.scene.name)
    blackboard_scene.register_key("face_exp", access=py_trees.common.Access.READ)
    print(blackboard_scene)

    rospy.init_node("face_polly_node", log_level=rospy.INFO)

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
            time.sleep(1)
            print(blackboard_tts)
            print(blackboard_bot)
            print(blackboard_scene)
        except KeyboardInterrupt:
            break
    print("\n")


if __name__ == "__main__":
    main()
