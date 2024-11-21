#!/usr/bin/env python3

##############################################################################
# Imports
##############################################################################

import argparse
import functools
import py_trees.console as console
import py_trees
import rospy
from harmoni_common_lib.constants import *
from harmoni_pytree.leaves.lip_sync_service import LipSyncServicePytree
from harmoni_pytree.leaves.aws_tts_service import AWSTtsServicePytree
from harmoni_pytree.leaves.script_service import ScriptService
from harmoni_pytree.leaves.chat_gpt_service import ChatGPTServicePytree
from harmoni_pytree.leaves.speaker_service import SpeakerServicePytree
from harmoni_pytree.leaves.microphone_service import MicrophoneServicePytree
from harmoni_pytree.leaves.check_stt_result import CheckSTTResult
from harmoni_pytree.leaves.google_service import SpeechToTextServicePytree
from harmoni_pytree.leaves.vad_service import VADServicePytree
import traceback

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
    #print(py_trees.display.unicode_blackboard())


def post_tick_handler(snapshot_visitor, behaviour_tree):
    print(
        "\n" + py_trees.display.unicode_tree(
            root=behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited
        )
    )
    #print(py_trees.display.unicode_blackboard())

def create_root(params):
    #Leaves
    face = LipSyncServicePytree("Face")
    script = ScriptService("Script", params)
    chatbot = ChatGPTServicePytree("ChatGPTPyTreeTest")
    tts = AWSTtsServicePytree("TextToSpeech")
    speaker = SpeakerServicePytree("Speaker")
    microphone=MicrophoneServicePytree("Microphone")
    stt=SpeechToTextServicePytree("SpeechToText")
    checkstt = CheckSTTResult("CheckResults", params)
    #vad = VADServicePytree("VAD")
    #ROOT
    #DETECTOR
    #detectors_parallel = py_trees.composites.Parallel("Detectors", policy = py_trees.common.ParallelPolicy.SuccessOnAll)
    #detectors_parallel.add_children([vad])
    #DIALOGUE
    dialogue_sequence = py_trees.composites.Sequence("Dialogue", memory=True)
    #SPEAKING
    sequence_speaking = py_trees.composites.Sequence("Speaking", memory=True)
    #SENSING
    sequence_sensing = py_trees.composites.Sequence("Sensing", memory=True)
    #SPEAKING SEQUENCE ADD
    sequence_speaking.add_children([script,chatbot,tts,speaker, face])
    #SENSING SEQUENCE ADD
    sequence_sensing.add_children([microphone, stt, checkstt])
    #DIALOGUE SEQUENCE ADD
    dialogue_sequence.add_children([sequence_speaking, sequence_sensing])
    #ROOT SEQUENCE ADD
    return dialogue_sequence

 ##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    params = rospy.get_param("pytree/default_param/")
    root =create_root(params)
    print("########################################################")

    print(description(root))
        
    ####################
    # Tree Stewardship
    ####################

    rospy.init_node("test_default", log_level=rospy.INFO)
    
    behaviour_tree = py_trees.trees.BehaviourTree(root)
    print(py_trees.display.unicode_tree(root=root))

    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    #behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.visitors.append(snapshot_visitor)
    behaviour_tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
    try:
        behaviour_tree.setup(timeout=20)
    except:
        print(traceback.format_exc())
        return

    ####################
    # Tick Tock
    ####################

    try:
        behaviour_tree.tick_tock(
            period_ms=500,
            number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
        )
    except:
        print("--------------------INTERRUPT------------------------")
        behaviour_tree.interrupt()
    print("\n")

if __name__ == "__main__":
    main()   


