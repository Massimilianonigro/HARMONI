#!/usr/bin/env python3

##############################################################################
# Imports
##############################################################################

import argparse
import functools
import py_trees.console as console
import py_trees
import rospy
import operator
from harmoni_common_lib.constants import *
from harmoni_pytree.leaves.lip_sync_service import LipSyncServicePytree
from harmoni_pytree.leaves.aws_tts_service import AWSTtsServicePytree
from harmoni_pytree.leaves.script_service import ScriptService
from harmoni_pytree.leaves.chat_gpt_service import ChatGPTServicePytree
from harmoni_pytree.leaves.speaker_service import SpeakerServicePytree
from harmoni_pytree.leaves.microphone_service import MicrophoneServicePytree
from harmoni_pytree.leaves.check_stt_result import CheckSTTResult
from harmoni_pytree.leaves.google_service import SpeechToTextServicePytree
from harmoni_pytree.leaves.face_detection_service import FaceDetectionServicePytree
from harmoni_pytree.leaves.face_tracking_service import FaceTrackingServicePyTree
from harmoni_pytree.leaves.check_speaking_turn_script import CheckSpeakingTurnScript
from harmoni_pytree.leaves.misty_head_movement_service import MistyHeadMovementServicePyTree
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
    sleep_time = 10
    rospy.loginfo("WAITING " + str(sleep_time) + " seconds to let services start up before leaves")
    rospy.sleep(sleep_time)
    face = LipSyncServicePytree("Face")
    script = ScriptService("Script", params)
    chatbot = ChatGPTServicePytree("ChatGPTPyTreeTest")
    tts = AWSTtsServicePytree("TextToSpeech")
    speaker = SpeakerServicePytree("Speaker")
    microphone=MicrophoneServicePytree("Microphone")
    stt=SpeechToTextServicePytree("SpeechToText")
    checkstt = CheckSTTResult("CheckResults", params)

    #Face starts usually before the speech, inserting a timer to make them synchronous
    lips_timer = py_trees.timers.Timer("TimerFaceStart", duration=0.8)
    check_speaking_turn = CheckSpeakingTurnScript("CheckSpeakingTurn", params)
    #vad = VADServicePytree("VAD")
    #ROOT
    #DETECTOR
    interaction_parallel = py_trees.composites.Parallel("Interaction", policy = py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    #detectors_parallel.add_children([vad])
    #DIALOGUE
    dialogue_sequence = py_trees.composites.Sequence("Dialogue", memory=True)
    #The idea is to have a leaf at the end of the speaking return FAILURE if yielding speaking turn
    #So that the sensing takes over or maintaining the turn returning SUCCESS
    dialogue_selector = py_trees.composites.Selector("DialogueSelector", memory=True)
    #SPEAKING
    sequence_speaking = py_trees.composites.Sequence("Speaking", memory=True)
    #SENSING
    sequence_sensing = py_trees.composites.Sequence("Sensing", memory=True)
    lips_sequence = py_trees.composites.Sequence("Lips", memory=True)
    lips_sequence.add_children([lips_timer,face])
    #LIPSYNC PARALLEL
    lipsync_parallel = py_trees.composites.Parallel("LipsAndSpeech",policy = py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True))
    lipsync_parallel.add_children([speaker,lips_sequence])
    #THINKING PARALLEL
    thinking_sequence = py_trees.composites.Sequence("ThinkingSequence", memory=True)
    thinking_sequence.add_children([script, chatbot,tts])
    #In the first scene, to avoid thinking motion I use the check_for_beginning_scene in the selector
    #It will return SUCCESS if the scene_counter is 0, otherwise it will return FAILURE and will activate the thinking motion
    #SPEAKING SEQUENCE ADD
    sequence_speaking.add_children([thinking_sequence,lipsync_parallel, check_speaking_turn])
    #SENSING SEQUENCE ADD
    sequence_sensing.add_children([microphone, stt, checkstt])
    #DIALOGUE SEQUENCE ADD
    dialogue_selector.add_children([sequence_speaking, sequence_sensing])
    dialogue_sequence.add_children([dialogue_selector])
    interaction_parallel.add_children([dialogue_sequence])
    #ROOT SEQUENCE ADD
    return interaction_parallel

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


