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
import rospkg
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
from harmoni_pytree.leaves.misty_change_led_py_tree import MistyChangeLEDPyTree
from harmoni_pytree.leaves.check_thinking_pytree import CheckThinkingPyTree
from harmoni_pytree.leaves.play_audio_service import PlayAudioServicePytree
from harmoni_pytree.leaves.misty_arms_service import MistyArmsMovementServicePyTree
from harmoni_pytree.leaves.misty_drive_pytree import MistyDriveMovementServicePyTree
from harmoni_pytree.leaves.vad_service import VADServicePytree
from py_trees.behaviours import CheckBlackboardVariableValue
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

    output = py_trees.display.unicode_blackboard(key_filter=['/harmoni/actuating/head/head_position'])
    print(output)
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
    face_det = FaceDetectionServicePytree("FaceDetection")
    face_tracker = FaceTrackingServicePyTree("FaceTracking")
    sound_after_talking = PlayAudioServicePytree("SoundAfterTalking", folder=rospkg.RosPack().get_path("harmoni_pytree") + "/audio_resources/breath",script_sound=True)
    change_led_to_green = MistyChangeLEDPyTree("ChangeColorToGreen",color={"red":0,"green":255,"blue":0})
    change_led_to_red = MistyChangeLEDPyTree("ChangeColorToRed",color={"red":255,"green":0,"blue":0})
    check_thinking = CheckThinkingPyTree("CheckThinking")
    thinking_motion = MistyHeadMovementServicePyTree("ThinkingMotionStart", movements=[[{"roll":-30,"pitch":-30,"duration":1.5}],[{"roll":30,"pitch":-30,"duration":1.5}]], random=True)
    thinking_motion_end = MistyHeadMovementServicePyTree("ThinkingMotionEnd", movements=[[{"roll": 0, "pitch": -10, "duration":1}]],random=True)
    #checkstt = CheckSTTResult("CheckSTTResult", params)
    play_thinking_hmm = PlayAudioServicePytree("PlayThinkingHmm",folder=rospkg.RosPack().get_path("harmoni_pytree") + "/audio_resources/hmms")
    hmm_timer = py_trees.timers.Timer("TimerHmmSound", duration=0.8)
    move_arms = MistyArmsMovementServicePyTree("MoveArms",script_gesture=True,gesture_filename="misty_gestures")
    move_body = MistyDriveMovementServicePyTree("MoveBody",script_gesture=True,gesture_filename="misty_drive")
    move_head = MistyHeadMovementServicePyTree("MoveHead",script_gesture=True,gesture_filename="misty_head")
    comparison_for_scene_counter = py_trees.common.ComparisonExpression(
        variable="scene/scene_counter",
        value=2,
        operator= operator.le
    )

    check_for_beginning_scene = CheckBlackboardVariableValue(
        name="CheckBeginningScene",
        check=comparison_for_scene_counter
    )

    #comparison_true_if_talking = py_trees.common.ComparisonExpression(
    #    variable="vad/result",
    #    value=True,
    #    operator= operator.eq
    #)

    #comparison_true_if_silent = py_trees.common.ComparisonExpression(
    #     variable="vad/result",
    #    value=str(False),
     #   operator= operator.eq
    #)
    
    #check_for_talking = CheckBlackboardVariableValue(
    #    name="CheckTalking",
    #    check=comparison_true_if_talking
    #)

    #check_for_silence = CheckBlackboardVariableValue(
    #    name="CheckSilence",
  #      check=comparison_true_if_silent
  #  )

    nodding_behavior = MistyHeadMovementServicePyTree("Nodding", movements=[[{"pitch": -12,"roll":0, "duration":0.1},{"pitch": -8,"roll":0, "duration":0.1}]])
    inviting_to_talk_behavior = MistyHeadMovementServicePyTree("InvitingToTalk", movements=[[{"roll":-20, "duration":3}],[{"roll":20,"duration":3}]])

    #nodding_selector = py_trees.composites.Selector("NoddingSelector", memory=True)
    #If silent returns SUCCESS we do not need to nod
    #nodding_selector.add_children([check_for_silence, nodding_behavior])
    #inviting_to_talk_selector = py_trees.composites.Selector("InvitingToTalkSelector", memory=True)
    #If talking returns SUCCESS we do not need to invite to talk
    #inviting_to_talk_selector.add_children([check_for_talking, inviting_to_talk_behavior])
    #Face starts usually before the speech, inserting a timer to make them synchronous
    lips_timer = py_trees.timers.Timer("TimerFaceStart", duration=1.0)
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
    #SPEAKING SELECTOR. Gives priority to Thinking, when thinking "FAILS" (finished creating utterance) starts speaking. 
    # When speaking "FAILS" (yielding turn) returns FAILURE hence goes to the sensing sequence
    speaking_selector = py_trees.composites.Selector("SpeakingSelector", memory=True)
    #SPEAKING
    sequence_speaking = py_trees.composites.Sequence("Speaking", memory=True)
    #SENSING
    sequence_sensing = py_trees.composites.Sequence("Sensing", memory=True)
    listening_sequence = py_trees.composites.Sequence("Listening_Sequence", memory=True)
    listening_sequence.add_children([microphone,stt])
    #inviting_to_talk_selector,nodding_selector
    listening_parallel = py_trees.composites.Parallel("Listening_Parallel",children=[listening_sequence, inviting_to_talk_behavior],policy = py_trees.common.ParallelPolicy.SuccessOnSelected(children=[listening_sequence],synchronise=False))
    lips_sequence = py_trees.composites.Sequence("Lips", memory=True)
    lips_sequence.add_children([lips_timer,face])
    #LIPSYNC PARALLEL
    lipsync_parallel = py_trees.composites.Parallel("LipsSpeechArms",children=[speaker,lips_sequence,move_arms, move_body, move_head],policy = py_trees.common.ParallelPolicy.SuccessOnSelected(children=[speaker, lips_sequence],synchronise=True))
    #THINKING PARALLEL
    thinking_parallel = py_trees.composites.Parallel("Thinking",policy = py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True))
    thinking_sequence = py_trees.composites.Sequence("ThinkingSequence", memory=True)
    thinking_sequence.add_children([script, chatbot,check_thinking])
    #In the first scene, to avoid thinking motion I use the check_for_beginning_scene in the selector
    #It will return SUCCESS if the scene_counter is 0, otherwise it will return FAILURE and will activate the thinking motion
    thinking_audio_sequence = py_trees.composites.Sequence("AudioHmmSequence",memory=True)
    thinking_audio_sequence.add_children([hmm_timer,play_thinking_hmm])
    thinking_parallel.add_children([thinking_motion,checkstt, thinking_audio_sequence])
    
    thinking_motion_end_selector = py_trees.composites.Selector("ThinkingMotionEndSelector", memory=True)
    thinking_motion_end_selector.add_children([check_for_beginning_scene,thinking_motion_end])
    #SPEAKING SEQUENCE ADD
    sequence_speaking.add_children([tts,thinking_motion_end_selector,face_tracker,lipsync_parallel, sound_after_talking, check_speaking_turn])
    #SPEAKING SELECTOR ADD
    speaking_selector.add_children([thinking_sequence, sequence_speaking])
    #SENSING SEQUENCE ADD
    sequence_sensing.add_children([change_led_to_green,listening_parallel,change_led_to_red, thinking_parallel])
    #DIALOGUE SEQUENCE ADD
    dialogue_selector.add_children([speaking_selector, sequence_sensing])
    dialogue_sequence.add_children([dialogue_selector])
    interaction_parallel.add_children([face_det, dialogue_sequence])
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
        behaviour_tree.setup(timeout=40)
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


