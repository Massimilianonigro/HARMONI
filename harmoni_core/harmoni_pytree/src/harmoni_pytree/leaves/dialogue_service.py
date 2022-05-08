#!/usr/bin/env python3

# Common Imports
import rospy
from harmoni_common_lib.constants import *
from actionlib_msgs.msg import GoalStatus

# Specific Imports
from harmoni_common_lib.constants import ActionType, DialogueNameSpace

#py_tree
import py_trees
import time

import py_trees.console

class DialogueServicePytree(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.name = name

        super(DialogueServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        rospy.Subscriber("/audio/audio", AudioData, self.playing_sound_pause_callback)
        rospy.Subscriber(
            SensorNameSpace.microphone.value + self.subscriber_id,
            AudioData,
            self.sound_data_callback,
        )
        self.text_pub = rospy.Publisher(
            DetectorNameSpace.stt.value + self.service_id, String, queue_size=10
        )
        

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):              
        if self.send_request:
            self.send_request = False
            self.logger.debug(f"Sending goal to {self.server_name}")
            self.service_client_lex.send_goal(
                action_goal = ActionType["REQUEST"].value,
                optional_data=self.blackboard_scene.utterance,
                wait=False,
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            new_status = py_trees.common.Status.RUNNING
        else:
            new_state = self.service_client_lex.get_state()
            print("update : ",new_state)
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                if self.client_result is not None:
                    self.blackboard_bot.result = eval(self.client_result)
                    self.client_result = None
                    new_status = py_trees.common.Status.SUCCESS
                else:
                    self.logger.debug(f"Waiting fot the result ({self.server_name})")
                    new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.PENDING:
                self.send_request = True
                self.logger.debug(f"Cancelling goal to {self.server_name}")
                self.service_client_lex.cancel_all_goals()
                self.client_result = None
                self.logger.debug(f"Goal cancelled to {self.server_name}")
                #self.service_client_lex.stop_tracking_goal()
                #self.logger.debug(f"Goal tracking stopped to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                new_status = py_trees.common.Status.FAILURE
                raise

        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

        

    def terminate(self, new_status):
        new_state = self.service_client_lex.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_lex.cancel_all_goals()
            self.client_result = None
            self.logger.debug(f"Goal cancelled to {self.server_name}")
            #self.service_client_lex.stop_tracking_goal()
            #self.logger.debug(f"Goal tracking stopped to {self.server_name}")
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        self.logger.debug("The result of the request has been received")
        self.logger.debug(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_result = result['message']
        return

    def _feedback_callback(self, feedback):
        """ Feedback is currently just logged """
        self.logger.debug("The feedback recieved is %s." % feedback)
        self.server_state = feedback["state"]
        return

def main():
    #command_line_argument_parser().parse_args()

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=PyTreeNameSpace.scene.name)
    blackboardProva.register_key("utterance", access=py_trees.common.Access.WRITE)
    blackboardProva.utterance = "domanda raccolta"
    blackboardProva2 = py_trees.blackboard.Client(name="blackboardProva2", namespace=DialogueNameSpace.bot.name+"/"+PyTreeNameSpace.trigger.name)
    blackboardProva2.register_key("result", access=py_trees.common.Access.READ)                        
    print(blackboardProva)
    print(blackboardProva2)

    rospy.init_node("bot_default", log_level=rospy.INFO)
    
    awslexPyTree = AWSLexTriggerServicePytree("AWSLexTriggerServicePytreeTest")
    awslexPyTree.setup()
    try:
        for unused_i in range(0, 10):
            awslexPyTree.tick_once()
            time.sleep(0.5)
            print(blackboardProva)
            print(blackboardProva2)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()
