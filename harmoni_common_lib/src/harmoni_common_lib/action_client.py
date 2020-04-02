#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import actionlib
from harmoni_common_msgs.msg import harmoniGoal, harmoniAction


class HarmoniActionClient():
    """
    Controllers and manager are clients.
    This class provides basic client functionality which controller and manager extend,
    including basic type checking, warnings, interrupts, etc.
    """
    def __init__(self):
        """ Initialization of the variables """
        self.set_com_flag_variables()

    def set_com_flag_variables(self):
        """ Initizalize or reset of communication flag variables """
        self.result_received = False
        self.feedback_received = False
        return

    def init_action_variables(self):
        """ Initizalize or reset action msg variables """
        self.action_result = {}
        self.action_feedback = {}
        self.action_result = {
            "do_continue": False,
            "message": ""
        }
        self.action_feedback = {
            "state": ""
        }

    def setup_client(self, action_type_name, execute_goal_result_callback, execute_goal_feedback_callback, wait=True):
        """ Init client action variables and setup client """
        self.init_action_variables()
        self.action_client = actionlib.SimpleActionClient(action_type_name, harmoniAction)
        if wait:
            self.action_client.wait_for_server()
        self.execute_goal_result_callback = execute_goal_result_callback
        self.execute_goal_feedback_callback = execute_goal_feedback_callback
        return

    def action_result_callback(self, terminal_stale, result):
        """ Save the action result """
        rospy.loginfo("Heard back result from: " + result.action)
        self.action_result["do_continue"] = result.do_continue
        self.action_result["message"] = result.message
        self.execute_goal_result_callback(self.action_result)
        return

    def action_feedback_callback(self, feedback):
        """ Save the action feedback """
        rospy.loginfo("Heard back feedback from: " + feedback.action)
        self.action_feedback["state"] = feedback.state
        self.execute_goal_feedback_callback(self.action_feedback)
        return

    def feedback_received(self):
        """ Check if feedback received """
        if self.feedback_received:
            self.feedback_received = False
            return True
        else:
            return False

    def result_received(self):
        """ Check if result received """
        if self.result_received:
            result = True
        else:
            result = False
        return result

    def feedback_data(self):
        """Return Feedback Data"""
        return(self.action_feedback)

    def result_data(self):
        """Return Result Data"""
        return(self.action_result)

    def send_goal(self, action_goal, optional_data="", child="", condition="", time_out=600):
        """ Reset of check variables. Send goal and set the time out """
        self.set_com_flag_variables()
        goal = harmoniGoal(action=action_goal, optional_data=optional_data, child=child, condition=condition)
        self.action_client.send_goal(goal, done_cb=self.action_result_callback, feedback_cb=self.action_feedback_callback)
        self.action_client.wait_for_result(rospy.Duration.from_sec(time_out))
        return
