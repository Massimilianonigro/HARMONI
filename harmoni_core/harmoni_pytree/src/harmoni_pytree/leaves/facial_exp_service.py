#!/usr/bin/env python3

# Common Imports
import rospy, rospkg, roslib
from harmoni_common_lib.constants import *
from harmoni_common_lib.action_client import HarmoniActionClient
from actionlib_msgs.msg import GoalStatus

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType
import time

#py_tree
import py_trees

class FacialExpServicePytree(py_trees.behaviour.Behaviour):
    """
    This class is a child class of behaviour class of pytree module. It sends requests to harmoni action
    client and updates the leaf status according to the goal status.
    """
    def __init__(self, name, test_mode=False, test_input=None):
        """Constructor for initializing blackboard and their keys

        Args:
            name (str): Name of the pytree
            test_mode (bool, optional): The mode of running the leaf. If set to true, 
            blackboard keys are given WRITE access for initialization with a value. Defaults to False.
            test_input (list, optional): The input to the blackboard keys for testing the leaf. If None,
            then deafult value is given to the blackboard keys which will be used as test input. Defaults to None.
        """
        # Attribute initialization
        self.name = name
        self.service_client_mouth = None
        self.service_client_eyes = None
        self.service_client_nose = None
        self.client_result = None
        self.send_request = True

        self.blackboards = []

        # blackboard containing the data for generating facial expressions
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        if test_mode:
            self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
            if test_input is None:
                self.blackboard_scene.face_exp = "[{'start': 1, 'type': 'viseme', 'id': 'POSTALVEOLAR'}]"
            else:
                self.blackboard_scene.face_exp = test_input
        else:
            self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.READ)

        super(FacialExpServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """Setting up of action client used for sending goals to the action server. Needs
            to called manually.
        """
        # setting up various action clients 
        self.server_name = "face"
        self.instance_id = "default"
        self.name_mouth = ActuatorNameSpace.face.name + "_mouth_" + self.instance_id
        self.service_client_mouth = HarmoniActionClient(self.name_mouth)
        self.name_nose = ActuatorNameSpace.face.name + "_nose_" + self.instance_id
        self.service_client_nose = HarmoniActionClient(self.name_nose)
        self.name_eyes = ActuatorNameSpace.face.name + "_eyes_" + self.instance_id
        self.service_client_eyes = HarmoniActionClient(self.name_eyes)

        self.service_client_eyes.setup_client(self.name_eyes, self._result_callback, self._feedback_callback)
        self.service_client_mouth.setup_client(self.name_mouth, self._result_callback, self._feedback_callback)
        self.service_client_nose.setup_client(self.name_nose, self._result_callback, self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """Method that is called before starting the ticks.
        """     
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        """This is called every time the behaviour tree is ticked. Sending of request to the action server is done here.
        Further status of the goal is updated here.

        Returns:
            py_trees.common.Status: Status of the task 
        """     
        
        # if request is to be sent
        if self.send_request:
            self.send_request = False
            self.data = self.blackboard_scene.face_exp
            self.logger.debug(f"Sending goal to {self.server_name}")
            
            # client sends goal to the action server
            self.service_client_mouth.send_goal(
                action_goal=ActionType.DO.value,
                optional_data=self.data,
                wait=True,
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            new_status = py_trees.common.Status.RUNNING
        else:
            # update state based on goal status
            new_state = self.service_client_mouth.get_state()
            print(new_state)
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.RUNNING
            elif new_state == GoalStatus.SUCCEEDED:
                new_status = py_trees.common.Status.SUCCESS
            else:
                new_status = py_trees.common.Status.FAILURE

        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status 

    def terminate(self, new_status):
        """This function is called whenever the behaviour switches to a non-running state(SUCCESS or FAILURE or ....).

        Args:
            new_status (py_trees.common.Status): The function is called with this parameter having the status of the behavior tree
        """      
        new_state = self.service_client_mouth.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_mouth.cancel_all_goals()
            self.client_result = None
            self.logger.debug(f"Goal cancelled to {self.server_name}")
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

    def _result_callback(self, result):
        """This function is called when the action client receives a reult of the goal sent. Update 
        of client_result takes place here which is used for updating blackboard key

        Args:
            result (dict): Contains the result of the goal sent by the client from harmoni action server
        """
        self.logger.debug("The result of the request has been received")
        self.logger.debug(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_result = result["message"]
        return

    def _feedback_callback(self, feedback):
        """This function is called by action client when it receives feedback from the action server
        Args:
            feedback (dict): Contains the feedback sent by the harmoni action server.
        """
        self.logger.debug("The feedback recieved is %s." % feedback)
        self.server_state = feedback["state"]
        return

def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=PyTreeNameSpace.scene.name)
    blackboardProva.register_key("face_exp", access=py_trees.common.Access.WRITE)

    blackboardProva.face_exp = "[{'start': 1, 'type': 'viseme', 'id': 'POSTALVEOLAR'}]"
    
    """
    [{'start':10, 'type': 'gaze', 'id':'target', 'point': [1,5,10]}]
    [{'start': 1, 'type': 'au', 'id': 'au13', 'pose': 1}]
    [{'start': 2, 'type': 'action', 'id': 'breath_face'}]
    [{'start': 5, 'type': 'action', 'id': 'saucy_face'}]
    [{'start': 8, 'type': 'viseme', 'id': 'POSTALVEOLAR'}]
    """

    rospy.init_node("face_default", log_level=rospy.INFO)

    facePyTree = FacialExpServicePytree("FaceServiceTest")

    facePyTree.setup()
    try:
        for unused_i in range(0, 12):
            facePyTree.tick_once()
            time.sleep(0.5)
            print(blackboardProva)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()
