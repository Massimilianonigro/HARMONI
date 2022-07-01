#!/usr/bin/env python3

# Common Imports
import rospy, rospkg, roslib

from harmoni_common_lib.action_client import HarmoniActionClient
from actionlib_msgs.msg import GoalStatus

# Specific Imports
# from harmoni_web.web_service import WebService
from harmoni_common_lib.constants import *
import time


#py_tree
import py_trees

class WebServicePytree(py_trees.behaviour.Behaviour):
    """
    This class is a child class of behaviour class of pytree module. It sends requests to harmoni action
    client and updates the leaf status according to the goal status. 
    """
    def __init__(self, name = "WebServicePytree", test_mode=False, test_input=None):
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
        self.service_client_web = None
        self.server_state = None
        self.server_name = None
        self.client_result = None
        self.old_image = None

        # Initialising blackboard
        self.blackboards = []

        # blackboard to store the image data
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        #self.blackboard_scene.image = 
        if test_mode:
            self.blackboard_scene.register_key("image", access=py_trees.common.Access.WRITE)
            if test_input is None:
                self.blackboard_scene.image = "[{'component_id':'img_only', 'set_content':'https://www.google.it/images/branding/googlelogo/2x/googlelogo_color_160x56dp.png'},{'component_id':'raccolta_container', 'set_content': ''}]"
            else:
                self.blackboard_scene.image = test_input
        else:
            self.blackboard_scene.register_key("image", access=py_trees.common.Access.READ)
        super(WebServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """Setting up of action client used for sending goals to the action server. Needs
            to called manually.
        """
                
        self.service_client_web = HarmoniActionClient(self.name)
        self.server_name = "web_default"
        self.service_client_web.setup_client(self.server_name,
                                            self._result_callback,
                                            self._feedback_callback)
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
        if self.old_image != self.blackboard_scene.image:
            self.logger.debug(f"Sending goal to {self.server_name}")
            # client sending goal to the action server
            # the goal to send is of type DO
            self.service_client_web.send_goal(
                action_goal = ActionType["DO"].value,
                optional_data = self.blackboard_scene.image,
                wait=False,
            )
            self.logger.debug(f"Goal sent to {self.server_name}")
            self.old_image = self.blackboard_scene.image
            new_status =  py_trees.common.Status.RUNNING
        else:
            new_state = self.service_client_web.get_state()
            print(new_state)
            if new_state == GoalStatus.ACTIVE:
                new_status = py_trees.common.Status.SUCCESS
            elif new_state == GoalStatus.SUCCEEDED:
                new_status = py_trees.common.Status.SUCCESS
            elif new_state == GoalStatus.PENDING:
                self.send_request = True
                self.logger.debug(f"Cancelling goal to {self.server_name}")
                self.service_client_yolo.cancel_all_goals()
                self.client_result = None
                self.logger.debug(f"Goal cancelled to {self.server_name}")
                new_status = py_trees.common.Status.RUNNING
            else:
                new_status = py_trees.common.Status.FAILURE
                raise

        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

    def terminate(self, new_status):
        """This function is called whenever the behaviour switches to a non-running state(SUCCESS or FAILURE or ....).

        Args:
            new_status (py_trees.common.Status): The function is called with this parameter having the status of the behavior tree
        """
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
    #command_line_argument_parser().parse_args()

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    rospy.init_node("web_default" , log_level=rospy.INFO)

    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=PyTreeNameSpace.scene.name)
    blackboardProva.register_key("image", access=py_trees.common.Access.WRITE)
    print(blackboardProva)

    blackboardProva.image = "[{'component_id':'img_only', 'set_content':'https://www.google.it/images/branding/googlelogo/2x/googlelogo_color_160x56dp.png'},{'component_id':'raccolta_container', 'set_content': ''}]"

    yoloPyTree = WebServicePytree("WebServicePytreeTest")

    additional_parameters = dict([
        ("WebServicePytree_mode",False)])

    yoloPyTree.setup(**additional_parameters)
    try:
        for unused_i in range(0, 16):
            yoloPyTree.tick_once()
            if unused_i%2:
                blackboardProva.image = "[{'component_id':'img_only', 'set_content':'https://firebasestorage.googleapis.com/v0/b/harmonithesis.appspot.com/o/land.jpeg?alt=media&token=79c113ec-4e61-49c1-bbdf-b865a946247e'},{'component_id':'raccolta_container', 'set_content': ''}]"
            else:
                blackboardProva.image = "[{'component_id':'img_only', 'set_content':'https://firebasestorage.googleapis.com/v0/b/harmonithesis.appspot.com/o/land2.jpeg?alt=media&token=a3437794-a763-4b47-9052-485fd5c61ae5'},{'component_id':'raccolta_container', 'set_content': ''}]"
            print(blackboardProva)
            time.sleep(2)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()