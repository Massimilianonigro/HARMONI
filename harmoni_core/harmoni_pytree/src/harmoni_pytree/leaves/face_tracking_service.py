#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import py_trees
import rospkg
import random
import rospy
import yaml
from harmoni_common_lib.constants import PyTreeNameSpace, DetectorNameSpace, ActuatorNameSpace
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import ActionType
from actionlib_msgs.msg import GoalStatus
from enum import Enum

class FaceSearchPolicy(Enum):
    LOOK_LEFT = 0
    LOOK_RIGHT = 1
    STAY_STILL = 2

TURNING_DURATION = 1.8
ADJUSTING_DURATION = 0.8
SEARCHING_DURATION = 0.5
class FaceTrackingServicePyTree(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.name = name
        self.blackboards = []
        self.last_addressee = ""
        self.last_postion = [0,0,0]
        self.service_client_head_movement = None
        #Last Areas of the face on the left and on the right seen 
        self.last_areas = [0,0]
        self.accepted_area_similarity = 0.4
        self.accepted_position_similarity = 0.3
        #Desired Center where we would like the face, indicated in percentage than the total dimension 
        self.desired_center_x = 0.5
        self.desired_center_y = 0.5
        #Last good position for left person (index 0) or right person (index 1). With position we refer to the 3 degrees of freedom of Misty's head (roll, pitch, yaw)
        self.last_good_positions = [[0,-8,-30],[0,-8,30]] 
        self.last_search_positions = [None, None]
        self.left_right_idxs = {"left":0, "right":1}
        self.default_good_position = self.last_good_positions
        #TODO: Adjust this params so that they work with normal camera. (Right now they are thought for Misty Camera)
        self.total_img_width = rospy.get_param("/camera/default_param/width")
        self.total_img_height = rospy.get_param("/camera/default_param/height")
        #Max Degrees of robot movement on roll, pitch and yaw axis used to get a face from seen to the center
        self.max_robot_movement = [0,10,45]
        #Maximum values on the roll, pitch, and yaw admitted by Misty's head
        self.max_robot_movement_possible_values = [0,15,80]
        #Maximum searching movement
        self.searching_mov_perc = 0.15
        #X and Y can be this amount off from the center of the image w.r.t the full image dimension to be accepted as good
        self.accepted_zone = 0.25
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        #Addressee is either left or right
        self.blackboard_scene.register_key(key="addressee", access=py_trees.common.Access.READ)
        self.blackboard_scene.register_key(key="addressee_name", access=py_trees.common.Access.READ)
        self.blackboard_face_det = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.face_detect.name)
        self.blackboard_face_det.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_head = self.attach_blackboard_client(name=self.name, namespace=ActuatorNameSpace.head.name)
        self.blackboard_head.register_key("head_position", access=py_trees.common.Access.WRITE)
        super(FaceTrackingServicePyTree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.send_request = True

    def setup(self):
        #this is the name of the json without the extension
        self.service_client_head_movement = HarmoniActionClient(self.name)
        self.server_name = "head_default"
        self.service_client_head_movement.setup_client(self.server_name, 
                                            self._result_callback,
                                            self._feedback_callback)
        self.logger.debug("Behavior %s interface action clients have been set up!" % (self.server_name))
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        
    def initialise(self):
        self.logger.debug("  %s [ScriptService::initialise()]" % self.name)

    def update(self):
        #If there is no past addressee or the addressee has changed, we quickly move to the last good position for that addresse (face is probably not in the field of view of the camera)
        new_head_positions = [0,0,0]
        current_addressee = self.blackboard_scene.addressee
        if self.service_client_head_movement.get_state() == GoalStatus.ACTIVE:
            return py_trees.common.Status.RUNNING
        if self.last_addressee == "" or self.last_addressee != current_addressee:
            rospy.loginfo("IN UPDATE IF PART")
            if current_addressee in ["left","right"]: 
                new_head_positions[1] = self.last_good_positions[self.left_right_idxs[current_addressee]][1]  
                new_head_positions[2] = self.last_good_positions[self.left_right_idxs[current_addressee]][2]
            else:
                raise ValueError("Addressee not set into the script or set to unknown value")
            self.service_client_head_movement.send_goal(
                        action_goal = ActionType["DO"].value,
                        optional_data=str({"roll":new_head_positions[0], "pitch":new_head_positions[1], "yaw":new_head_positions[2], "duration":TURNING_DURATION}),
                        wait=False)
            self.last_postion = new_head_positions
            self.blackboard_head.head_position = self.last_postion
            self.last_addressee = current_addressee
            return py_trees.common.Status.RUNNING
        #If the past addressee is equal to the current addressee, it means we are already in a good enough position and should be able to see the face of the person
        else:
            rospy.loginfo("IN UPDATE ELSE PART")
            #If we see multiple faces we choose the biggest one
            face, search_policy = self._get_face_or_search_policy(self._get_faces(self.blackboard_face_det.result["message"]))
            if face != None:
                rospy.loginfo("IN UPDATE FACE FOUND PART")
                #We calculate the position of the center of the face in the image, 
                #If the center is in the confidence zone, then we return success.
                #Otherwise we move the head to the center of the face and return FAILURE
                is_in_center, distances = self._is_face_in_the_center(face)
                if is_in_center:
                    self.last_addressee = current_addressee
                    self.last_good_positions[self.left_right_idxs[current_addressee]] = self.last_postion
                    self.last_areas[self.left_right_idxs[current_addressee]] = face["area"]
                    self.blackboard_head.head_position = self.last_postion
                    return py_trees.common.Status.SUCCESS
                else:
                    #Calculate the new head position, then return FAILURE
                    new_head_positions = self._calculate_new_head_position(distances)
                    self.last_postion = new_head_positions                    
                    self.service_client_head_movement.send_goal(
                        action_goal = ActionType["DO"].value,
                        optional_data=str({"roll":new_head_positions[0], "pitch":new_head_positions[1], "yaw":new_head_positions[2],"duration":ADJUSTING_DURATION}),
                        wait=False)
                    self.blackboard_head.head_position = self.last_postion
                    self.last_addressee = current_addressee
                    return py_trees.common.Status.RUNNING
            else:
                rospy.loginfo("IN UPDATE FACE NOT FOUND PART")
                #If face is not seen we should have something passed as Search Policy
                if search_policy == None:
                    #Some error happened we stay still
                    rospy.loginfo("Both Face and search policy turned out None, we stay still")
                    self.last_addressee = self.blackboard_scene.addressee
                    return py_trees.common.Status.RUNNING
                elif search_policy == FaceSearchPolicy.STAY_STILL:
                    self.last_addressee = self.blackboard_scene.addressee
                    return py_trees.common.Status.RUNNING
                else:
                    if self.last_search_positions[self.left_right_idxs[self.blackboard_scene.addressee]] == None:
                        self.last_search_positions[self.left_right_idxs[self.blackboard_scene.addressee]] = self.last_good_positions[self.left_right_idxs[self.blackboard_scene.addressee]]
                    if search_policy == FaceSearchPolicy.LOOK_LEFT:
                        #We aim to slighlty look left by 5 percent of the possible value
                        new_yaw = self.last_search_positions[self.left_right_idxs[self.blackboard_scene.addressee]][2] - self.max_robot_movement[2] * self.searching_mov_perc
                    else:
                        #We aim to slighlty look left by 5 percent of the possible value
                        new_yaw = self.last_search_positions[self.left_right_idxs[self.blackboard_scene.addressee]][2] + self.max_robot_movement[2] * self.searching_mov_perc
                    new_yaw = (new_yaw/abs(new_yaw)) * self.max_robot_movement_possible_values[2]if abs(new_yaw) > self.max_robot_movement_possible_values[2] else int(new_yaw)
                    new_head_positions = self.last_search_positions[self.left_right_idxs[self.blackboard_scene.addressee]][:2]
                    new_head_positions.append(new_yaw)
                    self.last_postion = new_head_positions  
                    self.last_search_positions[self.left_right_idxs[self.blackboard_scene.addressee]] = new_head_positions                  
                    self.service_client_head_movement.send_goal(
                        action_goal = ActionType["DO"].value,
                        optional_data=str({"roll":new_head_positions[0], "pitch":new_head_positions[1], "yaw":new_head_positions[2],"duration":SEARCHING_DURATION}),
                        wait=False)
                    self.blackboard_head.head_position = self.last_postion
                    self.last_addressee = current_addressee
                    return py_trees.common.Status.RUNNING

    def _get_faces(self, message):
        return json.loads(message)
    
    
    def _calculate_new_head_position(self, distances):
        #Distances are in percentage of the total image width and height
        #If the distance is negative (the face is above the center of the image) we have to move the head up thus subtracting distance
        #If the distance is positive (the face is below the center of the image) we have to move the head down thus adding distance (positive pitch is down)
        pitch = self.last_postion[1] + distances[1] * self.max_robot_movement[1]
        #If the distance is negative (the face is to the left of the center of the image) we have to move the head to the right thus adding distance
        #If the distance is positive (the face is to the right of the center of the image) we have to move the head to
        yaw = self.last_postion[2] - distances[0] * self.max_robot_movement[2]
        pitch = (pitch/abs(pitch)) * self.max_robot_movement_possible_values[1]  if abs(pitch) > self.max_robot_movement_possible_values[1] else pitch
        yaw = (yaw/abs(yaw)) * self.max_robot_movement_possible_values[2] if abs(yaw) > self.max_robot_movement_possible_values[2] else yaw
        return [0, round(pitch,1), round(yaw,1)]
    
    def _is_face_in_the_center(self, face):
        is_in_center = False
        #Get the center of the image
        img_center_x = self.total_img_width * self.desired_center_x
        img_center_y = self.total_img_height * self.desired_center_y
        #Distance from center of the face and center_x
        x_distance = (face["center_x"] - img_center_x)/self.total_img_width
        y_distance = (face["center_y"] - img_center_y)/self.total_img_height
        #If the distance is within the accepted zone, we return success
        if abs(x_distance) <= self.accepted_zone and abs(y_distance) <= self.accepted_zone:
            is_in_center = True
        return is_in_center, [x_distance, y_distance]

    #If we see multiple faces, we keep the two biggest faces
    #Given a set of faces in the form of an Object2DArray harmoni message,
    # calculate the area of each and choose the two biggest (People conversing with robot hopefully)
    def _choose_biggest_two_face(self, faces):
        if faces == None:
            return None
        for face in faces:
            face["area"] = face["width"] * face["height"]
        sorted_faces = sorted(faces, key=lambda k: k['area'], reverse=True)
        #Just the one face
        if len(faces) < 2:
            return sorted_faces
        #Otherwise return the biggest two
        return sorted_faces[:2]
    
    #TODO: Implement a set of rules that given 
    # - Last Position of the face
    # - Last Area of the face
    # - Addressee position (left or right)
    # - Addressee name (genitore o bambino)
    # - Gives back either
    #   - The center of the face if we are seeing it 
    #   - A policy if we do not see the face (Either where to look, or if we have to stay still)
    def _get_face_or_search_policy(self, faces):
        rospy.loginfo("IN GET FACE OR SEARCH POLICY")
        #First, if we have more than two faces, we would like to choose the biggest two which are likely the people interacting with the robot
        biggest_faces = self._choose_biggest_two_face(faces)
        rospy.loginfo("AFTER CHOOSE BIGGEST TWO FACE BIGGEST FACES ARE " + str(biggest_faces))
        if biggest_faces == None or len(biggest_faces) == 0:
            rospy.loginfo("BIGGEST FACES IS NONE")
            #We are not seeing any face,
            #If we never saw the face, we should probably look around,
            #If we saw the face, we should probably stay still (as we likely saved a good position for the face 
            # and it is unlikely the face moves that much so that we do not recognize it)
            if self.last_areas[self.left_right_idxs[self.blackboard_scene.addressee]] == 0:
                #We never saw the face, we should look around, if the face is on the left, we should look left and viceversa
                return None, FaceSearchPolicy.LOOK_LEFT if self.blackboard_scene.addressee == "left" else FaceSearchPolicy.LOOK_RIGHT
            else:
                #We saw the face, we should stay still
                return None, FaceSearchPolicy.STAY_STILL
        if len(biggest_faces) == 1:
            rospy.loginfo("BIGGEST FACES IS ONE")
            #We are seeing only one face - we have to decide if the face is the one we are looking for or not
            #If it is the first time seeing a face, we trust this judgement. 
            #If we saw the face before, we should check the face has relatively the same area and position
            if self._is_it_the_same_face(biggest_faces[0]):
                return biggest_faces[0], None
            else: 
                return None, FaceSearchPolicy.LOOK_LEFT if self.blackboard_scene.addressee == "left" else FaceSearchPolicy.LOOK_RIGHT
            
        if len(biggest_faces) == 2:
            rospy.loginfo("BIGGEST FACES IS TWO")
            #We are seeing two faces, we select the biggest and rightmost one if RIGHT and leftmost one if LEFT
            sorted_faces = sorted(biggest_faces, key=lambda k: k['center_x'], reverse=True)
            if self.blackboard_scene.addressee == "left":
                return sorted_faces[1], None
            else:
                return sorted_faces[0], None

    def _is_it_the_same_face(self, face):
        #If the face is in a similar position and has a simila area we return True
        #Otherwise we return False
        #If we didn't yet see the face we return True
        if ((self.last_good_positions[self.left_right_idxs[self.blackboard_scene.addressee]] 
            == self.default_good_position[self.left_right_idxs[self.blackboard_scene.addressee]]) or
            self.last_areas[self.left_right_idxs[self.blackboard_scene.addressee]] == 0):
            return True
        else: 
            #Meaning we already saw the face so both areas and last good position should be set
            if ((abs(face["area"] - self.last_areas[self.left_right_idxs[self.blackboard_scene.addressee]])
                     / self.last_areas[self.left_right_idxs[self.blackboard_scene.addressee]]) < self.accepted_area_similarity):
                return True
            else:
                return False
        

    def terminate(self, new_status):
        new_state = self.service_client_head_movement.get_state()
        print("terminate : ",new_state)
        if new_state == GoalStatus.SUCCEEDED or new_state == GoalStatus.ABORTED or new_state == GoalStatus.LOST:
            self.send_request = True
        if new_state == GoalStatus.PENDING:
            self.send_request = True
            self.logger.debug(f"Cancelling goal to {self.server_name}")
            self.service_client_speaker.cancel_all_goals()
            self.client_result = None
            self.logger.debug(f"Goal cancelled to {self.server_name}")
            #self.service_client_speaker.stop_tracking_goal()
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