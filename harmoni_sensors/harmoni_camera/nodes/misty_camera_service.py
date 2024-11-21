#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf


# Other Imports
import sys

path = sys.path
using_kinetic = any([True for p in path if ("kinetic" in p)])
if using_kinetic:
    sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
    sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")

import requests
from requests import Timeout
import websocket
import threading
from cv_bridge import CvBridge, CvBridgeError
from harmoni_common_lib.constants import SensorNameSpace
from sensor_msgs.msg import CompressedImage
import cv2
import base64
import numpy as np

class CameraService(HarmoniServiceManager):
    """Reads from a camera and publishes image data.

    As a sensor service, the camera is responsible for reading the image data
    from a physical camera and publishing it so that it can be recorded or
    used by a detector.

    The camera has many parameters which are set in the configuration.yaml

    The public functions exposed by the camera include start(), stop(), and pause()
    """

    def __init__(self, name, param):

        """ Initialization of variables and camera parameters """
        super().__init__(name)
        self.robot_ip = rospy.get_param("robot_ip")
        self.port = param["port"]
        self.width = param["width"]
        self.height = param["height"]
        self.rotation = param["rotation"]
        self.quality = param["quality"]
        self.show = param["show"]
        self.save = param["save"]
        self.outdir = param["outdir"]

        self.is_running = False
        self.is_paused = False
        self.ws = None
        self.ws_thread = None
        self.websocket_url = "ws://"+str(self.robot_ip)+":"+str(self.port)
       

        self.service_id = hf.get_child_id(self.name)

        """ Setup the camera """
        self.start_video_stream()

        """ Init the camera publisher"""
        self.camera_topic = SensorNameSpace.camera.value + self.service_id
        self._video_pub = rospy.Publisher(
            self.camera_topic + "/image_raw/compressed",
            CompressedImage,
            queue_size=1,
        )

        self.state = State.INIT
        return
    
    def start_video_stream(self):
        payload = {'Port': self.port,
                    'Rotation': self.rotation,
                    'Width': self.width,
                    'Height': self.height,
                    'Quality': self.quality
                }
        try:
            response = requests.post('http://{}/api/videostreaming/start'.format(self.robot_ip), 
                                    params = payload,
                                   timeout = 10)
            rospy.loginfo("---------------------------------------------------------\n")
            rospy.loginfo("VIDEOSTREAMING RESPONSE IS IS " + str(response))
            rospy.loginfo("---------------------------------------------------------\n")
            while response.status_code != 200:
                self._stop_stream()
                response = requests.post('http://{}/api/videostreaming/start'.format(self.robot_ip), 
                                    params = payload,
                                   timeout = 10)
                rospy.loginfo("---------------------------------------------------------\n")
                rospy.loginfo("VIDEOSTREAMING RESPONSE IS IS " + str(response))
                rospy.loginfo("---------------------------------------------------------\n")
                rospy.sleep(2)
        except Timeout:
            rospy.logwarn("web_player failed: The ip of the robot appears unreachable")
        

    def start(self):
        """Start the camera stream and publish images"""
        rospy.loginfo("Start the %s service" % self.name)
        if self.state == State.INIT:
            self.state = State.START
            self._read_stream_and_publish()  # Start the camera service at the INIT
            self.state = State.FAILED
        else:
            rospy.loginfo("Trying to start stream when already started")
            self.state = State.START
        return

    def stop(self):
        """Stop the service and close the stream"""
        rospy.loginfo("Stop the %s service" % self.name)
        try:
            self._close_stream()
            self.state = State.SUCCESS
        except Exception:
            self.state = State.FAILED
        return

    def pause(self):
        """Set the service to success to stop publishing"""
        rospy.loginfo("Pause the %s service" % self.name)
        self.state = State.SUCCESS
        return

  

    def _read_stream_and_publish(self):
        """Opening the stream """
        if not self.is_running:
            self.is_running = True
            self.is_paused = False
            self.ws = websocket.WebSocketApp(self.websocket_url,
                                             on_message=self._on_message,
                                             on_error=self._on_error,
                                             on_close=self._on_close)
            self.ws_thread = threading.Thread(target=self.ws.run_forever)
            self.ws_thread.start()
    
    def _on_message(self, ws, message):
        if self.is_paused:
            return
        try:
            ros_img = CompressedImage()
            ros_img.data = message
            ros_img.header.stamp = rospy.Time.now()
            ros_img.format = "jpeg"
            # Publish the image
            self._video_pub.publish(ros_img)
            if self.show:
                # Convert bytes to numpy array
                nparr = np.frombuffer(message, np.uint8)
                # Decode the numpy array as an image
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                cv2.imshow("PcCameraVideo", img)
            if self.save:
                # Convert bytes to numpy array
                nparr = np.frombuffer(message, np.uint8)
                # Decode the numpy array as an image
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                cv2.imwrite(self.outdir + "/misty_test_img.jpg", img)
        except Exception as e:
            rospy.logerr(f"Error processing message: {e}")
    
  

    def _on_error(self, ws, error):
        rospy.logerr(f"WebSocket error in misty camera: {error}")

    def _on_close(self, ws, close_status_code, close_msg):
        rospy.loginfo("WebSocket Misty Camera connection closed")

    def _close_stream(self):
        self._stop_websocket()
        self._stop_stream()
        rospy.loginfo("WebSocket connection and stream closed")

    def _stop_websocket(self):
        if self.is_running:
            self.is_running = False
            self.is_paused = False
            if self.ws:
                self.ws.close()
            if self.ws_thread:
                self.ws_thread.join()

    def _stop_stream(self):
        try:
            response = requests.post('http://{}/api/videostreaming/stop'.format(self.robot_ip), 
                                   timeout = 5)
            rospy.loginfo("---------------------------------------------------------\n")
            rospy.loginfo("CLOSING VIDEOSTREAM RESPONSE IS " + str(response))
            rospy.loginfo("---------------------------------------------------------\n")
        except Timeout:
            rospy.logwarn("web_player failed: The ip of the robot appears unreachable")
        
 
    


def main():
    """Set names, collect params, and give service to server"""

    service_name = SensorNameSpace.camera.name  # "camera"
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"

    try:
        rospy.init_node(service_name, log_level=rospy.INFO)

        # camera/default_param/[all your params]
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = CameraService(service_id, params)

        service_server = HarmoniServiceServer(service_id, s)

        print(service_name)
        print("**********************************************************************************************")
        print(service_id)

        #TODO: comment it out and create a test for ImageAI
        s.start()
        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
