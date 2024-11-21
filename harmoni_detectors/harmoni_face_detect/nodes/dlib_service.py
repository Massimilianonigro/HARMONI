#! /usr/bin/env python3


# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import DetectorNameSpace, SensorNameSpace
from harmoni_common_msgs.msg import Object2D, Object2DArray
from sensor_msgs.msg import Image, CompressedImage
import numpy as np 
import json

import sys

path = sys.path
using_kinetic = any([True for p in path if ("kinetic" in p)])
if using_kinetic:
    sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
    sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2
from cv_bridge import CvBridge, CvBridgeError
import dlib


class DlibFaceDetector(HarmoniServiceManager):
    """Face detector based off of Dlib

    Args:
        detector_threshold(float): Confidence threshold for faces. Positive values
            will return fewer detections, and negative values more detections.
            This value can be changed at any time with no major side-effects.
    """

    # The input image can be upsampled for the detector to see more faces.
    # This is usually not necessary.
    # UPSAMPLING = 0
    # DEFAULT_RATE = 10 # Hz

    def __init__(self, name, params, detector_threshold=0):
        super().__init__(name)
        self._upsampling = params["up_sampling"]
        self.subscriber_id = params["subscriber_id"]
        self.images_compressed = params["compressed"]
        self.show = params["show"]
        self.last_faces = Object2DArray()
        self.state = State.INIT
        self.detector_threshold = detector_threshold
        self.service_id = name
        camera_topic = SensorNameSpace.camera.value + self.subscriber_id + "/image_raw/compressed"
        self._image_source = camera_topic
        self._image_sub = None  # assign this when start() called.
        if not hf.topic_active(camera_topic,CompressedImage):
            rospy.logwarn(
                f"Unable to find topic {camera_topic} with correct type. Is it publishing yet?"
            )
        rospy.loginfo("TOPIC OF FACE DETECTOR IS " + str(self.service_id))
        self._face_pub = rospy.Publisher(
            self.service_id,
            Object2DArray,
            queue_size=1,
        )

        self._hogFaceDetector = dlib.get_frontal_face_detector()
        self._cv_bridge = CvBridge()


    def request(self,data):
        try:
            if self.images_compressed:
                image = rospy.wait_for_message(self._image_source, CompressedImage)
            else:
                image = rospy.wait_for_message(self._image_source, Image)
            self.result_msg = json.dumps(self.detect(image))
            self.state = State.SUCCESS
            self.response_received = True
        except:
            self.state = State.FAILED
            rospy.loginfo("Face Detection failed")
            self.response_received = True
            self.result_msg = ""
        return {"response": self.state, "message": self.result_msg}



    
    def _uncompress_image(self, compressed_image):
        np_arr = np.frombuffer(compressed_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        return image_np


    def detect(self, image):
        """Uses image to detect and publish face info.

        Args:
            image(Image): the image we want to run face detection on.
        """
        if self.images_compressed:
            frame = self._uncompress_image(image)
        else:
            frame = self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
        if frame is not None:
            h, w, _ = frame.shape
            faces_to_publish = []
            faces_to_send = []
            dets, probs, idx = self._hogFaceDetector.run(
                frame, self._upsampling, self.detector_threshold
            )
            for i, d in enumerate(dets):
                rospy.logdebug(f"Detections: {d}")
                center = d.center()
                x1 = d.left()
                y1 = d.top()
                x2 = d.right()
                y2 = d.bottom()

                faces_to_publish.append(
                    Object2D(
                        width=w,
                        height=h,
                        id=idx[i],
                        center_x=center.x,
                        center_y=center.y,
                        topleft_x=x1,
                        topleft_y=y1,
                        botright_x=x2,
                        botright_y=y2,
                        confidence=probs[i],
                    )
                )
                faces_to_send.append({"width": w, "height": h,"id":idx[i], "center_x": center.x, "center_y": center.y, "topleft_x": x1, "topleft_y": y1, "botright_x": x2, "botright_y": y2, "confidence": probs[i]})
                if self.show:
                    frame = self.draw_face_contour(frame, x1, y1, x2, y2)
            self._face_pub.publish(Object2DArray(faces_to_publish))
            if self.show:
                cv2.imshow("Face Detection", frame)
                cv2.waitKey(1)
        return faces_to_send
    
    def draw_face_contour(self, image, topleft_x, topleft_y, bottomright_x, bottomright_y):
 
        # Define the color for the rectangle (BGR format)
        color = (0, 255, 0)  # Green color for the rectangle
        thickness = 2         # Thickness of the rectangle border

        # Draw the rectangle on the image
        cv2.rectangle(image, (topleft_x, topleft_y), (bottomright_x, bottomright_y), color, thickness)

        return image


def main():
    # default doesn't really matter as the launch file overrides it anyway, so string literals are OK
    service_name = DetectorNameSpace.face_detect.name  # "face_detect"
    instance_id = rospy.get_param("instance_id")
    service_id = f"{service_name}_{instance_id}"

    try:
        rospy.init_node(service_name, log_level=rospy.INFO)

        rospy.logdebug(f"Node namespace: {rospy.get_namespace()}")
        params = rospy.get_param(instance_id + "_param/")
        s = DlibFaceDetector(service_id, params)
        service_server = HarmoniServiceServer(service_id, s)
        print(service_name)
        print("**********************************************************************************************")
        print(service_id)
        service_server.start_sending_feedback()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
