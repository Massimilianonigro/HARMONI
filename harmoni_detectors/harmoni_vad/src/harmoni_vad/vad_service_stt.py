#!/usr/bin/env python3

import rospy
from harmoni_common_lib.constants import State, DetectorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from std_msgs.msg import Bool

class VoiceActivityDetector(HarmoniServiceManager):
    def __init__(self, name, param, detector_threshold=0):
        super().__init__(name)
        
        self.state = State.INIT
        self.last_vad_states = []
     

    def start(self, rate=""):
        rospy.loginfo("==== STARTED Streaming VAD")
        self.state = State.START
        # Publishers and Subscribers
        self._vad_sub = rospy.Subscriber(
            "/harmoni/detecting/vad/default/vad_result",
            Bool,
            self.callback,
        )

    def stop(self):
        rospy.loginfo("Voice detector stopped.")
        self.state = State.SUCCESS
        try:
            self._vad_sub.unregister()
        except rospy.ROSInternalException:
            pass



    def request(self, data):
        """Return the current VAD state based on recent history"""
        self.state = State.REQUEST
        self.state = State.SUCCESS
        if self.last_vad_states == []:
            current_vad_state = False
        else:
            current_vad_state = any(self.last_vad_states)
        rospy.loginfo("LAST VAD STATES ARE  " + str(self.last_vad_states))
        rospy.loginfo("CURRENT VAD STATE IS " + str(current_vad_state))
        self.last_vad_states = []
        self.result_msg = str(current_vad_state)
        self.response_received = True
        
        return {"response": self.state, "message": self.result_msg}

    def callback(self, data):
        """Add incoming audio data to buffer"""
        self.last_vad_states.append(data.data)

    
def main():
    service_name = DetectorNameSpace.vad.name
    instance_id = rospy.get_param("instance_id")
    service_id = DetectorNameSpace.vad.value + instance_id

    try:
        rospy.init_node(service_name, log_level=rospy.INFO)
        params = rospy.get_param(f"{DetectorNameSpace.vad.name}/{instance_id}_param/")
        
        detector = VoiceActivityDetector(service_id, params)
        service_server = HarmoniServiceServer(service_id, detector)
        detector.start(params["rate_frame"])  # Start processing immediately
        service_server.start_sending_feedback()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()