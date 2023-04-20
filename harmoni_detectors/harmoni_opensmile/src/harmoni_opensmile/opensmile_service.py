#!/usr/bin/env python3

# Common Imports
import rospy
import roslib
from datetime import datetime

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import State, DetectorNameSpace, SensorNameSpace
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData
import numpy as np
import os
import io
import ast
import pyaudio
import wave
import opensmile

class OpenSmileDetector(HarmoniServiceManager):
    """Face expression recognition detector based off of FaceChannels
    Args:
        detector_threshold(float): Confidence threshold for faces. Positive values
            will return fewer detections, and negative values more detections.
            This value can be changed at any time with no major side-effects.
    """


    def __init__(self, name, param):
        super().__init__(name)
        self._rate = param["rate_frame"]
        self.subscriber_id = param["subscriber_id"]
        self.out_dir = param["output_dir"]
        self.service_id = name
        print(
            "Expected detected destination: ",
            DetectorNameSpace.opensmile.value + self.subscriber_id,
        )
        self._opensmile_pub = rospy.Publisher(
            self.service_id,
            Bool,
            queue_size=1,
        )
        self._mic_topic = SensorNameSpace.microphone.value + self.subscriber_id 
        self.first_frame = True
        self.state = State.INIT
        self.filename = "audio"
        self.done = False
        self.counter = 0


    def start(self, rate=""):
        """
        Args:
            rate(int): How often the detector should run per second (Hz).
                Note that this rate should be limited by subscribed camera framerate.
                TODO: actually use this rate. Rate currently matches camera publish rate regardless of this setting
        """
        rospy.loginfo("==== STARTED")
        self.state = State.START
        self._rate = rate
        self._microphone_sub = rospy.Subscriber(
            self._mic_topic, AudioData, self.detect_callback
        )

        

    def stop(self):
        rospy.loginfo("Face detector stopped.")
        self.state = State.SUCCESS
        try:
            self._image_sub.unregister()
        except rospy.ROSInternalException:
            pass

    def pause(self):
        self.stop()

    def _save_audio(self, data):
        data = np.frombuffer(data.data, np.uint8)
        if self.first_frame:
            if self.counter == 10:
                for i in range(0, 10):
                    if os.path.exists(self.out_dir + 'audio' + str(i) +'.wav'):
                        os.remove(self.out_dir + 'audio' + str(i) +'.wav')
                    else:
                        print("The file does not exist")
                self.counter = 1
            else:
                self.counter +=1
            filename = 'audio'+str(self.counter)#datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            p = pyaudio.PyAudio()
            #file_name = "audio_" + file_name_date
            self.path_audio = self.out_dir + filename + ".wav"
            self.wf = wave.open(self.path_audio, "wb")
            self.wf.setnchannels(1)
            self.wf.setsampwidth(
                p.get_sample_size(pyaudio.paInt16)
            )
            self.wf.setframerate(16000)
            self.wf.setnframes(1600)
            self.wf.writeframes(b"".join(data))
        else:
            self.wf.writeframes(b"".join(data))
        return self.path_audio

    def _opensmile_process(self, filename):
        done = False
        rospy.loginfo("++++ OPENSMILE PROCESSING ")
        sampling_rate = 16000
        smile = opensmile.Smile(
            feature_set = opensmile.FeatureSet.eGeMAPSv02,
            feature_level = 'lld',
            options = {'frameMode': 'fixed', 'frameSize': 0.01, 'frameStep': 1},
            verbose = True,
        )
        feature_vector = smile.process_files([filename])
        filename = filename.split(".")[0]
        feature_vector.to_csv(filename + '.csv')
        done = True
        
        
        return done

    def detect_callback(self, data):
        """Uses audio to detect and publish that the opensmile has processed the audio file info.
        Args:
            data(data): String from the opensmile
        """
        self._save_audio(data)
        if self.first_frame:
            self.filename = self.path_audio
            self.first_frame = False
            def baseline_cb(event):
                self.done = self._opensmile_process(self.filename)
                filename = self.path_audio.split(".")[0]
                if self.done: 
                    self._opensmile_pub.publish(self.done)
                    self.first_frame = True
                    self.done = False
            rospy.Timer(rospy.Duration(10), baseline_cb)
            
            

def main():

    service_name = DetectorNameSpace.opensmile.name  
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = DetectorNameSpace.opensmile.value + instance_id

    try:
        rospy.init_node(service_name, log_level=rospy.INFO)

        params = rospy.get_param(DetectorNameSpace.opensmile.name +'/'+ instance_id + "_param/")

        s = OpenSmileDetector(service_id, params)
        service_server = HarmoniServiceServer(service_id, s)
        s.start(1)
        service_server.start_sending_feedback()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
