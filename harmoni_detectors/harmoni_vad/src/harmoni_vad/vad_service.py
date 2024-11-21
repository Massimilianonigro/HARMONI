#!/usr/bin/env python3

# Common Imports
import rospy

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager

# Specific Imports
from harmoni_common_lib.constants import State, DetectorNameSpace, SensorNameSpace
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData
import numpy as np
import torch
from typing import Generator, List
import wave
from six.moves import queue
from pydub import AudioSegment
from io import BytesIO


class VoiceActivityDetector(HarmoniServiceManager):
    """Voice activity detector
    
    """


    def __init__(self, name, param, detector_threshold=0):
        super().__init__(name)
        self._rate = param["rate_frame"]
        self.subscriber_id = param["subscriber_id"]
        self.sampling_rate = param["sampling_rate"]
        self.n_samples = param["n_samples"]
        self.file_path_pre = param["test_outdir_pre"]
        self.file_path_post = param["test_outdir_post"]

        self.waiting_time = param["waiting_time"]
        #Microphone Parameters
        mic_params = rospy.get_param("microphone/default_param")
        self.original_sampling_rate = mic_params["audio_rate"]
        self.original_chunk_size = mic_params["chunk_size"]
        self.total_channels = mic_params["total_channels"]
        self.original_audio_format_width = mic_params["audio_format_width"]
        self.original_total_channels = mic_params["total_channels"]
        self.detector_threshold = detector_threshold
        self.service_id = name
        self._buff = queue.Queue()
       

        self.vad = False
        self._microphone_source_robot = SensorNameSpace.microphone.value + self.subscriber_id
        print("Expected audio source: ", self._microphone_source_robot)
        self._audio_sub = (
            None  # assign this when start() called. #TODO test subscription during init
        )
        model, utils = torch.hub.load(repo_or_dir = 'snakers4/silero-vad:v4.0',
                                      model = 'silero_vad',
                                      force_reload = True,
                                      onnx = True
                                      )
                                    
        (_,_,_,VADIterator, _) = utils
        self._vad_iterator = VADIterator(model)
        self._vad_pub = rospy.Publisher(
            self.service_id,
            Bool,
            queue_size=1,
        )
        self._first_stream = True
        self.state = State.INIT

    def start(self, rate=""):
        """
        Args:
            rate(int): How often the detector should run per second (Hz).
                Note that this rate should be limited by subscribed camera framerate.
                TODO: actually use this rate. Rate currently matches camera publish rate regardless of this setting
        """
        #rospy.Timer(rospy.Duration(60), baseline_cb)
        rospy.loginfo("==== STARTED")
        self.state = State.START
        self._rate = rate
        self._audio_sub = rospy.Subscriber(
            self._microphone_source_robot, AudioData, self.callback
        )
        

    def stop(self):
        rospy.loginfo("Voice detector stopped.")
        self.state = State.SUCCESS
        try:
            self._audio_sub.unregister()
        except rospy.ROSInternalException:
            pass

    def request(self, data):
        self.state = State.REQUEST
        #Waiting for data
        rospy.sleep(self.waiting_time)
        audio = self.get_audio_from_buffer()
        self.detect_vad(audio)
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.SUCCESS
        self.result_msg = str(self.vad)
        self.response_received = True
        return {"response": self.state, "message": self.result_msg}

    def callback(self, data):
        """ Callback function subscribing to the microphone topic"""
        #rospy.loginfo("Add data to buffer")
        if self.state == State.REQUEST:
            self._buff.put(data.data)
    
    def get_audio_from_buffer(self):
        data = []
        while True:
            try:
                chunk = self._buff.get(block=False)
                if chunk is None:
                    return
                data.append(chunk)
            except queue.Empty:
                break
        return data

    def pause(self):
        self.stop()
    
    def resample_audio(self,audio_bytes, original_rate=48000, target_rate=16000):
        # Create AudioSegment from bytes
        audio = AudioSegment.from_raw(
            BytesIO(audio_bytes), 
            sample_width=self.original_audio_format_width, 
            frame_rate=original_rate, 
            channels=self.total_channels
        )
        
        # Resample
        resampled = audio.set_frame_rate(target_rate)
    
        # Convert to bytes
        return resampled.raw_data

    def detect_vad(self, audio):
        """Uses image to detect and publish face info.
        Args:
            audio(AudioData): the data streaming we want to analyse
        """
        #Resampling data for 16000 instead of original sampling rate (48000)
        #Converting data to byte string
        data = b"".join(audio)
        self._record_audio_data_callback(data, self.file_path_pre, self.original_sampling_rate)
        data = self.resample_audio(data, original_rate=self.original_sampling_rate, target_rate=self.sampling_rate)
        rospy.loginfo("Data RESAMPLED: ")
        data = np.frombuffer(data, np.uint8)
        vad_results = []
        self._record_audio_data_callback(data, self.file_path_post, self.sampling_rate)
        for chunk in self.chunk_audio_fixed_size(data, chunk_size=self.n_samples):
            speech_dict = self._vad_iterator(chunk, return_seconds = True)
            vad_results.append(speech_dict)
        vad_results = [True if result != None else False for result in vad_results]
        rospy.loginfo("VAD results AREEEEEEEE IN ")
        aggregate_vad = lambda bool_list: sum(bool_list) > len(bool_list) / 2
        rospy.loginfo("AGGREGATE VAD IS ")
        self.vad = aggregate_vad(vad_results)
        self._vad_pub.publish(self.vad)
    


    def chunk_audio_fixed_size(self,audio: np.ndarray, chunk_size: int = 512) -> Generator[np.ndarray, None, None]:
        """
        Yields fixed-size chunks from audio array for Silero VAD.
        Last chunk will be zero-padded if needed.
        
        Args:
            audio: Input audio array (1D)
            chunk_size: Size of each chunk in samples (default: 512)
        
        Yields:
            np.ndarray: Chunks of size chunk_size
        """
        # Convert to numpy array if not already
        audio = np.array(audio)
        
        # Process full chunks
        for i in range(0, len(audio), chunk_size):
            chunk = audio[i:i + chunk_size]
            
            # If this is the last chunk and it's incomplete
            if len(chunk) < chunk_size:
                # Create a zero-padded chunk
                padded_chunk = np.zeros(chunk_size)
                padded_chunk[:len(chunk)] = chunk
                yield padded_chunk
            else:
                yield chunk
    
    def _record_audio_data_callback(self, data, filepath, sampling_rate):
        """Callback function to write data"""
        self.wf = wave.open(filepath, "wb")
        self.wf.setnchannels(self.total_channels) 
        #self.wf.setsampwidth(self.p.get_sample_size(self.audio_format))
        self.wf.setsampwidth(self.original_audio_format_width)
        self.wf.setframerate(sampling_rate)
        self.wf.setnframes(self.original_chunk_size)
        self.wf.writeframes(data)


def main():
    service_name = DetectorNameSpace.vad.name  
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = DetectorNameSpace.vad.value + instance_id

    try:
        rospy.init_node(service_name, log_level=rospy.INFO)

        params = rospy.get_param(DetectorNameSpace.vad.name +'/'+ instance_id + "_param/")

        s = VoiceActivityDetector(service_id, params)
        service_server = HarmoniServiceServer(service_id, s)
        #s.start(1)
        service_server.start_sending_feedback()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
