#!/usr/bin/env python3

import rospy
from harmoni_common_lib.constants import State, DetectorNameSpace, SensorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData
import numpy as np
import torch
from collections import deque
from typing import Generator, List
import wave
from six.moves import queue
from pydub import AudioSegment
from io import BytesIO

class VoiceActivityDetector(HarmoniServiceManager):
    def __init__(self, name, param, detector_threshold=0):
        super().__init__(name)
        self._rate = param["rate_frame"]
        self.subscriber_id = param["subscriber_id"]
        self.sampling_rate = param["sampling_rate"]
        self.n_samples = param["n_samples"]
        self.file_path_pre = param["test_outdir_pre"]
        self.file_path_post = param["test_outdir_post"]
        self.window_size = param.get("window_size", 3)  # Number of chunks to consider for voting
        
        # Microphone Parameters
        mic_params = rospy.get_param("microphone/default_param")
        self.original_sampling_rate = mic_params["audio_rate"]
        self.original_chunk_size = mic_params["chunk_size"]
        self.total_channels = mic_params["total_channels"]
        self.original_audio_format_width = mic_params["audio_format_width"]
        
        # Initialize VAD components
        self.detector_threshold = detector_threshold
        self.service_id = name
        self._buff = queue.Queue()
        self._processed_chunks = deque(maxlen=self.window_size)  # Store recent VAD results
        
        # Initialize Silero VAD
        model, utils = torch.hub.load(
            repo_or_dir='snakers4/silero-vad:v4.0',
            model='silero_vad',
            force_reload=True,
            onnx=True
        )
        (_, _, _, VADIterator, _) = utils
        self._vad_iterator = VADIterator(model)
        
        # Publishers and Subscribers
        self._vad_pub = rospy.Publisher(
            self.service_id,
            Bool,
            queue_size=1,
        )
        self._audio_sub = None
        self.state = State.INIT
        
        self._microphone_source_robot = SensorNameSpace.microphone.value + self.subscriber_id
        print("Expected audio source: ", self._microphone_source_robot)
        
        # Processing flags and state
        self.is_processing = False
        self.current_vad_state = False
        
        # Start processing timer
        self.process_timer = None

    def start(self, rate=""):
        rospy.loginfo("==== STARTED Streaming VAD")
        self.state = State.START
        self._rate = rate
        self._audio_sub = rospy.Subscriber(
            self._microphone_source_robot, 
            AudioData, 
            self.callback
        )
        # Start continuous processing
        self.is_processing = True
        self.process_timer = rospy.Timer(
            rospy.Duration(1.0 / float(self._rate)),
            self.process_audio
        )

    def stop(self):
        rospy.loginfo("Voice detector stopped.")
        self.state = State.SUCCESS
        self.is_processing = False
        if self.process_timer:
            self.process_timer.shutdown()
        try:
            self._audio_sub.unregister()
        except rospy.ROSInternalException:
            pass

    def request(self, data):
        """Return the current VAD state based on recent history"""
        self.state = State.REQUEST
        rospy.loginfo("CURRENT VAD STATE IS " + str(self.current_vad_state))
        self.state = State.SUCCESS
        self.result_msg = str(self.current_vad_state)
        self.response_received = True
        return {"response": self.state, "message": self.result_msg}

    def callback(self, data):
        """Add incoming audio data to buffer"""
        if self.is_processing:
            self._buff.put(data.data)

    def process_audio(self, event=None):
        """Process audio chunks continuously"""
        if not self.is_processing:
            return

        audio_data = self.get_audio_from_buffer()
        if not audio_data:
            return

        # Process the audio
        data = b"".join(audio_data)
        # Resample the audio
        resampled_data = self.resample_audio(
            data,
            original_rate=self.original_sampling_rate,
            target_rate=self.sampling_rate
        )
        audio_array = np.frombuffer(resampled_data, np.uint8)

        # Process chunks and update VAD state
        vad_results = []
        for chunk in self.chunk_audio_fixed_size(audio_array, chunk_size=self.n_samples):
            speech_dict = self._vad_iterator(chunk, return_seconds=True)
            vad_results.append(False if speech_dict == None else True)

        # Update processed chunks history
        if vad_results:
            #TODO CHECK IF THIS IS CORRECT
            self.current_vad_state = any(vad_results)  # True if any chunk contains speech            
            # Update current state and publish
            self._vad_pub.publish(self.current_vad_state)

    def get_audio_from_buffer(self):
        """Get accumulated audio data from buffer"""
        data = []
        while True:
            try:
                chunk = self._buff.get(block=False)
                if chunk is None:
                    return None
                data.append(chunk)
            except queue.Empty:
                break
        return data if data else None

    def resample_audio(self, audio_bytes, original_rate=48000, target_rate=16000):
        """Resample audio to target rate"""
        audio = AudioSegment.from_raw(
            BytesIO(audio_bytes),
            sample_width=self.original_audio_format_width,
            frame_rate=original_rate,
            channels=self.total_channels
        )
        resampled = audio.set_frame_rate(target_rate)
        return resampled.raw_data

    def chunk_audio_fixed_size(self, audio: np.ndarray, chunk_size: int = 512) -> Generator[np.ndarray, None, None]:
        """Chunk audio into fixed-size pieces"""
        audio = np.array(audio)
        for i in range(0, len(audio), chunk_size):
            chunk = audio[i:i + chunk_size]
            if len(chunk) < chunk_size:
                padded_chunk = np.zeros(chunk_size)
                padded_chunk[:len(chunk)] = chunk
                yield padded_chunk
            else:
                yield chunk

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