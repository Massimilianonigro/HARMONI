#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace
from botocore.exceptions import BotoCoreError, ClientError
from std_msgs.msg import String, Bool, Float32
from contextlib import closing
import soundfile as sf
import numpy as np
import boto3
import re
import json
import ast
import sys


class AWSTtsService(HarmoniServiceManager):
    """
    Amazon tts service
    """

    def __init__(self, name, param):
        """Constructor method: Initialization of variables and polly parameters + setting up"""
        super().__init__(name)
        """ Initialization of variables and tts parameters """
        self.region_name = param["region_name"]
        self.voice = param["voice"]
        #SSML Params
        self.language = param["language"]
        self.max_pitch = param["max_pitch"]
        self.min_pitch = param["min_pitch"]
        self.max_volume = param["max_volume"]
        self.min_volume = param["min_volume"]
        self.max_rate = param["max_rate"]
        self.min_rate = param["min_rate"]
        self.default_pitch = param["default_pitch"]
        self.default_rate = param["default_rate"]
        self.outdir = param["outdir"]
        self.wav_header_length = param["wav_header_length"]
        self.tts_pub = rospy.Publisher(ActuatorNameSpace.tts.value + "default", String, queue_size = 1)
        """ Setup the tts request """
        self._setup_aws_tts()
        """Setup the tts service as server """
        self.state = State.INIT
        return

    def _setup_aws_tts(self):
        """[summary] Setup the tts polly request, connecting to AWS services"""
        self.tts = boto3.client("polly", region_name=self.region_name)
        self.vis_transl = {
            "p": "BILABIAL",
            "f": "LABIODENTAL",
            "T": "INTERDENTAL",
            "s": "DENTAL_ALVEOLAR",
            "t": "DENTAL_ALVEOLAR",
            "S": "POSTALVEOLAR",
            "r": "POSTALVEOLAR",
            "k": "VELAR_GLOTTAL",
            "J": "PALATAL_NASAL",
            "L": "PALATAL_LATERAL_APPROXIMANT",
            "i": "CLOSE_FRONT_VOWEL",
            "u": "CLOSE_BACK_VOWEL",
            "@": "MID_CENTRAL_VOWEL",
            "a": "OPEN_FRONT_VOWEL",
            "e": "OPEN_FRONT_VOWEL",
            "E": "OPEN_FRONT_VOWEL",
            "o": "OPEN_BACK_VOWEL",
            "O": "OPEN_BACK_VOWEL",
            "sil": "IDLE",
        }
        return

    def _split_text(self, text):
        """[summary]
        Split long sentences
        Args:
            text (str): Sentence to be synthetised

        Returns:
            text_array (list): array which containes the part of the text splitted
        """
        if "." in text:
            text_array = text.split(".")
        else:
            text_array = []
            text_array.append(text)
        return text_array

    def _split_behaviors(self, s):
        """[summary]
        Split the text from the behaviors
        Args:
            s (str): input text

        Returns:
            list: list of splitted text and actions
        """
        if len(s) >= 2 and s[-1] == "*" and s[0] == "*":
            return [s]
        else:
            return re.split("\s+", s)

    def _get_text_and_actions(self, sentence):
        """[summary]
        Get text and actions from the sentence
        Args:
            sentence (str): Input text before requesting

        Returns:
            (phrase, actions): Get the text and the action
        """
        tokens = re.split("(\*[^\*\*]*\*)", sentence)
        phrase = "".join(list(filter(lambda s: "*" not in s, tokens)))
        rospy.loginfo("Processing the phrase: %s" % phrase)
        tokens = list(map(lambda s: self._split_behaviors(s), tokens))
        words = []
        for t in tokens:
            words += list(filter(lambda s: len(s) > 0, t))
        actions = []
        i = 0
        for w in words:
            if re.match("\*.*\*", w):
                args = w.strip("*").split()
                name = args.pop(0)
                actions.append([i, name, args])
            else:
                i += 1
        return (phrase, actions)

    def _get_behaviors(self, response, actions):
        """[summary]
        Processing the response from AWS Polly and get the behaviors
        Args:
            response (json): Response from Polly service which contains information about word timing, duration, and visemes
            actions (json): Collects the actions in the text (words into stars **: gesture and facial expressions)

        Returns:
            data (json): It contains all the data including words and actions information
        """

        xSheet = []
        if "AudioStream" in response:
            with closing(response["AudioStream"]) as stream:
                data = stream.read()
                xSheet = data.split(b"\n")
                xSheet = [line.decode("utf-8") for line in xSheet if line != ""]
                xSheet = [json.loads(line) for line in xSheet if line != ""]
        else:
            print("Could not stream audio")
        word_times = list(filter(lambda l: l["type"] == "word", xSheet))
        data = []
        for w in word_times:
            data.append(
                {
                    "character": float(w["start"]) / 1000.0,  # convert ms to seconds
                    "type": "word",
                    "start": float(w["time"]) / 1000.0,
                    "value": str(w["value"]),
                }
            )
        for a in actions:
            if a[0] > len(word_times) - 1:
                a[0] = xSheet[-1]["time"] / 1000.0  # convert ms to seconds
            else:
                a[0] = (word_times[a[0]]["time"]) / 1000.0  # convert ms to seconds
        for a in actions:
            args = a[2]
            if a[1] == "web":
                data.append(
                    {
                        "start": float(a[0])
                        + 0.01,  # prevent visemes and actions from being at exactly the same time
                        "type": "web",
                        "args": args,
                        "id": a[1],
                    }
                )  # End edits
            else:
                data.append(
                    {
                        "start": float(a[0])
                        + 0.01,  # prevent visemes and actions from being at exactly the same time
                        "type": "action",
                        "args": args,
                        "id": a[1],
                    }
                )  # End edits
        visemes = list(
            map(
                lambda l: [l["time"], self.vis_transl[l["value"]]],
                filter(lambda l: l["type"] == "viseme", xSheet),
            )
        )
        for v in visemes:
            data.append(
                {
                    "start": float(v[0]) / 1000.0,  # convert ms to seconds
                    "type": "viseme",
                    "id": v[1],
                }
            )
        return data

    def _get_audio(self, response):
        """[summary]
        This function writes the audio file getting data from Polly
        Args:
            response (obj): response from amazon Polly for getting audio data

        Returns:
            data: audio data
        """
        data = {}
        data["file"] = self.outdir + "/tts.ogg"
        if "AudioStream" in response:
            with closing(response["AudioStream"]) as stream:
                output = data["file"]
                try:
                    with open(output, "wb") as file:
                        file.write(stream.read())
                except IOError as error:
                    print(error)
        else:
            print("Could not stream audio")
        return data

    def _get_response(self, behavior_data):
        """[summary]

        Args:
            behavior_data (json): json containing behavior data (visemes, facial expressions, and gestures) and audio data (audio_frame, and audio_data)

        Returns:
            response (str): it is a object stringified which contained information about
                audio_frame (int)
                audio_data (str): string of audio data array
                behavior_data (str): string of behaviors
        """
        behaviours = list(sorted(behavior_data, key=lambda i: i["start"]))
        data, samplerate = sf.read(self.outdir + "/tts.ogg")
        sf.write(self.outdir + "/tts.mp3", data, samplerate)
        file_handle = self.outdir + "/tts.mp3"
        data = np.fromfile(file_handle, np.uint8)[
            self.wav_header_length :
        ]  # Loading wav file
        data = data.astype(np.uint8).tostring()
        data_array = data
        audio_frame = samplerate
        response = {
            "audio_frame": audio_frame,
            "audio_data": data_array,
            "behavior_data": str(behaviours),
        }
        return str(response)

    def text_to_ssml(
    self,
    text: str,
    language: str = "en-US",   # language code (e.g., "en-US" for English US, "es-ES" for Spanish)
    max_pitch: str = "110%",   # maximum pitch percentage (e.g., "110%")
    min_pitch: str = "95%",    # minimum pitch percentage (e.g., "95%")
    max_volume: str = "+4dB",  # maximum volume (e.g., "+4dB")
    min_volume: str = "medium",# minimum volume level (e.g., "medium" or "+0dB")
    max_rate: str = "105%",    # maximum rate percentage (e.g., "105%")
    min_rate: str = "98%",     # minimum rate percentage (e.g., "98%")
    default_pitch: str = "100%",# baseline pitch (e.g., "100%")
    default_rate: str = "100%" # baseline rate (e.g., "100%")
) -> str:
        # Split the text into sentences based on punctuation for processing
        sentences = re.split(r'(?<=[.!?])\s+', text.strip())
        
        # Initialize SSML string with language
        ssml_text = f'<speak xml:lang="{language}">'

        for sentence in sentences:
            if not sentence:
                continue
            
            # Check the sentence ending
            ending = sentence[-1]
            
            # Set SSML parameters
            pitch = default_pitch  # start with the default pitch
            rate = default_rate    # start with the default rate
            volume_start = max_volume
            volume_end = min_volume
            break_duration = "500ms"  # default pause duration

            # Apply pitch and volume adjustments based on sentence type
            if ending == '?':
                # Raise pitch slightly at the end for questions
                pitch = max_pitch
                # Highlight the last 2-3 words for emphasis
                sentence = re.sub(r'(\b\w+\W*){1,3}$', r'<prosody pitch="{}">\g<0></prosody>'.format(max_pitch), sentence)
                break_duration = "700ms"  # longer pause for questions
            elif ending == '!':
                # Raise pitch and speed slightly for exclamations
                pitch = max_pitch
                rate = max_rate
                sentence = re.sub(r'(\b\w+\W*){1,3}$', r'<prosody pitch="{}">\g<0></prosody>'.format(max_pitch), sentence)
                break_duration = "600ms"  # moderate pause for exclamations
            else:
                # Slightly lower pitch for regular statements
                pitch = min_pitch

            # Wrap each sentence with SSML prosody tags and add a break
            ssml_text += (
                f'<prosody pitch="{pitch}" rate="{rate}" volume="{volume_start}">'
                f"{sentence.strip()}"
                f'</prosody><break time="{break_duration}"/>'
            )
        
        # Close the SSML tag
        ssml_text += "</speak>"
        return ssml_text


    def request(self, input_text):
        """[summary]

        Args:
            input_text (str): Input string to synthetize
        Returns:
            object: It containes information about the response received (bool) and response message (str)
                response: bool
                message: str
        """
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        text = input_text
        [text, actions] = self._get_text_and_actions(text)
        self.tts_pub.publish(text)
        try:
            text = (
               self.text_to_ssml(
                   text,
                    language=self.language,
                    max_pitch=self.max_pitch,
                    min_pitch=self.min_pitch,
                    max_volume=self.max_volume,
                    min_volume=self.min_volume,
                    max_rate=self.max_rate,
                    min_rate=self.min_rate,
                    default_pitch=self.default_pitch,
                    default_rate=self.default_rate
               )    
            )
            json_response = self.tts.synthesize_speech(
                Text=text,
                TextType="ssml",
                OutputFormat="json",
                VoiceId=self.voice,
                SpeechMarkTypes=["viseme", "word"],
                Engine="standard"
            )
            behavior_data = self._get_behaviors(json_response, actions)
            ogg_response = self.tts.synthesize_speech(
                Text=text,
                TextType="ssml",
                OutputFormat="ogg_vorbis",
                VoiceId=self.voice,
                Engine="standard"
            )
            audio_data = self._get_audio(ogg_response)
            tts_response = self._get_response(behavior_data)
            self.state = State.SUCCESS
            self.response_received = True
            self.result_msg = tts_response
            rospy.loginfo("Request successfully completed")
        except (BotoCoreError, ClientError) as error:
            rospy.logerr("The erros is " + str(error))
            self.state = State.FAILED
            self.response_received = True
            self.result_msg = ""
        return {"response": self.state, "message": self.result_msg}

def main():
    """[summary]
    Main function for starting HarmoniPolly service
    """
    service_name = ActuatorNameSpace.tts.name
    instance_id = rospy.get_param("instance_id")
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name)

        param = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = AWSTtsService(service_id, param)

        service_server = HarmoniServiceServer(service_id, s)

        print(service_name)
        print("****************************************************************************")
        print(service_id)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()