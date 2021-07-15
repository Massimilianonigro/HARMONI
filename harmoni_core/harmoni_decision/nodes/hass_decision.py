#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

import sys
# print(sys.path)
sys.path.insert(0, "/root/harmoni_catkin_ws/src/HARMONI/harmoni_core/harmoni_pattern/nodes/")
# print(sys.path)

from sequential_pattern import SequentialPattern
# from harmoni_pattern.sequential_pattern import SequentialPattern

# Specific Imports
import rospkg
import json
import inspect

from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import DetectorNameSpace, ActionType, ActuatorNameSpace
from collections import deque
from time import sleep, time
import threading

# CHECK if needed
import logging
import ast
import os


class HomeAssistantDecisionManager(HarmoniServiceManager):
    """Instantiates behaviors and receives commands/data for them.
    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name, pattern_list, instance_id , words_file_path, test_input, script,activity_script, path, feeling_pattern_script_path, feeling_script):
        super().__init__(name)
        self.name = name
        self.service_id = instance_id
        self.pattern_script_path = path
        self.feeling_pattern_script_path = feeling_pattern_script_path
        self.script = script
        self.feeling_script = feeling_script
        self.scripted_services = pattern_list
        self.index = 0
        self.config_activity_script = activity_script
        
        self.text_pub = rospy.Publisher(
            "/harmoni/detecting/stt/default", String, queue_size=10
        )

        self.activity_is_on = False
        self.current_quiz = "Arte"
        self.populate_scene(self.index) 
        self.class_clients={}
        self._setup_classes()
        self.quiz_end = 6
        self.last_word = "casa"
        self.words_index = 1
        self.cycles = 1
        self.end = 2
        self.feeling_index = 0
        self.answers = [True, True,True, True, True, True, True]
        self.words = set()
        self.used_words = set()
        self._setup_activities(words_file_path)
        self.state = State.INIT


    def _setup_activities(self, words_file_path):
        # try:
        self.words = set(line.strip() for line in open(words_file_path))
        rospy.loginfo(f"Found {len(self.words)} words")

        # Split all words in syllables
        # for word in self.words:
        #     rospy.loginfo(self._divide(word))


        # except:
        #     rospy.loginfo("Couldn't load txt file")
        return

    def _setup_classes(self):
        """
        Set up the pattern classes from the patterns found in pattern parameter in configuration.yaml
        """
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pattern")

        for pattern in self.scripted_services:
            pattern_script_path = pck_path + f"/pattern_scripting/{pattern}.json"
            with open(pattern_script_path, "r") as read_file:
                script = json.load(read_file)
            rospy.loginfo(pattern)
            self.class_clients[pattern] = SequentialPattern(pattern, script)
            self.client_results[pattern] = deque()

        rospy.loginfo("Classes instantiate")
        rospy.loginfo(
            f"{self.name} Decision manager needs these pattern classes: {self.scripted_services}"
        )
        rospy.loginfo("Decision interface classes clients have been set up!")
        return

    def start(self, service="simple_dialogue"):
        self.index = 0
        self.state = State.START

        # CHECK oven log HOME ASSISTANT every x seconds
        # self.check_log_request()

        # CHECK weather HOME ASSISTANT every x seconds
        # self.weather_request()

        # CHECK wellbeing every x seconds
        # self.wellbeing_request()

        # TODO REMOVE THIS IS JUST TO TEST FEELING 2
        self.feeling_populate(self.feeling_index)
        self.class_clients[service] = SequentialPattern(service, self.feeling_script)

        self.do_request(self.index, service, 'Ciao')

        return



    def wellbeing_request(self):
        
        def daemon():
            while True:
                # Time between home assistant weather check
                sleep(20)
                
                if not self.activity_is_on:

                    self.text_pub.publish("LOG-come-stai")
                    
                    #wait some more time
                    sleep(35)

        d = threading.Thread(target=daemon)
        d.setDaemon(True)
        d.start()
        return


    def weather_request(self):
        
        def daemon():
            while True:
                # Time between home assistant weather check
                sleep(15)
                
                if not self.activity_is_on:
                    rospy.loginfo("Starting home assistant weather check thread")
                    service = "hass"
                    self.class_clients[service].reset_init()

                    optional_data = "{ \"action\":\"check_weather\", \"entity_id\":\"weather.domus\"}"

                    result_msg = self.class_clients[service].request(optional_data)

                    result_msg = ast.literal_eval(result_msg)

                    for item in result_msg:
                        if "h" in item.keys():
                            msg = item["h"]["data"]
                            break
                    else:
                        msg = ""

                    rospy.loginfo("Received result from home assistant "+ msg)

                    self.text_pub.publish(msg)
                    
                    #wait some more time
                    sleep(35)

        d = threading.Thread(target=daemon)
        d.setDaemon(True)
        d.start()
        return



    def check_log_request(self):
        
        def daemon():
            while True:
                # Time between home assistant log checks
                sleep(400)
                
                if not self.activity_is_on:
                    rospy.loginfo("Starting home assistant check log thread")
                    service = "hass"
                    self.class_clients[service].reset_init()

                    optional_data = "{ \"action\":\"check_log\", \"entity_id\":\"switch.oven_power\", \"answer\":\"yes\"}"

                    result_msg = self.class_clients[service].request(optional_data)

                    result_msg = ast.literal_eval(result_msg)

                    for item in result_msg:
                        if "h" in item.keys():
                            msg = item["h"]["data"]
                            break
                    else:
                        msg = ""

                    rospy.loginfo("Received result from home assistant "+ msg)

                    self.text_pub.publish(msg)
                    
                    #wait some more time
                    sleep(35)

        d = threading.Thread(target=daemon)
        d.setDaemon(True)
        d.start()
        return

    def do_request(self, index, service, optional_data=None):
        rospy.loginfo("_____START STEP " + str(index) + " DECISION MANAGER FOR SERVICE " + service + "_______")
        self.state = State.REQUEST

        if optional_data != "":
            optional_data = str(optional_data)

        def daemon():
            rospy.loginfo("Starting")
            self.class_clients[service].reset_init()

            result_msg = self.class_clients[service].request(optional_data)

            result = {"service": service, "message": result_msg}
            rospy.loginfo("Received result from class")

            self._result_callback(result) 

            rospy.loginfo('Exiting')
        d = threading.Thread(target=daemon)
        d.setDaemon(True)
        d.start()
        return

    def _result_callback(self, result):
        """ Receive and store result with timestamp """
        rospy.loginfo("The result of the request has been received")
        rospy.loginfo(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_results[result["service"]].append(
            {"time": time(), "data": result["message"]}
        )
        if isinstance(result["message"], str) and result["message"] != "":
            result_data = ast.literal_eval(result["message"])

        rospy.loginfo(result_data)

        service_message = "ciao ciao"

        if result['service'] == "simple_dialogue":
            
            rospy.loginfo("Index " + str(self.index))

            # Get message from bot
            for item in result_data:
                if "b" in item.keys():
                    msg = item["b"]["data"]
                    break
                else:
                    msg = ""
            
            if "{" in msg:
                rospy.loginfo("{ in msg")
                service = "hass"
            else:
                if msg == "ACTIVITY-1":
                    self.activity_is_on = True
                    service = "catena_di_parole"
                    msg = "Giochiamo alla catena di parole. A turno bisogna dire una parola che comincia con la sillaba finale di quella precedente. La prima parola è: casa."
                elif msg == "ACTIVITY-2":
                    self.activity_is_on = True
                    service = "questions"
                    msg = "Ciao"         
                    self.index =0
                    self.populate_scene(self.index)
                elif msg == "ACTIVITY-3":
                    self.activity_is_on = True
                    service = "feeling_activity"
                    msg = self.get_feeling_question(self.feeling_index)          
                else:
                    service = "simple_dialogue"

            self.do_request(self.index, service, optional_data = msg) 

            self.state = State.SUCCESS

        elif result['service'] == "feeling_activity":

            # prendi da stt
            for item in result_data:
                if "s" in item.keys():
                    if item["s"]["data"] != "":
                        msg = item["s"]["data"]
                        # break

            msg = msg.lower()         

            rospy.loginfo("Word by user: " + msg)

            yes = {"sì", "ok", "va bene"}
            no = {"no"}
            has_answered = False

            service = "feeling_activity"

            for synonym in yes:
                if synonym.lower() in msg:
                    rospy.loginfo("The user said yes")
                    self.answers.append(True)
                    has_answered = True
                    self.feeling_index = self.feeling_index + 1

            if not has_answered:
                for synonym in no:
                    if synonym.lower() in msg:
                        rospy.loginfo("The user said no")
                        self.answers.append(False)
                        self.feeling_index = self.feeling_index + 1
            
            if len(self.answers) == 7:
                service = "feeling_activity_2"
                self.feeling_index = 0

                self.feeling_populate(self.feeling_index - 1)
                self.class_clients[service] = SequentialPattern(service, self.feeling_script)

            else:
                data = self.get_feeling_question(index = self.feeling_index)

            self.do_request(self.index, service, optional_data = data )

        elif result['service'] == "feeling_activity_2":


            for item in result_data:
                if "h" in item.keys():
                    msg = item["h"]["data"]
                    break
                else:
                    msg = ""
           

            self.feeling_index = self.feeling_index + 1

            if self.feeling_index == 7:
                if self.answers:
                    service = "questions"
                    self.activity_is_on = True   
                    self.index = 0
                    self.populate_scene(self.index)
                else:
                    service = "simple_dialogue"
                    self.activity_is_on = False
                
                self.feeling_index = 0

            else:
                service = "feeling_activity_2"
                self.feeling_populate(self.feeling_index, msg)
                self.class_clients[service] = SequentialPattern(service, self.feeling_script)
            
            self.do_request(self.index, service) 

            self.state = State.SUCCESS

        elif result['service'] == "questions":

            # prendi da stt
            for item in result_data:
                if "s" in item.keys():
                    if item["s"]["data"] != "":
                        msg = item["s"]["data"]
                        # break

            msg = msg.lower()         

            rospy.loginfo("Word by user: " + msg)

            if self.current_quiz == "Arte":
                if msg != "stop" and msg != "basta" and msg != "fine" and msg != "no": # TODO contain not ==
                    service = "questions"

                    rospy.loginfo(msg)

                    left = {"sinistra", "prima", "numero uno"}
                    right = {"destra", "seconda", "numero due"}
                    answer = 0

                    if msg == "sì":
                        self.index = 0
                        self.current_quiz = "Geografia"
                        self.populate_scene(self.index, "Ok! Giochiamo di nuovo. ")
                        self.class_clients[service] = SequentialPattern(service, self.script)
                        self.do_request(self.index, service, optional_data = "ciao ciao") 

                    elif msg != "":
                        for synonym in self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][self.index]["text_1"]:
                            if synonym.lower() in msg:
                                    rospy.loginfo("Chosen left answer")
                                    answer = 1
                            else:        
                                for word in left:
                                    if word in msg :

                                        rospy.loginfo(word)
                                        rospy.loginfo(msg)
                                        rospy.loginfo(str(msg in word ))

                                        rospy.loginfo("Chosen left answer")
                                        answer = 1
                                        break
                        
                        if answer != 1:
                            for synonym in self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][self.index]["text_1"]:
                                if synonym.lower() in msg:
                                        rospy.loginfo("Chosen right answer")
                                        answer = 2    
                                else:        
                                    for word in right:
                                        if  word in msg :
                                            rospy.loginfo("Chosen right answer")
                                            answer = 2    

                        if answer == int(self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][self.index]["answer"]):
                            rospy.loginfo("Correct answer")

                            if( self.index == self.quiz_end):
                                self.populate_scene(self.index, congratulations=True)
                            else:
                                self.index = self.index +1 
                                self.populate_scene(self.index)

                            self.class_clients[service] = SequentialPattern(service, self.script)

                        elif answer == 0:
                            rospy.loginfo("No image was selected")

                            self.populate_scene(self.index, "Non hai selezionato alcuna opzione. Riprova. ")                        
                            self.class_clients[service] = SequentialPattern(service, self.script)
                        else:
                            rospy.loginfo("Wrong answer")  

                            self.populate_scene(self.index, "La risposta che hai dato non è corretta. ")   
                            self.class_clients[service] = SequentialPattern(service, self.script)     
                                    
                        self.do_request(self.index, service, optional_data = service_message) 
                else:
                    self.activity_is_on = False
                    self.do_request(self.index, "simple_dialogue", optional_data = "Fine dell'attività.") 
            
            elif self.current_quiz == "Geografia":
                if msg != "stop" and msg != "basta" and msg != "fine" and msg != "no": # TODO contain not ==
                    service = "questions"

                    rospy.loginfo(msg)

                    if msg != "":

                        correct_answer = False
                        for synonym in self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][self.index]["text_1"]:
                            if synonym.lower() in msg:
                                rospy.loginfo("Correct answer")
                                correct_answer = True
                                break

                        if correct_answer == True:
                            if self.index == self.quiz_end:
                                service_message = "Congratulazioni, hai finito tutte le attività."
                                self.current_quiz = "Arte"
                                service = "simple_dialogue"
                            
                            else:
                                self.index = self.index +1 
                                self.populate_scene(self.index)
                                self.class_clients[service] = SequentialPattern(service, self.script)
                                        
                        else:
                            rospy.loginfo("Wrong answer")  

                            self.populate_scene(self.index, "La risposta che hai dato non è corretta. ", suggestion = True)   
                            self.class_clients[service] = SequentialPattern(service, self.script)     

                        self.do_request(self.index, service, optional_data = service_message) 
                else:
                    self.activity_is_on = False
                    self.do_request(self.index, "simple_dialogue", optional_data = "Fine dell'attività.") 
                    self.current_quiz = "Arte"

            self.state = State.SUCCESS


        elif result['service'] == "catena_di_parole":

            # prendi da stt
            for item in result_data:
                if "s" in item.keys():
                        word = item["s"]["data"]
                        break
                else:
                    word = ""
            


            rospy.loginfo("Word by user: " + word)

            # Remove whitespace and keep the first word
            word = word.lstrip()
            sep = ' '
            word = word.split(sep, 1)[0]   
            word = word.lower()         

            rospy.loginfo("Word by user: " + word)

            if word != "stop" and word != "basta" and word != "fine":
                service = "catena_di_parole"

                # controllo index se minore fai questo
                if( self.words_index < self.end):
                                    
                    if word == "passo":
                        rospy.loginfo("User passed")
                        new_word = self._retrieve_word_starting_with_last_syllable(word)
                        self.last_word = new_word
                        msg = "Cambiamo parola: " + new_word

                    elif word !="":
                        if self._check_word_in_dictionary(word):
                            rospy.loginfo("Word is in dictionary")

                            if self._check_word_not_used_yet(word):
                                rospy.loginfo("Word is new")

                                if self._compare_word_syllable(self.last_word, word):
                                    rospy.loginfo("Syllables are the same")

                                    self.used_words.add(word)
                                    msg = self._retrieve_word_starting_with_last_syllable(word)
                                    self.used_words.add(msg)
                                    self.last_word = msg
                                    self.words_index = self.words_index + 1

                                else:
                                    rospy.loginfo("wrong word syllable")
                                    msg = "La parola: " + word + ": non inizia con la sillaba: " + self._get_last_syllable(self.last_word) + ". Riprova"
                            else:
                                rospy.loginfo("word already used")
                                msg = "La parola: " + word + ": è già stata detta. Riprova."
                        else:
                            rospy.loginfo("wrong word not in dict")
                            msg = "La parola: " + word + ": non è presente nel mio dizionario. Riprova."
                    else:
                        msg = self.last_word

                else:   # index is = end, end of cycle
                        if word == "sì":
                            msg = "Va bene! Continuiamo. La parola è " + self.last_word
                            self.cycles = self.cycles + 1
                            self.words_index = 1

                        elif word == "no":
                            msg = "Va bene! Fine dell'attività"
                            self.words_index = 1
                            self.cycles = 1

                        else:
                            msg = "Congratulazioni, hai indovinato " + str(self.end * self.cycles) + " parole. Vuoi giocare ancora?"

                self.do_request(self.index, service, optional_data = msg) 

            else:
                self.activity_is_on = False
                self.words_index = 1
                self.cycles = 1
                self.do_request(self.index, "simple_dialogue", optional_data = "Fine dell'attività.") 

            self.state = State.SUCCESS

        elif result['service'] == "hass":

            # TODO depending on the code received send a different message to reverse dialogue (ok / not ok)
            # TODO manage multiple consecutive hass requests

            service = "simple_dialogue"

            for item in result_data:
                if "h" in item.keys():
                    msg = item["h"]["data"]
                    break
                else:
                    msg = ""
            
            self.do_request(self.index, service, msg)
            self.state = State.SUCCESS

        else:
            rospy.loginfo("Error")
            self.state = State.FAILURE

        rospy.loginfo("_____END STEP " + str(self.index) + " DECISION MANAGER_______")
        return


    def get_feeling_question(self, index):
        return self.config_activity_script[2]["Feeling activity"][0]["Questions"][index]["question"]

    def feeling_populate(self, index, msg = ""):

        if self.answers[index]:
            content = 0
        else:
            content = 1

        x = self.config_activity_script[2]["Feeling activity"][0]["Answers"][index]["command"][content]
        self.feeling_script[0]["steps"][2]["hass_default"]["trigger"] = json.dumps(x) if x else ""

        self.feeling_script[0]["steps"][0]["tts_default"]["trigger"] = msg + self.config_activity_script[2]["Feeling activity"][0]["Answers"][index]["answer"][content]       

        with open(self.feeling_pattern_script_path, "w") as json_file:
            json.dump(self.feeling_script, json_file)

    # --------------- QUIZ


    def populate_scene(self, index_scene, feedback_text = "Scegli la risposta corretta ", congratulations = False,  suggestion = False):

        if self.current_quiz == "Arte":

            intro = "Iniziamo il quiz di cultura generale. " if index_scene == 0 else ""

            if (congratulations == True):
                self.script[1]["steps"][0]["web_default"]["trigger"] = (
                    "[{'component_id':'img_end', 'set_content':'../assets/imgs/" + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["reward"]["img"]
                    + "'}, {'component_id':'title_end', 'set_content':'"
                    + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["reward"]["text"]
                    + "'}, {'component_id':'imgtext_container', 'set_content':''}]"
                )
                self.script[1]["steps"][1]["tts_default"]["trigger"] = self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["reward"]["text"]

            else: # NOT THE NED OF THE GAME
                self.script[1]["steps"][0]["web_default"]["trigger"] = (
                    "[{'component_id':'img_1', 'set_content':'../assets/imgs/" + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["img_1"]
                    + "'}, {'component_id':'img_2', 'set_content':'../assets/imgs/" + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["img_2"]
                    + "'}, {'component_id':'title', 'set_content':'"
                    + feedback_text + "<br>" + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["text"]
                    + "'}, {'component_id':'text_1', 'set_content':'"
                    + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["text_1"][0]
                    + "'}, {'component_id':'text_2', 'set_content':'"
                    + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["text_2"][0]
                    + "'}, {'component_id':'questions_container', 'set_content':''}]"
                )
                self.script[1]["steps"][1]["tts_default"]["trigger"] = intro + feedback_text + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["text"]
        
        if self.current_quiz == "Geografia":
            
            # Add suggestion to webpage if flag is true
            sugg_to_add = self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["sugg"] if suggestion else ""

            self.script[1]["steps"][0]["web_default"]["trigger"] = (
                "[{'component_id':'img', 'set_content':'../assets/imgs/" + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["img_1"]
                + "'}, {'component_id':'title', 'set_content':'"
                + feedback_text + "<br>" + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["text"]
                + "'}, {'component_id':'text', 'set_content':'"
                + sugg_to_add + "'}, {'component_id':'question_container', 'set_content':''}]"
            )
            self.script[1]["steps"][1]["tts_default"]["trigger"] = feedback_text + self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["text"] if not suggestion else self.config_activity_script[0]["Q&A"][0]["General"][0][self.current_quiz]["tasks"][index_scene]["sugg"]
        
        with open(self.pattern_script_path, "w") as json_file:
            json.dump(self.script, json_file)


    # -------------------- SYLLABLES ACTIVITY

    # ORIGINAL CODE IN CBM BASIC 2.0 by Franco Musso, March 1983:
	# http://ready64.it/ccc/pagina.php?ccc=09&pag=036.jpg
	# Adapted from its JAVASCRIPT translation by Francesco Sblendorio, May 2013:
	# http://www.sblendorio.eu/Misc/Sillabe

    def _is_vowel(self, c):
	    return "AEIOUÁÉÍÓÚÀÈÌÒÙ".find(c.upper()) != -1

    def _divide(self, word):
        if word == "":
            return

        rospy.loginfo("Dividing word: " + word)
        a = word.upper()
        result = ""
        s = 0
        while (s < len(a)):
            # rospy.loginfo("s : " + str(s) + " len a: " + str(len(a)))
            if not self._is_vowel(a[s]):
                result += word[s]
                s = s + 1

            elif len(a)-1 != s and not self._is_vowel(a[s+1]):
                if s+2 >= len(a):
                    result += word[s:s+2]+"-"
                    s= s + 2
                elif self._is_vowel(a[s+2]):
                    result += word[s]+"-"
                    s = s + 1
                elif a[s+1] == a[s+2]:
                    result += word[s:s+2]+"-"
                    s= s + 2
                elif "SG".find(a[s+1]) != -1:
                    result += word[s]+"-";
                    s = s + 1
                elif "RLH".find(a[s+2]) != -1:
                    result += word[s]+"-"
                    s = s + 1
                else :
                    result += word[s:s+2]+"-"
                    s= s + 2
                
            elif len(a)-1 != s and ("IÍÌ".find(a[s+1]) != -1) :
                if s>1 and a[s-1:s+1]=="QU" and (len(a) != s+2 and self._is_vowel(a[s+2])):
                    result += word[s:s+2]
                    s= s + 2
                elif len(a) != s+2 and self._is_vowel(a[s+2]) :
                    result += word[s]+"-"
                    s = s + 1
                else :
                    result += word[s]
                    s = s + 1
                
            elif "IÍÌUÚÙ".find(a[s])!=-1 :
                result += word[s]
                s = s + 1
            else :
                result += word[s]+"-"
                s = s + 1
            
        if result[len(result)-1] == "-":
            result = result[0:len(result)-1]
        
        return result

    def _check_word_in_dictionary(self, input_word):
        return input_word in self.words

    def _check_word_not_used_yet(self, input_word):
        return not input_word in self.used_words

    def _compare_word_syllable(self, input_word, word_found):
        return self._get_first_syllable(word_found) == self._get_last_syllable(input_word)  

    def _get_last_syllable(self, input_word):
        input_word_split = self._divide(input_word)
        rospy.loginfo("Input word in syllables: "+ input_word_split)
        input_word_syllables = (input_word_split.split('-'))
        rospy.loginfo("Last syllable : "+ input_word_syllables[-1] )
        return input_word_syllables[-1].strip()        

    def _get_first_syllable(self, word_found):
        word_found_split = self._divide(word_found)
        rospy.loginfo("Word divided in syllables: "+ word_found_split)
        syllables = (word_found_split.split('-'))
        rospy.loginfo("Syllable : "+ syllables[0] )
        return syllables[0].strip()     



    def _retrieve_word_starting_with_last_syllable(self, input_word):
        """ Retrieves a word from the dictionary that starts with the same syllable as the final syllable of the input word """

        found = False
        last_syllable = self._get_last_syllable(input_word)
        list = [x for x in self.words.difference(self.used_words) if x.startswith(last_syllable)]
        list_iterator = iter(list)

        while not found:
            word_found = next(list_iterator,"")
            if word_found == "":
                rospy.loginfo("Word not found")
                break
            rospy.loginfo("Word with selected prefix: "+ word_found)
            found = self._compare_word_syllable(input_word, word_found)
            rospy.loginfo("Syllable equal to prefix?: "+ str(found))

        return word_found

    # --------------------


if __name__ == "__main__":
    name = rospy.get_param("/pattern_name/")
    # test = rospy.get_param("/test_" + name + "/")
    # test_input = rospy.get_param("/test_input_" + name + "/")
    test_input = "Input"
    instance_id = rospy.get_param("/instance_id_" + name + "/")
    
    
    pattern_list = []

    # pattern_list.append("simple_dialogue")
    pattern_list.append("simple_dialogue")
    pattern_list.append("hass")
    pattern_list.append("catena_di_parole")
    pattern_list.append("questions")
    pattern_list.append("feeling_activity")  
    pattern_list.append("feeling_activity_2")        

    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_decision")
    words_file_path = pck_path + f"/dict/words.txt"

    pck_path = rospack.get_path("harmoni_decision")
    words_file_path = pck_path + f"/dict/words.txt"
    config_activity_path = pck_path + f"/resources/config_activity.json"
    with open(config_activity_path, "r") as read_file:
        activity_script = json.load(read_file)

    pck_path = rospack.get_path("harmoni_pattern")
    pattern_script_path = pck_path + f"/pattern_scripting/questions.json"
    with open(pattern_script_path, "r") as read_file:
        script = json.load(read_file)
    
    feeling_pattern_script_path = pck_path + f"/pattern_scripting/feeling_activity_2.json"
    with open(feeling_pattern_script_path, "r") as read_file:
        feeling_script = json.load(read_file)

    try:
        rospy.init_node(name + "_decision")
        bc = HomeAssistantDecisionManager(name, pattern_list, instance_id, words_file_path, test_input, script, activity_script, pattern_script_path, feeling_pattern_script_path, feeling_script)
        rospy.loginfo(f"START from the first step of {name} decision.")

        bc.start(service="feeling_activity_2")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
