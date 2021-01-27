#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.websocket_client import HarmoniWebsocketClient
import harmoni_common_lib.helper_functions as hf
from harmoni_pattern.sequential_pattern import SequentialPattern

# Specific Imports
import rospkg
import json
import os
import inspect
import ast
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import DetectorNameSpace, ActionType, ActuatorNameSpace
from collections import deque
from time import time
import threading
import logging


class LinguisticDecisionManager(HarmoniServiceManager, HarmoniWebsocketClient):
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name, pattern_list, test_id, url, test_input):
        HarmoniServiceManager.__init__(self,name)
        self.name = name
        self.url = url
        self.service_id  = test_id
        if isinstance(test_input, str):
            self.activity_selected = ast.literal_eval(test_input)
        self.index = 0
        self.sequence_scenes = {}
        self.type_web = ""
        self.patient_id =""
        self.session_id = ""
        self.scripted_services = pattern_list
        self.setup_scene()
        self.class_clients={}
        self._setup_classes()
        self.command = None
        self.start_time = None
        self.state = State.INIT
        self.robot_sentence = ''
        self.robot_story = ''
        self.keyWordsStory = ''
        self.askQuestions = []
    

    def _setup_classes(self):
        """
        Set up the pattern classes
        """
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pattern")
        for pattern in self.scripted_services:
            pattern_script_path = pck_path + f"/pattern_scripting/{pattern}.json"
            with open(pattern_script_path, "r") as read_file:
                script = json.load(read_file)
            rospy.loginfo(pattern)
            self.class_clients[pattern] = SequentialPattern(pattern,script)
            self.client_results[pattern] = deque()
        rospy.loginfo("Classes instantiate")
        rospy.loginfo(
            f"{self.name} Decision manager needs these pattern classess: {self.scripted_services}"
        )
        rospy.loginfo("Decision interface classess clients have been set up!")
        return

    def connect_socket(self, patient_id):
        self.message={"action":"OPEN", "patientId": patient_id}
        self.patient_id = int(patient_id)
        rospy.loginfo("Connection to the socket")
        #ip = "192.168.1.83"
        ip = "192.168.1.104"
        #ip = "192.168.122.127" // ip corry pc
        port = 3210
        secure =False
        HarmoniWebsocketClient.__init__(self,ip, port, secure, self.message)

    def open(self, message):
        rospy.loginfo(message)
        if isinstance(message, str):
            message = ast.literal_eval(message)
        if message["patientId"] == self.patient_id:
            rospy.loginfo("Pairing works")
            # set activity to idle
            self.do_request(0,"idle",data="Benvenuto, oggi giocheremo insieme!")
        else:
            rospy.loginfo("Wrong code")
            # rewrite the code
            self.do_request(0,"code", data="Hai sbagliato codice. Riproviamo")
        return

    def store_data(self, correct, item):
        now_time = time()
        int_time = now_time-self.start_time
        rospy.loginfo(f"The time is {int_time}")
        payload = {"action":"FINISHED", "patientId":self.patient_id, "sessionId":self.session_id,"data":{"miniTask":self.index, "correct":correct,"itemSelected": item,"time":int_time}}
        return payload

    def next(self,message):
        rospy.loginfo(message)
        #send finished
        self.stop("multiple_choice")
        self.command = "NEXT"
        self.send(self.store_data(False, ""))
        return

    def replay(self,message):
        rospy.loginfo(message)
        #send finished
        self.stop("multiple_choice")
        self.command = "REPLAY"
        self.send(self.store_data(False, ""))
        return

    def previous(self,message):
        rospy.loginfo(message)
        #send finished
        self.index-=2
        rospy.loginfo(self.index)
        self.command = "PREVIOUS"
        self.send(self.store_data(False, ""))
        self.stop("multiple_choice")
        return

    def terminate(self,message):
        self.command = "TERMINATE"
        rospy.loginfo("terminate")
        self.stop("multiple_choice")
        return

    def pause(self,message):
        self.command = "PAUSE"
        rospy.loginfo("pause")
        self.stop("multiple_choice")
        return

    def resume(self, message):
        self.command = "RESUME"
        rospy.loginfo("resume game")
        rospy.loginfo(message)
        if isinstance(message, str):
            message = ast.literal_eval(message)
        self.activity_selected = message["config"]
        self.session_id = message["sessionId"]
        rospy.loginfo("The activity selected is " + str(self.activity_selected))
        self.setup_scene()
        #self.send({"action":"STARTED","patientId":self.patient_id, "sessionId":self.session_id})
        rospy.sleep(2) ##handle when it finishes
        self.do_request(self.activity_selected["miniTaskId"],"multiple_choice")
        return

    def repeat(self, message):
        self.command = "REPEAT"
        rospy.loginfo("repeat game")
        rospy.loginfo(message)
        self.index = 0
        self.do_request(0,"intro")
        return

    def play_game(self, message):
        rospy.loginfo("play game")
        rospy.loginfo(message)
        if isinstance(message, str):
            message = ast.literal_eval(message)
        self.activity_selected = message["config"]
        self.session_id = message["sessionId"]
        rospy.loginfo("The activity selected is " + str(self.activity_selected))
        self.setup_scene()
        self.send({"action":"STARTED","patientId":self.patient_id, "sessionId":self.session_id})
        self.start_time = time()
        rospy.sleep(2) ##handle when it finishes
        self.index = 0
        self.do_request(0,"intro")
        return

    def start(self, service="code"):
        self.index = 0
        self.state = State.START
        rospy.loginfo("Parte il servizio-------")
        rospy.loginfo(service)
        self.do_request(self.index,service)
        return


    def stop(self, service):
        """Stop the Behavior Pattern """
        try:
            rospy.loginfo("Stop the goal")
            self.class_clients[service].stop("web_page_default")
            self.state = State.SUCCESS
        except Exception as E:
            self.state = State.FAILED
        return

    def do_request(self, index, service, data=None):
        rospy.loginfo("_____START STEP "+str(index)+" DECISION MANAGER FOR SERVICE "+service+"_______")
        self.state = State.REQUEST
        optional_data=None
        self.command=None
        #if self.type_web=="alt":
        #    service = "display_image"
        if service=="multiple_choice":
            if self.type_web=="full":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_full', 'set_content':'"+self.url + self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["third_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["first_img"]+".png'},{'component_id':'distr_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["second_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="choices":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["third_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["first_img"]+".png'},{'component_id':'distr_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["second_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="composed":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'first_img', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["first_img"]+".png'},{'component_id':'second_img', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["second_img"]+".png'},{'component_id':'third_img', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["third_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            elif self.type_web=="alt":
                if self.sequence_scenes["tasks"][index]["text"]=="":
                    service = "display_image"
                else:
                    optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'target_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["third_img"]+".png'},{'component_id':'comp_img_"+self.type_web+"', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["first_img"]+".png'}, {'component_id':'multiple_choice_"+self.type_web+"_container', 'set_content':''}]"}
            else:
                rospy.loginfo("Not existing activity")
                return
        elif service=="display_image":
            if self.type_web=="alt":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
            elif self.type_web == "retelling":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
                if self.index > 0 and self.index <8:
                    self.robot_story += '\n' + self.sequence_scenes["tasks"][index]["text"]
                    rospy.loginfo("Qui c'è la storia detta dal robot")
                    rospy.loginfo(self.robot_story)
            else:
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["main_img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
        elif service=="intro":
            optional_data = {"tts_default": self.sequence_scenes["intro"]["text"], "web_page_default":"[{'component_id':'main_img_alt', 'set_content':'"+self.url +self.sequence_scenes["intro"]["img"]+".png'},{'component_id':'display_image_container', 'set_content':''}]"}
            self.index=0
            service = "display_image"
        elif service=="idle":
            if data:
                optional_data = {"tts_default": data}
            self.index=0
        elif service=="code":
            if data:
                optional_data = {"tts_default": data}
        elif service == "sentence_repetition":
            if self.type_web == "repetition":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"]}
                self.robot_sentence = self.sequence_scenes["tasks"][index]["text"]
                rospy.loginfo("DOVREBBE ESSERE LA FRASE GIUSTA QUESTA QUI -->")
                rospy.loginfo(self.robot_sentence)
            elif self.type_web == "retelling":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"],"web_page_default":"[{'component_id':'main_img_full_1', 'set_content':'"+self.url + self.sequence_scenes["tasks"][index]["img1"]+".png'},{'component_id':'main_img_full_2', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img2"]+".png'},{'component_id':'main_img_full_3', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img3"]+".png'},{'component_id':'main_img_full_4', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img4"]+".png'}, {'component_id':'retelling_container', 'set_content':''}]"}
                rospy.loginfo("Qui siamo in sentence_repetition però con type_web retelling")
            else:
                rospy.loginfo("Qui non dovresti mai arrivaci")
        elif service == "retelling":
            if self.type_web == "retelling":
                optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"],"web_page_default":"[{'component_id':'main_img_full_1', 'set_content':'"+self.url + self.sequence_scenes["tasks"][index]["img1"]+".png'},{'component_id':'main_img_full_2', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img2"]+".png'},{'component_id':'main_img_full_3', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img3"]+".png'},{'component_id':'main_img_full_4', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img4"]+".png'}, {'component_id':'retelling_container', 'set_content':''}]"}
                #Questa è l'optional data che usa multiple choice full 
                #optional_data = {"tts_default": self.sequence_scenes["tasks"][index]["text"], "web_page_default":"[{'component_id':'main_img_full', 'set_content':'"+self.url + self.sequence_scenes["tasks"][index]["img1"]+".png'},{'component_id':'target_img_full', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img2"]+".png'},{'component_id':'comp_img_full', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img3"]+".png'},{'component_id':'distr_img_full', 'set_content':'"+self.url +self.sequence_scenes["tasks"][index]["img4"]+".png'}, {'component_id':'multiple_choice_full_container', 'set_content':''}]"}
            else:
                rospy.loginfo("VEDI CHE IL TYPE WEB NON E UGUALE A retelling")
        if optional_data!="":
            optional_data = str(optional_data)
        def daemon():
            rospy.loginfo("Starting")
            self.class_clients[service].reset_init()
            result_msg = self.class_clients[service].request(optional_data)
            result = {"service":service, "message":result_msg}
            rospy.loginfo("Received result from class")
            self._result_callback(result)
            rospy.loginfo('Exiting')
        d = threading.Thread(target=daemon)
        d.setDaemon(True)
        d.start()
        return


    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        rospy.loginfo("The result of the request has been received")
        rospy.loginfo(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_results[result["service"]].append(
            {"time": time(), "data": result["message"]}
        )
        result_data = []
        if isinstance(result["message"], str) and result["message"]!="":
            result_data = ast.literal_eval(result["message"])
        web_result = []
        if result_data != None or not result_data:
            for data in result_data:
                # if "w" in data:
                #     web_result.append(data["w"]["data"])
                if "s" in data and (data["s"]["data"])!='':
                    web_result.append(data["s"]["data"])
        if web_result:
            for res in web_result:
                rospy.loginfo("OH QUA c'è il RES")
                rospy.loginfo(res)
                if res != '' and res!= None: 
                    rospy.loginfo("res NOT EMPTY")
                    rospy.loginfo(res)
                    if result['service'] == "sentence_repetition":
                        self.senteceRepetition(self.robot_sentence,res)
                    elif result['service'] == "retelling":
                        print("\n\n\n\n\n\n\n\n\n\n\n")
                        print(self.index)
                        print("\n\n\n\n\n\n\n\n\n\n\n")
                        if self.index == 9:
                            rospy.loginfo("CI E' STATA RESTITUITA L'INTERA STORIA E CHIAMIAMO RETELLING")
                            #TODO vedi cosa passare alla funzione perchè dobbiamo cambiarla
                            self.askQuestions = self.retelling(res)
                            #Clear robot story after calling retelling()
                            #N.B. Non serve più self.robot_story TODO eliminalo
                            rospy.loginfo("robot_story è stata pulita")
                            self.robot_story = ''
                        else:
                            rospy.loginfo("CI E' STATA RESTITUITA LA RISPOSTA A UNA DOMANDA E CHIAMIAMO SIMPLE_RETELLING")
                            self.simpleRetelling((self.index - 8), res)
        rospy.loginfo("_____END STEP "+str(self.index)+" DECISION MANAGER_______")
        rospy.loginfo(web_result)
        result_empty = True
        if self.index < (len(self.sequence_scenes["tasks"])-1) and self.index!=-1:
            if self.command =="NEXT" or self.command=="PREVIOUS":
                if result['service']=="multiple_choice":
                    service = "multiple_choice"
                    if self.type_web=="alt":
                        service="display_image"
                    self.index+=1
                    self.do_request(self.index,service)
                    self.state = State.SUCCESS
                    result_empty = False
                elif result['service'] == "display_image":
                    if self.type_web=="alt":
                        rospy.loginfo("Here")
                        service = "multiple_choice"
                        self.index+=1
                        self.do_request(self.index,service)
                    elif self.type_web == "repetition":
                        rospy.loginfo("Sei nel codice che abbiamo scritto noi")
                        service = "sentence_repetition"
                        self.index+=1
                        self.do_request(self.index,service)
                elif result['service'] == "sentence_repetition":
                    if self.type_web == "repetition":
                        rospy.loginfo("Stai ripetendo la seconda frase")
                        service = "sentence_repetition"
                        self.index+=1
                        self.do_request(self.index,service)
            elif self.command=="TERMINATE" or self.command=="PAUSE":
                rospy.loginfo("------------Terminate")
                service="idle"
                self.do_request(self.index,service, data="Abbiamo terminato l'attività. *QT/bye* Ciao ciao, alla prossima!")
            elif self.command=="RESUME":
                rospy.loginfo("RESUME")
            elif self.command=="REPLAY":
                rospy.loginfo("REPLAY")
                self.do_request(self.index,result['service'])
            else:
                if result['service']=="multiple_choice":
                        #res = web_result[1]
                        service = "multiple_choice"
                        for res in web_result:
                            if res=="" or res=="_the_queue_is_empty":
                                result_empty = True
                            else:
                                rospy.loginfo(res)
                                if isinstance(res, str):
                                    res = ast.literal_eval(res)
                                itemselected = res["set_view"].replace(self.url, "")
                                if "arget" in res["set_view"]:
                                    self.index+=1
                                    if (self.type_web=="alt" and self.sequence_scenes["tasks"][self.index]["main_img"]!=""):
                                        service="display_image"
                                    elif self.sequence_scenes["tasks"][self.index]["first_img"]=="": #if choices empty only show main img.
                                        rospy.loginfo("empty choices")
                                        service="display_image"
                                    self.do_request(self.index,service)
                                    self.state = State.SUCCESS
                                    rospy.loginfo("Correct")
                                    self.send(self.store_data(True, itemselected))
                                    result_empty = False
                                elif "omp" in res["set_view"]:
                                    self.do_request(self.index, service)
                                    rospy.loginfo("Wrong")
                                    self.state = State.FAILED
                                    self.send(self.store_data(False, itemselected))
                                    result_empty = False
                                elif "istr" in res["set_view"]:
                                    self.do_request(self.index, service)
                                    rospy.loginfo("Wrong")
                                    self.state = State.FAILED
                                    self.send(self.store_data(False, itemselected))
                                    result_empty = False
                elif result['service'] == "display_image":
                    if self.type_web=="alt":
                        rospy.loginfo("Here")
                        service = "multiple_choice"
                        if self.index==0 and self.sequence_scenes["tasks"][self.index]["main_img"]!="":
                            service = "display_image"
                            self.index+=1
                            self.do_request(0,service)
                        else:
                            self.index+=1
                            self.do_request(self.index,service)
                    #elif self.sequence_scenes["tasks"][self.index]["first_img"]=="":
                    #    service="display_image"
                    #    self.index+=1
                    #    self.do_request(self.index,service)
                    elif self.type_web == "repetition":
                        rospy.loginfo("Here nostra repetition")
                        service = "sentence_repetition"
                        if self.index==0:
                            self.do_request(0,service)
                        else:
                            self.index+=1
                            self.do_request(self.index,service)
                    elif self.type_web == "retelling":
                        rospy.loginfo("Here nostra retelling")
                        rospy.loginfo(self)
                        service = "display_image"
                        if self.index >7:
                            service = "retelling"
                        if self.index == 8:
                            #TODO prenditi le keyword
                            for i in range(2,9):
                                self.keyWordsStory += self.sequence_scenes["tasks"][self.index]["keyword"+str(i)] + "\n"
                            self.keyWordsStory += self.sequence_scenes["tasks"][self.index]["keyword9"]
                            rospy.loginfo("Ecco le keyword della storia")
                            rospy.loginfo(self.keyWordsStory)
                        if self.index==0:
                            self.do_request(0,service)
                            #self.index+=1 # se vuoi skippare la parte in cui quitty parla
                            self.index = 7 # decommenta questo e commenta quello sopra
                        else:
                            self.do_request(self.index,service)
                            self.index+=1
                    else:
                        rospy.loginfo("Here")
                        service = "multiple_choice"
                        self.do_request(self.index,service)
                elif result['service'] == "code":
                    for res in web_result:
                        if res!="":
                            rospy.loginfo("The result is: "+str(res))
                            if isinstance(res, str):
                                res = ast.literal_eval(res)
                                res = res["patient_id"]
                            patient_id = res
                            self.connect_socket(patient_id)
                elif result['service'] == "sentence_repetition":
                    if self.type_web == "repetition":
                        rospy.loginfo("Siamo in result_callback e stiamo caricando il nuovo sentence repetition")
                        service = "sentence_repetition"
                        self.index+=1
                        self.do_request(self.index,service)
                    elif self.type_web == "retelling":
                        service = "sentence_repetition"
                        if self.askQuestions:
                            self.index = 9 + self.askQuestions[0]
                            self.askQuestions.pop(0)
                            self.do_request(self.index,service)
                            print("Sono dentro al bellissimo if che abbiamo fatto con indice" + str(self.index))
                elif result['service'] == "retelling":
                    if self.type_web == "retelling":
                        rospy.loginfo("Siamo in result_callback e abbiamo caricando il nuovo retelling")
                        service = "retelling"
                        if self.index > 8:
                            if self.askQuestions:
                                self.index = 9 + self.askQuestions[0] - 1
                                self.askQuestions.pop(0)
                                print("Sono dentro al bellissimo if che abbiamo fatto con indice " + str(self.index))
                            else:
                                #TODO sei stato bravissimo
                                service = "idle"
                                self.do_request(self.index,service,data="Ottimo lavoro. Sei stato bravissimo!")
                                rospy.loginfo("End of activity")
                                self.index = -1
                        self.do_request(self.index,service)
        else:
            if self.index==len(self.sequence_scenes["tasks"])-1:
                service = "idle"
                self.do_request(self.index,service,data="Ottimo lavoro. Sei stato bravissimo!")
                rospy.loginfo("End of activity")
                self.index = -1
        return

    def senteceRepetition(self, robot, child):
        if robot == child:
            return 1
        resultRobot = []
        resultChild = []
        senteceRobot = robot.split()
        senteceChild = child.split()
        if  len(senteceRobot) != len(senteceChild):
            print("Current senteces have different number of words \n")
            resultRobot.append(robot)
            resultChild.append(child)
        else:
            for i in range(len(senteceRobot)):
                if senteceRobot[i] != senteceChild[i]:
                    # resultRobot.append("\033[93m"+senteceRobot[i]+"\033[0m")
                    resultRobot.append(senteceRobot[i].upper())
                    # resultChild.append("\033[93m"+senteceChild[i]+"\033[0m")
                    resultChild.append(senteceChild[i].upper())
                else:
                    resultRobot.append(senteceRobot[i])
                    resultChild.append(senteceChild[i])

        print(' '.join(resultRobot))
        print(' '.join(resultChild))
        #TODO output json del terapista
        return 0

    def simpleRetelling(self, question, answer):
        questionWithoutAnswer = 0
        lines = self.keyWordsStory.split("\n")
        for line in lines:
            line = line.strip()
            parts = line.split(";")
            if question == int(parts[0]):
                parts.pop(0)
                for key in parts:
                    synonymous = key.split(",")
                    found = 0
                    for syn in synonymous:
                        if found == 1:
                            break
                        if answer.find(syn) != -1:
                            found = 1
                    if found == 0:
                        questionWithoutAnswer = question
                        #f.write("Alla domanda " + str(question) + " il bambino ha risposto:\n" + answer + "\n")
                        break
                    else:
                        print("")
                        #f.write("Alla domanda " + str(question) + " il bambino ha risposto:\n" + answer + "\n")
        #TODO output json del terapista
        print("Questa è una risposta definitiva che il bimbo non ha dato e deve essere aggiunta al json")
        print(questionWithoutAnswer)
        return questionWithoutAnswer

    def retelling(self, child):
        askQuestion = []
        question = -1
        found = 0
        positionsStart = ""
        positionsEnd = ""
        chiediDomande = 1 #chiedo la domanda con 1, non chiedo la domanda con 0
        #keyWords = open("KeywordDataset.txt", "r")
        #lines = keyWords.readlines()
        lines = self.keyWordsStory.split("\n")
        for line in lines:
            line = line.strip()
            parts = line.split(";")
            question = int(parts[0]) # this is the number of the question
            parts.pop(0)
            for key in parts:
                synonymous = key.split(",")
                found = 0
                for syn in synonymous:
                    if child.find(syn) != -1:
                        found = 1
                        ll = list(find_all(child,syn))
                        for l in ll:
                            positionsStart += (str(l)+ ",")
                            positionsEnd += (str(l + len(syn)) + ",")
                        #print("Sto per cancellare la parola " + syn)
                        #child = child.replace(syn, "", 1)
                if found == 0:
                    chiediDomande = 1
                    break
                else:
                    chiediDomande = 0
                    positionsStart += (";")
                    positionsEnd += (";")
            print("Puoi trovare la risposta alla domanda " + line)
            if chiediDomande == 0:
                newChild = checkDistances(child,positionsEnd,positionsStart)
                if newChild == '1':
                    print("Non ho trovato risposta, aggiungero la domanda alla lista di domande da chiedere\n"
                        "non dovrei aver modificato il testo\n")
                    if question == 2:
                        askQuestion.append(1)
                    askQuestion.append(question)
                else:
                    child = str(newChild)
                    print("Il nuovo testo del child ora è:")
                    print(child+"\n")
            else:
                print("Non ho trovato risposta, aggiungero la domanda alla lista di domande da chiedere\n"
                    "non dovrei aver modificato il testo\n")
                if question == 2:
                    askQuestion.append(1)
                askQuestion.append(question)
            positionsEnd = ""
            positionsStart = ""
        print("Questions without answer: ")
        print(askQuestion)
        return askQuestion

    def find_all(self, a_str, sub):
        start = 0
        while True:
            start = a_str.find(sub, start)
            if start == -1: return
            yield start
            start += len(sub) # use start += 1 to find overlapping matches

    def checkDistances(self, childStory, positionsEnd, positionsStart):
        # [3, 23, 30, -1, 10, -1, 25, 37, -1, 16, -1]
        # [0, 19, 25, -1, 5,  -1, 22, 33, -1, 11, -1]
        distance = 21
        positionsStart = positionsStart.strip()
        arrayS = positionsStart.split(";")
        arrayS.pop(len(arrayS)-1)
        #print(arrayS)
        positionsEnd = positionsEnd.strip()
        arrayE = positionsEnd.split(";")
        arrayE.pop(len(arrayE) - 1)
        #print(arrayE)

        dictionaryStart = dict()
        tmpDic = dict()
        dictionaryEnd = dict()
        i = 0
        for keyWordsSetStart in arrayS:
            keyWordsStart = keyWordsSetStart.split(",")
            keyWordsStart.pop(len(keyWordsStart) - 1)
            tmpDic["vector_" + str(i)] = keyWordsStart
            dictionaryStart.update(tmpDic)
            i += 1
        i = 0
        for keyWordsSetEnd in arrayE:
            keyWordsEnd = keyWordsSetEnd.split(",")
            keyWordsEnd.pop(len(keyWordsEnd) - 1)
            tmpDic["vector_" + str(i)] = keyWordsEnd
            dictionaryEnd.update(tmpDic)
            i += 1
        #print(dictionaryStart)
        #print(dictionaryEnd)
        #print(dictionaryStart["vector_0"][0])

        if len(arrayS) == 1:
            print("Trovato! " + dictionaryEnd["vector_0"][0])
            return cleanWordFromTo(childStory, dictionaryStart["vector_0"][0], dictionaryEnd["vector_0"][0])

        if len(arrayS) == 2:
            # Prima ricerca senza shiftata i vettori
            for index1,parola1 in enumerate(dictionaryEnd["vector_0"]):
                for index2,parola2 in enumerate(dictionaryStart["vector_1"]):
                    if abs(int(parola1)-int(parola2)) < distance:
                        print("Trovato! " + parola1 + " - " + parola2)
                        childStory1 = cleanWordFromTo(childStory, dictionaryStart["vector_1"][index2], dictionaryEnd["vector_1"][index2])
                        return cleanWordFromTo(childStory1,dictionaryStart["vector_0"][index1], dictionaryEnd["vector_0"][index1])
            #Seconda ricerca shiftata di 1
            for index1,parola1 in enumerate(dictionaryEnd["vector_1"]):
                for index2,parola2 in enumerate(dictionaryStart["vector_0"]):
                    if abs(int(parola1)-int(parola2)) < distance:
                        print("Trovato! " + parola1 + " - " + parola2)
                        childStory1 = cleanWordFromTo(childStory, dictionaryStart["vector_0"][index2], dictionaryEnd["vector_0"][index2])
                        return cleanWordFromTo(childStory1, dictionaryStart["vector_1"][index1], dictionaryEnd["vector_1"][index1])

        if len(arrayS) == 3:
            #Prima ricerca senza shiftata i vettori
            for index1,parola1 in enumerate(dictionaryEnd["vector_0"]):
                for index2,parola2 in enumerate(dictionaryStart["vector_1"]):
                    if abs(int(parola1) - int(parola2)) < distance:
                        parola3 = dictionaryEnd["vector_1"][index2]
                        for index4,parola4 in enumerate(dictionaryStart["vector_2"]):
                            if abs(int(parola3) - int(parola4)) < distance:
                                print("Trovato! " + parola1 + " - " + parola2 + " - " + parola3 + " - " + parola4)
                                childStory1 = cleanWordFromTo(childStory, dictionaryStart["vector_2"][index4], dictionaryEnd["vector_2"][index4])
                                childStory2 = cleanWordFromTo(childStory1, dictionaryStart["vector_1"][index2], dictionaryEnd["vector_1"][index2])
                                return cleanWordFromTo(childStory2, dictionaryStart["vector_0"][index1], dictionaryEnd["vector_0"][index1])
            #Seconda ricerca shiftata di 1
            for index1,parola1 in enumerate(dictionaryEnd["vector_1"]):
                for index2,parola2 in enumerate(dictionaryStart["vector_2"]):
                    if abs(int(parola1) - int(parola2)) < distance:
                        parola3 = dictionaryEnd["vector_2"][index2]
                        for index4,parola4 in enumerate(dictionaryStart["vector_0"]):
                            if abs(int(parola3) - int(parola4)) < distance:
                                print("Trovato! " + parola1 + " - " + parola2 + " - " + parola3 + " - " + parola4)
                                childStory1 = cleanWordFromTo(childStory, dictionaryStart["vector_0"][index4], dictionaryEnd["vector_0"][index4])
                                childStory2 = cleanWordFromTo(childStory1, dictionaryStart["vector_2"][index2], dictionaryEnd["vector_2"][index2])
                                return cleanWordFromTo(childStory2, dictionaryStart["vector_1"][index1], dictionaryEnd["vector_1"][index1])
            # Terza ricerca shiftata di 2
            for index1,parola1 in enumerate(dictionaryEnd["vector_2"]):
                for index2,parola2 in enumerate(dictionaryStart["vector_0"]):
                    if abs(int(parola1) - int(parola2)) < distance:
                        parola3 = dictionaryEnd["vector_0"][index2]
                        for index4,parola4 in enumerate(dictionaryStart["vector_1"]):
                            if abs(int(parola3) - int(parola4)) < distance:
                                print("Trovato! " + parola1 + " - " + parola2 + " - " + parola3 + " - " + parola4)
                                childStory1 = cleanWordFromTo(childStory, dictionaryStart["vector_1"][index4], dictionaryEnd["vector_1"][index4])
                                childStory2 = cleanWordFromTo(childStory1, dictionaryStart["vector_0"][index2], dictionaryEnd["vector_0"][index2])
                                return cleanWordFromTo(childStory2, dictionaryStart["vector_2"][index1], dictionaryEnd["vector_2"][index1])

        if len(arrayS) == 4:
            # Prima ricerca senza shiftata i vettori
            for index1,parola1 in enumerate(dictionaryEnd["vector_0"]):
                for index2,parola2 in enumerate(dictionaryStart["vector_1"]):
                    if abs(int(parola1) - int(parola2)) < distance:
                        parola3 = dictionaryEnd["vector_1"][index2]
                        for index4,parola4 in enumerate(dictionaryStart["vector_2"]):
                            if abs(int(parola3) - int(parola4)) < distance:
                                parola4 = dictionaryEnd["vector_2"][index4]
                                for index5,parola5 in enumerate(dictionaryEnd["vector_3"]):
                                    if abs(int(parola4) - int(parola5)) < distance:
                                        print("Trovato! " + parola1 + " - " + parola2 + " - " + parola3 + " - " + parola4 + " - " + parola5)
                                        childStory1 = cleanWordFromTo(childStory, dictionaryStart["vector_3"][index5], dictionaryEnd["vector_3"][index5])
                                        childStory2 = cleanWordFromTo(childStory1, dictionaryStart["vector_2"][index4], dictionaryEnd["vector_2"][index4])
                                        childStory3 = cleanWordFromTo(childStory2, dictionaryStart["vector_1"][index2], dictionaryEnd["vector_1"][index2])
                                        return cleanWordFromTo(childStory3, dictionaryStart["vector_0"][index1], dictionaryEnd["vector_0"][index1])
            # Seconda ricerca shiftata di 1
            for index1,parola1 in enumerate(dictionaryEnd["vector_1"]):
                for index2,parola2 in enumerate(dictionaryStart["vector_2"]):
                    if abs(int(parola1) - int(parola2)) < distance:
                        parola3 = dictionaryEnd["vector_2"][index2]
                        for index4,parola4 in enumerate(dictionaryStart["vector_3"]):
                            if abs(int(parola3) - int(parola4)) < distance:
                                parola4 = dictionaryEnd["vector_3"][index4]
                                for index5,parola5 in enumerate(dictionaryEnd["vector_0"]):
                                    if abs(int(parola4) - int(parola5)) < distance:
                                        print("Trovato! " + parola1 + " - " + parola2 + " - " + parola3 + " - " + parola4 + " - " + parola5)
                                        childStory1 = cleanWordFromTo(childStory, dictionaryStart["vector_0"][index5], dictionaryEnd["vector_0"][index5])
                                        childStory2 = cleanWordFromTo(childStory1, dictionaryStart["vector_3"][index4], dictionaryEnd["vector_3"][index4])
                                        childStory3 = cleanWordFromTo(childStory2, dictionaryStart["vector_2"][index2], dictionaryEnd["vector_2"][index2])
                                        return cleanWordFromTo(childStory3, dictionaryStart["vector_1"][index1], dictionaryEnd["vector_1"][index1])
            # Seconda ricerca shiftata di 2
            for index1,parola1 in enumerate(dictionaryEnd["vector_2"]):
                for index2,parola2 in enumerate(dictionaryStart["vector_3"]):
                    if abs(int(parola1) - int(parola2)) < distance:
                        parola3 = dictionaryEnd["vector_3"][index2]
                        for index4,parola4 in enumerate(dictionaryStart["vector_0"]):
                            if abs(int(parola3) - int(parola4)) < distance:
                                parola4 = dictionaryEnd["vector_0"][index4]
                                for index5,parola5 in enumerate(dictionaryEnd["vector_1"]):
                                    if abs(int(parola4) - int(parola5)) < distance:
                                        print("Trovato! " + parola1 + " - " + parola2 + " - " + parola3 + " - " + parola4 + " - " + parola5)
                                        childStory1 = cleanWordFromTo(childStory, dictionaryStart["vector_1"][index5], dictionaryEnd["vector_1"][index5])
                                        childStory2 = cleanWordFromTo(childStory1, dictionaryStart["vector_0"][index4], dictionaryEnd["vector_0"][index4])
                                        childStory3 = cleanWordFromTo(childStory2, dictionaryStart["vector_3"][index2], dictionaryEnd["vector_3"][index2])
                                        return cleanWordFromTo(childStory3, dictionaryStart["vector_2"][index1], dictionaryEnd["vector_2"][index1])
            # Seconda ricerca shiftata di 3
            for index1, parola1 in enumerate(dictionaryEnd["vector_3"]):
                for index2, parola2 in enumerate(dictionaryStart["vector_0"]):
                    if abs(int(parola1) - int(parola2)) < distance:
                        parola3 = dictionaryEnd["vector_0"][index2]
                        for index4,parola4 in enumerate(dictionaryStart["vector_1"]):
                            if abs(int(parola3) - int(parola4)) < distance:
                                parola4 = dictionaryEnd["vector_1"][index4]
                                for index5,parola5 in enumerate(dictionaryEnd["vector_2"]):
                                    if abs(int(parola4) - int(parola5)) < distance:
                                        print("Trovato! " + parola1 + " - " + parola2 + " - " + parola3 + " - " + parola4 + " - " + parola5)
                                        childStory1 = cleanWordFromTo(childStory, dictionaryStart["vector_2"][index5], dictionaryEnd["vector_2"][index5])
                                        childStory2 = cleanWordFromTo(childStory1, dictionaryStart["vector_1"][index4], dictionaryEnd["vector_1"][index4])
                                        childStory3 = cleanWordFromTo(childStory2, dictionaryStart["vector_0"][index2], dictionaryEnd["vector_0"][index2])
                                        return cleanWordFromTo(childStory3, dictionaryStart["vector_3"][index1], dictionaryEnd["vector_3"][index1])
        # return la storia corretta se hai trovato una distanza che ti soddisfa oppure 1 se devi chiedere la domanda
        return "1"

    def cleanWordFromTo(self, text, positionStart, positionEnd):
        i = int(positionStart)
        textList = list(text)
        if int(positionStart) > int(positionEnd):
            print("ERRORE NELLA CHIAMATA DI CLEAN WORD: POSIZIONE START (" + str(positionStart) +") > POSIZIONE END (" + str(positionEnd) + ")")
            return text
        while i < int(positionEnd):
            textList[i] = ""
            i += 1
        text = "".join(textList)
        return text

    def setup_scene(self):
        """Setup the scene """
        activity_episode = ""
        base_dir = os.path.dirname(__file__)
        activity_name = self.activity_selected["test"]
        activity_structure = self.activity_selected["structure"]
        activity_type = self.activity_selected["activityType"]
        if "episode" in self.activity_selected:
            activity_episode = self.activity_selected["episode"]
        with open(
            base_dir + "/resources/multiple_choice_setup.json", "r"
        ) as json_file:
            data = json.load(json_file)
        for typ in data:
            if activity_type in typ:
                for struct in typ[activity_type]:
                    if activity_structure in struct:
                        for nam in struct[activity_structure]:
                            if activity_name in nam:
                                self.type_web = nam[activity_name]["activity_type"]
                                if activity_episode!="":
                                    for ep in nam[activity_name]["content"]:
                                        if activity_episode in ep:
                                            self.sequence_scenes = ep[activity_episode]
                                else:
                                    self.sequence_scenes=nam[activity_name]
        return True

if __name__ == "__main__":
        name = rospy.get_param("/name/")
        test = rospy.get_param("/test_" + name + "/")
        test_input = rospy.get_param("/test_input_" + name + "/")
        test_id = rospy.get_param("/test_id_" + name + "/")
        url = rospy.get_param("/url_" + name + "/")
        pattern_dict = rospy.get_param("/pattern")
        pattern_list = []
        for p in pattern_dict:
            pattern_list.append(p)
        rospy.loginfo(pattern_list)
        try:
            rospy.init_node(name+"_decision")
            bc = LinguisticDecisionManager(name, pattern_list, test_id, url, test_input)
            rospy.loginfo(f"START from the first step of {name} decision.")
            if test:
                bc.start(service="intro")
            else:
                bc.start()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
