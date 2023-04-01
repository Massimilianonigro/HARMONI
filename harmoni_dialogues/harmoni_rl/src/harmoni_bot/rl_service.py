#!/usr/bin/env python3

# Common Imports
import rospy
from dotenv import load_dotenv
from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf
# Specific Imports
from harmoni_common_lib.constants import DialogueNameSpace
import openai
import os

class RLService(HarmoniServiceManager):
    """This is a class representation of a harmoni_dialogue service
    (HarmoniServiceManager). It is essentially an extended combination of the
    :class:`harmoni_common_lib.service_server.HarmoniServiceServer` and :class:`harmoni_common_lib.service_manager.HarmoniServiceManager` classes

    :param name: Name of the current service
    :type name: str
    :param param: input parameters of the configuration.yaml file
    :type param: from yaml
    """

    def __init__(self, name, param):
        """Constructor method: Initialization of variables and lex parameters + setting up"""
        super().__init__(name)
        """ Initialization of variables and lex parameters """
        self.user_id = param["user_id"]
        self.rl_name = param["rl_name"]
        self.rl_alias = param["rl_alias"]
        self.region_name = param["region_name"]
        self.intentName = None
        self.state = State.INIT
        return

    def setup_rl(self):
        """[summary] Setup the lex request, connecting to RL services"""
        env_path="/root/harmoni_catkin_ws/src/HARMONI/.env"
        load_dotenv(env_path)
        rospy.loginfo("Connecting to GPT")
        openai.organization = "org-c177RgKPMbEYIWkVXobIkNcv"
        openai.api_key = os.getenv("OPENAI_API_KEY")
        openai.Model.list()
        rospy.loginfo("Connected")
        return

    def request(self, input_text):
        """[summary]

        Args:
            input_text (str): User request (or input text) for triggering Lex Intent

        Returns:
            object: It containes information about the response received (bool) and response message (str)
                response: bool
                message: str
        """
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        textdata = input_text
        result = {"response": False, "message": None}
        try:
            gpt_response = openai.Completion.create(
            model="text-davinci-003",
            prompt=input_text,
            temperature=0.9,
            max_tokens=150,#150,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0.6,
            stop=[" Human:", " AI:"]
            )
            rospy.loginfo("++++++++++++++++++++++++++++++++++++")
            rospy.loginfo(gpt_response)
            rospy.loginfo(f"The chatgpt response is {gpt_response['choices'][0]['text']}")
            response = gpt_response['choices'][0]['text']
            ai_response = response.split("AI:")[-1]
            self.result_msg = ai_response
            self.response_received = True
            self.state = State.SUCCESS
        except rospy.ServiceException:
            self.state = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_received = True
            self.result_msg = ""
        return {"response": self.state, "message": self.result_msg}

def main():
    """[summary]
    Main function for starting HarmoniChatGPT service
    """
    service_name = DialogueNameSpace.rl.name
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name, log_level=rospy.DEBUG)
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")
        s = ChatGPTService(service_id, params)
        s.setup_chat_gpt()
        service_server = HarmoniServiceServer(service_id, s)
        #s.request("The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n\nHuman: Hello, who are you?\nAI: I am an AI created by OpenAI. How can I help you today?\nHuman: I want to do a positive psychology exercise")
        print(service_name)
        print("**********************************************************************************************")
        print(service_id)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()