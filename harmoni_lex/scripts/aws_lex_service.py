#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import boto3
from harmoni_common_lib.child import WebServiceServer
from harmoni_common_lib.service_manager import HarmoniExternalServiceManager

class AWSLexService(HarmoniExternalServiceManager):
    """
    Amazon Lex service
    """

    def __init__(self, name, param):
        """ Initialization of variables and lex parameters """
        rospy.loginfo("AWS Lex initializing")
        self.name = name
        self.user_id = param["user_id"]
        self.bot_name = param["bot_name"]
        self.bot_alias = param["bot_alias"]
        self.region_name = param["region_name"]
        """ Setup the lex request """
        self.setup_aws_lex()
        """Setup the lex service as server """
        self.state = self.State.INIT 
        super(AWSLexService, self).__init__(self.state)
        return

    def setup_aws_lex(self):
        self.lex_client = boto3.client('lex-runtime', region_name=self.region_name)
        return

    def response_update(self, response_received, state, result_msg):
        super(AWSLexService, self).update(response_received=response_received, state = state, result_msg=result_msg)
        return

    def test(self):
        super(AWSLexService, self).test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def request(self, input_text):
        rospy.loginfo("Start the %s request" % self.name)
        self.state = self.State.DO_REQUEST
        rate = "" #TODO: TBD
        super(AWSLexService, self).request(rate)
        textdata = input_text
        try:
            lex_response = self.lex_client.post_content(botName = self.bot_name,
														botAlias = self.bot_alias,
														userId = self.user_id,
														contentType = 'text/plain; charset=utf-8',
														accept = 'text/plain; charset=utf-8',
														inputStream = textdata)
            self.state = self.State.COMPLETE_RESPONSE
            self.response_update(response_received=True, state=self.state, result_msg=lex_response["message"])
        except rospy.ServiceException, e:
            self.start = self.State.END
            print "Service call failed: %s" %e
            self.response_update(response_received=True, state=self.state, result_msg="")
        return


def main():
    try:
        service_name = "lex"
        rospy.init_node(service_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        param = rospy.get_param("/"+service_name+"_param/")
        s = AWSLexService(service_name, param)
        web_service_server = WebServiceServer(name=service_name, service_manager=s)
        web_service_server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
