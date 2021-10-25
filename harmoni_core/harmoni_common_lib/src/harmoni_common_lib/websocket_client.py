#!/usr/bin/env python3

# Importing the libraries
import rospy
import json
import ast
import socketio
import subprocess


class HarmoniWebsocketClient(object):
    """
    The Harmoni Websocket client class implements a template class for interfacing between
    an external server and the client. 
    Class to send and receive messages to a websocket server.
    """

    def __init__(self,openmessage="Hello"):
        """ The client setup will start the socket connection.
        Args:
            ip (str): url of websocket server
            port (int): port of server
            secure (bool): if the server is https (true) or not (false)
        """
        shell_command = "hostname -I"
        process = subprocess.check_output(shell_command, shell=True)
        process = process.decode("utf-8") 
        ips = process.split(" ")
        for ip in ips:
            if "192" in ip:
                local_url = ip
        self.message = json.dumps(openmessage)
        #websocket.enableTrace(True)
        print("inizio")
        sio = socketio.Client()
        
        @sio.event
        def connect():
            print('connection established')

        @sio.on('start')
        def my_message(data):
            print('message received with ', data)
            self.play_game(data)
            #sio.emit('my response', {'response': 'my response'})
        
        @sio.on('kill')
        def disconnect(data):
            print('disconnected from server')
            self.terminate()
        
        sio.connect('https://misty.I3lab.group', 
            socketio_path= "/wss/socket.io",
            auth= {
                "token": "123",
                "ip" : local_url
                })
        print("connesso")
        sio.wait()
                
    

    def open(self, message):
        """ Make a request of another service, such as a web service
        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("open")
        return

    def play_game(self, message):
        """ Make a request of another service, such as a web service
        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("play")
        return

    def terminate(self):
        """ Make a request of another service, such as a web service
        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("terminate")
        return