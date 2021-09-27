#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
import rospkg
import json
import inspect
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import (
    DetectorNameSpace,
    ActionType,
    ActuatorNameSpace,
)
from sequential_pattern import SequentialPattern
from collections import deque
from time import time
import threading


class MistyDecisionManager(HarmoniServiceManager):
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name, script, instance_id, path, url):
        super().__init__(name)
        self.name = name
        self.script = script
        self.url = url
        self.service_id = instance_id
        self.state = State.INIT

""" component id = {type}_container
    title_id = {type}_tytle
    card_id = [img/txt]_N_{title} !!!! position of N is important!

    types available:    QA (img+txt)
                        QA_text (txt only)
                        touch (img only)
                        touch_text (txt only)
"""        

def build_task(task_type, param_dict):
    
    remote_url = 'https://stream.fs.i3lab.group/misty/images/'
    
    speech = "<prosody rate='slow'>" + param_dict['text'].replace("'", " ") + "</prosody>"
    text =  param_dict['text'].replace("'", " ")
    
    web_trigger = f"[{{'component_id':'{task_type}_container', 'set_content':''}}, {{'component_id':'{task_type}_title', 'set_content':'{text}'}}"

    for i in range(6):
        
        img = ('' if (not (f"img_{i}" in param_dict.keys()) or param_dict[f'img_{i}'] == '') else remote_url + param_dict[f'img_{i}'])
        txt = ('' if (not (f"text_{i}" in param_dict.keys()) or param_dict[f'text_{i}'] == '') else param_dict[f'text_{i}'])

        web_trigger = web_trigger + f", {{'component_id': 'img_{i}_{task_type}', 'set_content':'{img}'}}" 
    
        web_trigger = web_trigger + f", {{'component_id': 'txt_{i}_{task_type}', 'set_content':'{txt}'}}" 
        
    web_trigger = web_trigger + "]"
            
    script = [{"set": "sequence",
                "steps": [
                    {"web_default": 
                         {"action_goal": "DO",
                        "resource_type": "actuator",
                        "wait_for": "new",
                        "trigger":  web_trigger }
                    },
                    {"tts_default": 
                        {"action_goal": "REQUEST",
                        "resource_type": "service",
                        "wait_for": "new",
                        "trigger":speech}
                    },
                    [
                        {"speaker_default": 
                            {"action_goal": "REQUEST",
                            "resource_type": "actuator",
                            "wait_for": "new",
                            "trigger": "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"}
                        },
                        {"face_mouth_default": 
                            {"action_goal": "DO",
                            "resource_type": "actuator",
                            "wait_for": "new"}
                        }
                    ],
                    {"web_default": 
                        {"action_goal": "REQUEST",
                        "resource_type": "actuator",
                        "wait_for": "new",
                        "trigger": web_trigger}
                    } 
                    ]
                }
            ]
    return script


def build_script(type, param_dict):

    if type == "intro":
        speech = "<prosody rate='slow'>" + param_dict['text'].replace("'", " ") + "</prosody>"
        text = param_dict['text'].replace("'", " ")
        img = param_dict['img']
        sound = param_dict['sound']
        script = [{"set": "sequence",
            "steps": [ 
                {"tts_default": 
                    {"action_goal": "REQUEST",
                    "resource_type": "service",
                    "wait_for": "new",
                    "trigger":speech}
                },
                [
                    {"speaker_default": 
                        {"action_goal": "REQUEST",
                        "resource_type": "actuator",
                        "wait_for": "new"}
                    }, 
                    {"face_mouth_default": 
                        {"action_goal": "DO",
                        "resource_type": "actuator",
                        "wait_for": "new"}
                    },
                    {
                    "gesture_default": {
                        "action_goal": "REQUEST",
                        "resource_type": "actuator",
                        "wait_for": "new",
                        "trigger":"{'gesture':'Misty/bye', 'timing': 0.5}"
                    }
                }
                ],
                {"web_default": 
                    {"action_goal": "DO",
                    "resource_type": "actuator",
                    "wait_for": "new",
                    "trigger": f"[{{'component_id':'display_image_container', 'set_content':''}}, {{'component_id':'main_img_alt', 'set_content':'https://stream.fs.i3lab.group/misty/{img}'}}]"}
                }
                ]
            }
        ]
    if type == "task":
        if ('familiarization' in param_dict.keys() and param_dict['familiarization']):
            if ('img_1' in param_dict.keys()):
                script = build_task("touch", param_dict)
            else:
                script = build_task("touch_text", param_dict)
        elif ('img_1' in param_dict.keys() and param_dict['img_1'] != ''):
            script = build_task("QA", param_dict)
        else:
            script = build_task("QA_text", param_dict)
    if type == "reward":
        speech = "<prosody rate='slow'>" + param_dict['text'].replace("'", " ") + "</prosody>"
        text = param_dict['text'].replace("'", " ")
        img = param_dict['img']
        sound = param_dict['sound']
        script = [{"set": "sequence",
            "steps": [ 
                {"tts_default": 
                    {"action_goal": "REQUEST",
                    "resource_type": "service",
                    "wait_for": "new",
                    "trigger":speech}
                },
                [
                    {"speaker_default": 
                        {"action_goal": "REQUEST",
                        "resource_type": "actuator",
                        "wait_for": "new"}
                    },
                    {"face_mouth_default": 
                        {"action_goal": "DO",
                        "resource_type": "actuator",
                        "wait_for": "new"}
                    },
                    {
                    "gesture_default": {
                        "action_goal": "REQUEST",
                        "resource_type": "actuator",
                        "wait_for": "new",
                        "trigger":"{'gesture':'Misty/bye', 'timing': 0.5}"
                    }
                }
                ],
                {"web_default": 
                    {"action_goal": "DO",
                    "resource_type": "actuator",
                    "wait_for": "new",
                    "trigger": f"[{{'component_id':'display_image_container', 'set_content':''}}, {{'component_id':'main_img_alt', 'set_content':'../assets/imgs/{img}'}}]"}
                }
                ]
            }
        ]
    if type == "good":
        speech = "<prosody rate='slow'>" +  param_dict["phrase"] + "</prosody>"
        text = param_dict["phrase"]
        script = [{"set": "sequence",
                "steps": [ 
                    {"tts_default": 
                        {"action_goal": "REQUEST",
                        "resource_type": "service",
                        "wait_for": "new",
                        "trigger":speech}
                    },
                    [
                        {"speaker_default": 
                            {"action_goal": "REQUEST",
                            "resource_type": "actuator",
                            "wait_for": "new"}
                        },
                        {"face_mouth_default": 
                            {"action_goal": "DO",
                            "resource_type": "actuator",
                            "wait_for": "new"}
                        },
                        {
                        "gesture_default": {
                            "action_goal": "REQUEST",
                            "resource_type": "actuator",
                            "wait_for": "new",
                            "trigger":"{'gesture':'Misty/yes', 'timing': 0.5}"
                        }
                    }
                    ]
                ]
            }
        ]
    if type == "bad":
        speech = "<prosody rate='slow'>" +  param_dict["phrase"] + "</prosody>"
        text = param_dict["phrase"]
        script = [{"set": "sequence",
                "steps": [ 
                    {"tts_default": 
                        {"action_goal": "REQUEST",
                        "resource_type": "service",
                        "wait_for": "new",
                        "trigger":speech}
                    },
                    [
                        {"speaker_default": 
                            {"action_goal": "REQUEST",
                            "resource_type": "actuator",
                            "wait_for": "new"}
                        },
                        {"face_mouth_default": 
                            {"action_goal": "DO",
                            "resource_type": "actuator",
                            "wait_for": "new"}
                        }
                    ]
                ]
            }
        ]
    return script

def read_activities(json_path, activity_type, area, level3, level4):
    with open(json_path, "r") as read_file:
        script = json.load(read_file)
    #script = ast.literal_eval(script)
    #print(level4)
    if (level3 != "None"):
        if (level4 != "None"):
            intro = script[activity_type][area][level3][level4]["intro"]
            tasks = script[activity_type][area][level3][level4]["tasks"]
            try:
                reward = script[activity_type][area][level3][level4]["reward"]
            except:
                reward = None
        else : 
            intro = script[activity_type][area][level3]["intro"]
            tasks = script[activity_type][area][level3]["tasks"]
            try:
                reward = script[activity_type][area][level3]["reward"]
            except:
                reward = None
    else : 
        intro = script[activity_type][area]["intro"]
        tasks = script[activity_type][area]["tasks"]
        try:
            reward = script[activity_type][area]["reward"]
        except:
            reward = None

    pattern_to_use = rospy.get_param("pattern_name")
    
    intro_script = build_script("intro", intro)

    s = SequentialPattern(pattern_to_use, intro_script)
    result = s.start()

    for task in tasks:

        task_script = build_script("task", task)
        s.reset_init()
        result = s.request(task_script)
        
        if ("text_right" in task.keys()): #txt is an identifier for question that does not have a right or wrong answer
            if ("text_target" in task.keys()):
                #print(result)
                result = result.replace("txt", "text")
                result = result[0:6]
                if (result == task["text_target"]):
                    phrase = task["text_right"]
                    result_script = build_script("good", {"phrase" : phrase})
                else:
                    phrase = task["text_wrong"]
                    result_script = build_script("bad", {"phrase" : phrase})
            else:
                #print(f"recieved result: {result}")
                result = result.replace("txt", "img")
                result = result[0:5]
                #print("is target in " + task[result])
                
                if ("target" not in task[result]):
                    phrase = task["text_wrong"]
                    result_script = build_script("bad", {"phrase" : phrase})
                else:
                    phrase = task["text_right"]
                    result_script = build_script("good", {"phrase" : phrase})
            
            t = SequentialPattern(pattern_to_use, result_script)
            t.reset_init()
            t.start()
    
    if (reward != None):
        reward_script = build_script("reward", reward)    
        s.reset_init()
        result = s.request(reward_script)
    
    return 


if __name__ == "__main__":
    """Set names, collect params, and give service to server"""

    #call_start = rospy.get_param("start")
    pattern_to_use = rospy.get_param("pattern_name")
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{pattern_to_use}_{instance_id}"
    activity_type = rospy.get_param("activity_type")
    area = rospy.get_param("area")
    level3 = rospy.get_param("level3")
    level4 = rospy.get_param("level4")

    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_pattern")
    pattern_script_path = pck_path + "/activities/config_activity.json"
    #with open(pattern_script_path, "r") as read_file:
    #    script = json.load(read_file)
    
    try:
        rospy.init_node(pattern_to_use, log_level=rospy.INFO)
        bc = MistyDecisionManager(
            None, None, None, pattern_script_path, None
        )
        read_activities(pattern_script_path, activity_type, area, level3, level4)
        
        # multiple_choice/default_param/[all your params]
        #params = rospy.get_param(pattern_to_use + "/" + instance_id + "_param/")
        #service_server = HarmoniServiceServer(service_id, s)
        #service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass