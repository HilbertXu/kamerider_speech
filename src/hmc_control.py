#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 2019/04/05
    Author: Xu Yucheng
    Abstract: Code for help-me-carry project
"""
import roslib
'''
roslib.load_manifest ('speech')
roslib.load_manifest () reads the package manifest and sets up the python library path based on the package dependencies. 
It's required for older rosbuild-based packages, but is no longer needed on catki
'''
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import os
import sys
import time
import wave
import datetime
import pyaudio
from kamerider_speech.msg import mission
from sound_play.libsoundplay import SoundClient
from read_xml_files import main as read_main
from play_signal_sound import play_signal_sound
from turtlebot_msgs.srv import SetFollowState

class help_me_carry(object):
    """
        class for help me carry
    """
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.voice = rospy.get_param("~voice", "voice_cmu_us_bdl_arctic_clunits")
        self.question_start_signal = rospy.get_param("~question_start_signal", "/home/nvidia/catkin_ws/src/kamerider_speech/sounds/question_start_signal.wav")

        self.pub_message_topic_name     = None
        self.sub_pocket_back_topic_name = None
        self.sub_xfei_back_topic_name   = None

        # Mission keywords
        self.target_room = None
        self.target_location = None
        self.target_person = None
        self.target_object = None
        self.target_mission = None

        # Initialize sound client
        self.sh = SoundClient(blocking=True)
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)

        # Initialize SetFollowerState client
        self.set_state = rospy.ServiceProxy("/turtlebot_follower/change_state", SetFollowState)
        self.get_params()

    def get_params(self):
        self.pub_message_topic_name = rospy.get_param("pub_message_topic_name", "/hmc_control")
        self.sub_pocket_back_topic_name = rospy.get_param("sub_pocket_back_topic_name", "/lm_data")
        self.sub_xfei_back_topic_name = rospy.get_param("sub_xfei_back_topic_name", "/xfei_output")

        pub = rospy.Publisher(self.pub_message_topic_name, String, queue_size=1)
        rospy.Subscriber(self.sub_pocket_back_topic_name, String, self.pocketCallback)
        rospy.Subscriber(self.sub_pocket_back_topic_name, String, self.xfeiCallback)
    
    def pocketCallback(self, msg):
        print (msg.data)
        if msg.data.lower() == "jack follow":
            print ("Start follow-me")
            self.sh.say("Now i will start following you", self.voice)
            self.sh.say("Please slow down if i cannot follow you", self.voice)
            state_msg = True
            response = self.set_state(state_msg)
        
        if msg.data.lower() == "jack stop":
            print ("Stop follow-me")
            state_msg = False
            response = self.set_state(state_msg)
            self.sh.say("Ok i will stop following you right now", self.voice)
            self.sh.say("I have already remember the current location", self.voice)
            self.sh.say("Please hang the luggage on my arm", self.voice)
            """
                @TODO
                此处对机械臂发消息，使机械臂向前伸出
            """
            rospy.set_param("in_position", "true")
    
    def xfeiCallback(self, msg):
        string = msg.data
        symbols = ["!", "?", ".", ",", ";", ":"]
        output = []
        if string[-1] in symbols:
            string = string[:-1]
        for part in string.lstrip().split(","):
            for word in part.split():
                output.append(word)
	    output = [item.lower() for item in output]
	    print (output)

    def cleanup(self):
        self.sh.say("S H I T", self.voice)

if __name__ == '__main__':
    rospy.init_node("hmc_control", anonymous=True)
    hmc = help_me_carry()
    rospy.spin()
