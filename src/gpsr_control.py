#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2018/03/02
Author: Xu Yucheng 
Abstract： 订阅输出Pocketsphinx识别结果的话题，然后根据输出的结果来控制机器人
'''

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

# 定义表示任务状态的变量
UNSTART = 0
PROCESS = 1
FINISH  = 2

class gpsr_speech_control(object):
    """Class to read the recognition output of pocketsphinx"""
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # Default infomations
        self.gestures, self.locations, self.names, self.objects = read_main()
        # Type of task
        self.task_type = ['person', 'object', 'object2person', 'Q&A']
        # State of task
        self.task_state = [UNSTART, PROCESS, FINISH]
        # Predefined missions
        self.missions = ['put', 'bring', 'take', 'guide', 'find', 'answer', 'introduce',
                         'grasp', 'get', 'give', 'tell', 'navigate', 'look', 'deliver']
        # Define parameters
        self.voice = None
        self.question_start_signal = None
        self.cmd_files = None
        # Publisher topics
        self.pub_to_nav_topic_name = None
        self.pub_to_image_topic_name = None
        self.pub_to_arm_topic_name = None
        # Subscriber topics
        self.sub_nav_back_topic_name = None
        self.sub_image_back_topic_name = None
        self.sub_arm_back_topic_name = None
        # Mission keywords
        self.target_room = None
        self.target_location = None
        self.target_person = None
        self.target_object = None
        self.target_mission = None
        # Mission keywords
        self._room = UNSTART
        self._location = UNSTART
        self._person = UNSTART
        self._object = UNSTART
        self._mission = UNSTART
        
        self.init_params()
        self.get_params()
    
    def init_params(self):
        # Mission keywords
        self.target_room = None
        self.target_location = None
        self.target_person = None
        self.target_object = None
        self.target_mission = None
        # Mission state
        self._room = UNSTART
        self._location = UNSTART
        self._person = UNSTART
        self._object = UNSTART
        self._mission = UNSTART

    def get_params(self):
        # Initialize sound client
        self.sh = SoundClient()
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)

        # Get parameters
        self.voice = rospy.get_param("~voice", "voice_us2_mbrola")
        self.cmd_files = rospy.get_param("~cmd_file", "/home/kamerider/catkin_ws/src/kamerider_speech/CommonFiles")

        self.pub_to_arm_topic_name   = rospy.get_param("pub_to_arm_topic_name"  , "/speech_to_arm")
        self.pub_to_nav_topic_name   = rospy.get_param("pub_to_nav_topic_name"  , "/speech_to_nav")
        self.pub_to_image_topic_name = rospy.get_param("pub_to_image_topic_name", "/speech_to_image")

        self.sub_arm_back_topic_name   = rospy.get_param("sub_arm_back_topic_name"  , "/arm_to_speech")
        self.sub_nav_back_topic_name   = rospy.get_param("sub_nav_back_topic_name"  , "/nav_to_speech")
        self.sub_image_back_topic_name = rospy.get_param("sub_image_back_topic_name", "/image_to_speech")

        rospy.Subscriber(self.sub_arm_back_topic_name, String, self.armCallback)
        rospy.Subscriber(self.sub_nav_back_topic_name, String, self.navCallback)
        rospy.Subscriber(self.sub_image_back_topic_name, String, self.imageCallback)

        self.arm_pub   = rospy.Publisher(self.pub_to_arm_topic_name, String, queue_size=1)
        self.nav_pub   = rospy.Publisher(self.pub_to_nav_topic_name, String, queue_size=1)
        self.image_pub = rospy.Publisher(self.pub_to_image_topic_name, String, queue_size=1)

        # Start gpsr task
        self.start_gpsr()
    
    def publishr_message(self, pub, msg):
        # 接受一个发布器和待发布的消息
        pub.publish(msg)
    
    def start_gpsr(self):
        # 播放初始化的音频，并提醒操作者如何使用语音操作Jack
        self.sh.say("Hello my name is Jack", self.voice)
        rospy.sleep(3)
        self.sh.say("Please say Jack to wake me up before each question", self.voice)
        rospy.sleep(4)
        self.sh.say("I am ready for your command if you hear", self.voice)
        rospy.sleep(3.5)
        play_signal_sound()
    
    def armCallback(self, msg):
        if msg.data == "object_target_grasped":
            self._object = FINISH
    
    def navCallback(self, msg):
        if msg.data == "room_target_arrived":
            self._room = FINISH
        if msg.data == "loc_target_arrived":
            self._location = FINISH

    def imageCallback(self, msg):
        if msg.data == "person_target_found":
            self._person = FINISH

    def xfeiCallback(self, msg):
        string = msg.data
        symbols = ["!", "?", ".", ",", ";", ":"]
        output = []
        if string[-1] in symbols:
            string = string[:-1]
        for part in string.lstrip().split(","):
            for word in part.split():
                output.append(word)
        print (output)
        self.parse_output(output)

    
    # 定义完成任务的函数，首先是移动到指定房间, 然后指定地点，然后找到指定人或者物体
    def move_to_room(self):
        msg = mission()
        msg.mission_type = 'room'
        msg.mission_name = str(self.target_room)
        if self._room == UNSTART:
            self.publishr_message(self.nav_pub, msg)
            self._room = PROCESS
    
    def move_to_location(self):
        msg = mission()
        msg.mission_type = 'location'
        msg.mission_name = str(self.target_location)
        if self._location == UNSTART:
            self.publishr_message(self.nav_pub, msg)
            self._location = PROCESS

    def find_person(self):
        msg = mission()
        msg.target_type = 'person'
        msg.mission_name = str(self.target_person)
        if self._person == UNSTART:
            self.publishr_message(self.image_pub, msg)
            self._person = PROCESS
    
    def find_object(self):
        msg = mission()
        msg.target_type = 'object'
        msg.mission_name = str(self.target_object)
        if self._object == UNSTART:
            self.publishr_message(self.image_pub, msg)
            self._object = PROCESS
    
    def answer_question(self, output):
        if "language" in output or "program" in output or "programming" in output:
            if "who" in output:
                self.sh.say("I heard the question", self.voice)
                rospy.sleep(3)
                self.sh.say("Who invented the C programming language", self.voice)
                rospy.sleep(5)
                self.sh.say("The answer is Ken Thompson and Dennis Ritchie", self.voice)
                rospy.sleep(6)
                self.sh.say("Okay i am ready for your next question", self.voice)
                rospy.sleep(4)
            if "When" in output:
                if "C" in output:
                    self.sh.say("I heard the question", self.voice)
                    rospy.sleep(3)
                    self.sh.say("When was the C programming language invented", self.voice)
                    rospy.sleep(5)
                    self.sh.say("The answer is C was developed after B in 1972 at Bell Labs", self.voice)
                    rospy.sleep(7)
                    self.sh.say("Okay i am ready for your next question", self.voice)
                    rospy.sleep(4)
                if "B" in output or "big" in output:
                    self.sh.say("I heard the question", self.voice)
                    rospy.sleep(3)
                    self.sh.say("When was the B programming language invented", self.voice)
                    rospy.sleep(5)
                    self.sh.say("The answer is B was developed circa 1969 at Bell Labs", self.voice)
                    rospy.sleep(7)
                    self.sh.say("Okay i am ready for your next question", self.voice)
                    rospy.sleep(4)
        if "computer" in output or "but" in output or "bug" in output:
            self.sh.say("I heard the question", self.voice)
            rospy.sleep(3)
            self.sh.say("Where does the term computer bug come from", self.voice)
            rospy.sleep(5)
            self.sh.say("The answer is From a moth trapped in a relay", self.voice)
            rospy.sleep(6)
            self.sh.say("Okay i am ready for your next question", self.voice)
            rospy.sleep(4)
        if "compile" in output or "compiler" in output:
            self.sh.say("I heard the question", self.voice)
            rospy.sleep(3)
            self.sh.say("Who invented the first compiler", self.voice)
            rospy.sleep(4)
            self.sh.say("The answer is Grace Brewster Murray Hopper invented it", self.voice)
            rospy.sleep(7)
            self.sh.say("Okay i am ready for your next question", self.voice)
            rospy.sleep(4)
        if "platform" in output:
            if "open" in output:
                self.sh.say("I heard the question", self.voice)
                rospy.sleep(3)
                self.sh.say("Which robot is used in the Open Platform League", self.voice)
                rospy.sleep(7)
                self.sh.say("The answer is There is no standard defined for OPL", self.voice)
                rospy.sleep(7)
                self.sh.say("Okay i am ready for your next question", self.voice)
                rospy.sleep(4)
            if "domestic" in output:
                self.sh.say("I heard the question", self.voice)
                rospy.sleep(3)
                self.sh.say("Which robot is used in the Domestic Standard Platform League", self.voice)
                rospy.sleep(8)
                self.sh.say("The answer is The Toyota Human Support Robot", self.voice)
                rospy.sleep(6)
                self.sh.say("Okay i am ready for your next question", self.voice)
                rospy.sleep(4)
            if "social" in output:
                self.sh.say("I heard the question", self.voice)
                rospy.sleep(3)
                self.sh.say("Which robot is used in the Social Standard Platform League", self.voice)
                rospy.sleep(8)
                self.sh.say("The answer is The SoftBank Robotics Pepper", self.voice)
                rospy.sleep(6)
                self.sh.say("Okay i am ready for your next question", self.voice)
                rospy.sleep(4)
        if "name" in output and "team" in output:
            self.sh.say("I heard the question", self.voice)
            rospy.sleep(3)
            self.sh.say("What's the name of your team", self.voice)
            rospy.sleep(4)
            self.sh.say("The answer is kamerider", self.voice)
            rospy.sleep(4)
            self.sh.say("Okay i am ready for your next question", self.voice)
            rospy.sleep(4)
        if "time" in output:
            curr_time = str(time.strftime('%H:%M:%S'))
            self.sh.say("I heard the question", self.voice)
            rospy.sleep(3)
            self.sh.say("What time is it", self.voice)
            rospy.sleep(3)
            self.sh.say("The answer is "+curr_time, self.voice)
            rospy.sleep(9)
            self.sh.say("Okay i am ready for your next question", self.voice)
            rospy.sleep(4)
        if "day" in output or "today" in output:
            today = datetime.date.today()
            today.strftime("%Y-%m-%d")
            self.sh.say("I heard the question", self.voice)
            rospy.sleep(3)
            self.sh.say("What day is today", self.voice)
            rospy.sleep(3)
            self.sh.say("The answer is "+today, self.voice)
            rospy.sleep(9)
            self.sh.say("Okay i am ready for your next question", self.voice)
            rospy.sleep(4)
        if "dream" in output:
            self.sh.say("I heard the question", self.voice)
            rospy.sleep(3)
            self.sh.say("Do you have dreams", self.voice)
            rospy.sleep(3)
            self.sh.say("The answer is I dream of Electric Sheep", self.voice)
            rospy.sleep(5)
            self.sh.say("Okay i am ready for your next question", self.voice)
            rospy.sleep(4)
        if "city" in output and "next" in output:
            self.sh.say("I heard the question", self.voice)
            rospy.sleep(3)
            self.sh.say("In which city will next year's RoboCup be hosted", self.voice)
            rospy.sleep(6)
            self.sh.say("The answer is It hasn't been announced yet", self.voice)
            rospy.sleep(7)
            self.sh.say("Okay i am ready for your next question", self.voice)
            rospy.sleep(4)
        if "canada" in output:
            if "origin" in output:
                self.sh.say("I heard the question", self.voice)
                rospy.sleep(3)
                self.sh.say("What is the origin of the name Canada", self.voice)
                rospy.sleep(5)
                self.sh.say("The answer is The name Canada comes from the Iroquois word Kanata, meaning village or settlement", self.voice)
                rospy.sleep(13)
                self.sh.say("Okay i am ready for your next question", self.voice)
                rospy.sleep(4)
            if "capital" in output:
                self.sh.say("I heard the question", self.voice)
                rospy.sleep(3)
                self.sh.say("What is the capital of Canada", self.voice)
                rospy.sleep(4)
                self.sh.say("The answer is The capital of Canada is Ottawa", self.voice)
                rospy.sleep(7)
                self.sh.say("Okay i am ready for your next question", self.voice)
                rospy.sleep(4)
            if "national" in output:
                self.sh.say("I heard the question", self.voice)
                rospy.sleep(3)
                self.sh.say("What is the national anthem of Canada", self.voice)
                rospy.sleep(5)
                self.sh.say("The answer is O Canada", self.voice)
                rospy.sleep(4)
                self.sh.say("Okay i am ready for your next question", self.voice)
                rospy.sleep(4)

    def parse_output(self, output):
        # 首先收集输出中的各类关键词
        for room in self.locations.keys():
            if room.lower() in output:
                self.target_room = room
        for person in self.names.keys():
            if person.lower() in output:
                self.target_person = person
        for Object in self.objects.keys():
            if Object.lower() in output:
                self.target_object = Object
                self.target_location = self.objects[Object]['location']
        for mission in self.missions:
            if mission in output:
                self.target_mission = mission
        
        # 根据接受到的关键词执行任务
        if self.target_room == None and self.target_object == None and self.target_mission == None:
            # 若没有接受到有关房间，物体，任务的关键词， 则判断需要回答问题
            self.answer_question(output)
            self.init_params()

        if self.target_room != None and self.target_person != None and self.target_mission == 'answer':
            # 若同时听到房间信息以及回答问题的要求
            # 一般会是要求机器人前往某个房间找到某个人，然后回答问题
            # 则先前往指定房间，在到达房间之后回答问题
            while(1):
                if self._room == UNSTART:
                    self.move_to_room()
                if self._room == FINISH and self._person == UNSTART:
                    self.find_person()
                if self._person == FINISH:
                    temp_str = "I have found " + str(self.target_person)
                    self.sh.say(temp_str, self.voice)
                    rospy.sleep(3)
                    # 播放初始化的音频，并提醒操作者如何使用语音操作Jack
                    self.sh.say("Hello my name is Jack", self.voice)
                    rospy.sleep(3)
                    self.sh.say("Please say Jack to wake me up before each question", self.voice)
                    rospy.sleep(4)
                    self.sh.say("I am ready for your question if you hear", self.voice)
                    rospy.sleep(3.5)
                    play_signal_sound()
                self.init_params()
        
        if self.target_room != None and self.target_object! = None:
            # 若同时听到房间信息和物品信息
            # 则判断是要去某个房间找到或者抓取某个物体
            # 则先前往指定房间，然后找到指定物体
            while(1):
                if self._room == UNSTART:
                    self.move_to_room()
                if self._room == FINISH and self._location == UNSTART:
                    self.move_to_location()
                if self._location == FINISH and self._object == UNSTART:
                    self.find_object()
                self.init_params()
            
        
        
 

    def mission_excute(self):
        # 首先控制机器人移动到指定的房间
        
        # 首先判断是不是要求回答问题

        # 然后根据关键词对机器人进行相应的操作

        

    def cleanup(self):
		rospy.loginfo("shuting down gpsr control node ....")

if __name__ == '__main__':
    rospy.init_node("gpsr_speech_control", anonymous=True)
    ctrl = gpsr_speech_control()
    rospy.spin()




