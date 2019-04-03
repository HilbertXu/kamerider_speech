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
        self.months = ['january', 'february', 'march', 'april', 'may', 'june',
                       'july', 'august', 'september', 'october', 'november', 'december']
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
        self.sh = SoundClient(blocking=True)
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
        self.sh.say("Please say Jack to wake me up before each question", self.voice)
        self.sh.say("I am ready for your command if you hear", self.voice)
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
        output = [item.lower() for item in output]
        print (output)
        # self.parse_output(output)
        self.answer_question(output)

    
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
        if "program" in output or "programming" in output or "programmer" in output:
            if "who" in output:
                if "c" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("Who invented the C programming language", self.voice)
                    self.sh.say("The answer is Ken Thompson and Dennis Ritchie", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
                if "python" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("Who created the Python Programming Language", self.voice)
                    self.sh.say("The answer is Python was invented by Guido van Rossum", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
                if "first" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("Who is considered to be the first computer programmer", self.voice)
                    self.sh.say("The answer is Ada Lovelace", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
            if "when" in output:
                if "c" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("When was the C programming language invented", self.voice)
                    self.sh.say("The answer is C was developed after B in 1972 at Bell Labs", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
                if "b" in output or "big" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("When was the B programming language invented", self.voice)
                    self.sh.say("The answer is B was developed circa 1969 at Bell Labs", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
            if "which" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Which program do Jedi use to open PDF files", self.voice)
                self.sh.say("The answer is Adobe Wan Kenobi", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "computer" in output:
            if "but" in output or "bug" in output:
                if "where" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("Where does the term computer bug come from", self.voice)
                    self.sh.say("The answer is From a moth trapped in a relay", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
                if "first" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("What was the first computer bug", self.voice)
                    self.sh.say("The answer is The first actual computer bug was a dead moth stuck in a Harvard Mark II", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
            if "first" in output or "pass" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("What was the first computer in pass the Turing test", self.voice)
                self.sh.say("The answer is Some people think it was IBM Watson, but it was Eugene, a computer designed at England's University of Reading", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "compile" in output or "compiler" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("Who invented the first compiler", self.voice)
            self.sh.say("The answer is Grace Brewster Murray Hopper invented it", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "platform" in output or "which" in output:
            if "open" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Which robot is used in the Open Platform League", self.voice)
                self.sh.say("The answer is There is no standard defined for OPL", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "domestic" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Which robot is used in the Domestic Standard Platform League", self.voice)
                self.sh.say("The answer is The Toyota Human Support Robot", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "social" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Which robot is used in the Social Standard Platform League", self.voice)
                self.sh.say("The answer is The SoftBank Robotics Pepper", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "name" in output and "team" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("What's the name of your team", self.voice)
            self.sh.say("The answer is kamerider", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "time" in output:
            curr_time = str(time.strftime('%H:%M:%S'))
            self.sh.say("I heard the question", self.voice)
            self.sh.say("What time is it", self.voice)
            self.sh.say("The answer is "+curr_time, self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "day" in output or "today" in output:
            today = datetime.date.today()
            today = today.strftime("%Y-%m-%d")
            year = today[0]
            month = today
            self.sh.say("I heard the question", self.voice)
            self.sh.say("What day is today", self.voice)
            self.sh.say("The answer is "+today, self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "dream" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("Do you have dreams", self.voice)
            self.sh.say("The answer is I dream of Electric Sheep", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "city" in output and "next" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("In which city will next year's RoboCup be hosted", self.voice)
            self.sh.say("The answer is It hasn't been announced yet", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "canada" in output:
            if "origin" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("What is the origin of the name Canada", self.voice)
                self.sh.say("The answer is The name Canada comes from the Iroquois word Kanata, meaning village or settlement", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "capital" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("What is the capital of Canada", self.voice)
                self.sh.say("The answer is The capital of Canada is Ottawa", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "national" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("What is the national anthem of Canada", self.voice)
                self.sh.say("The answer is O Canada", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "handsome" in output or "person" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Who's the most handsome person in Canada?", self.voice)
                self.sh.say("The answer is I that Justin Trudeau is very handsome", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "time" in output or "many" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("How many time zones are there in Canada", self.voice)
                self.sh.say("The answer is Canada spans almost 10 million square km and comprises 6 time zones", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "usa" in output or "year" in output:
                if "first" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("In what year was Canada invaded by the USA for the first time", self.voice)
                    self.sh.say("The answer is The first time that the USA invaded Canada was in 1775", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
                if "second" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("What year was Canada invaded by the USA for the second time", self.voice)
                    self.sh.say("The answer is The USA invaded Canada a second time in 1812", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
            if "why" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Why is Canada named Canada", self.voice)
                self.sh.say("The answer is French explorers misunderstood the local native word Kanata, which means village", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "desert" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Where is Canada's only desert", self.voice)
                self.sh.say("The answer is Canada's only desert is British Columbia", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)

        if "world" in output or "word" in output or "worlds" in output or "words" in output:
            if "longest" in output or "street" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("What's the longest street in the world", self.voice)
                self.sh.say("The answer is Yonge Street in Ontario is the longest street in the world", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "largest" in output or "coin" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("What is the world's largest coin", self.voice)
                self.sh.say("The answer is The Big Nickel in Sudbury, Ontario. It is nine meters in diameter", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "street" in output or "how" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("How long is Yonge Street in Ontario", self.voice)
            self.sh.say("The answer is Yonge street is almost 2,000 km, starting at Lake Ontario, and running north to the Minnesota border", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "london" in output and "name" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("What's the name of the bear cub exported from Canada to the London Zoo in 1915", self.voice)
            self.sh.say("The answer is The bear cub was named Winnipeg. It inspired the stories of Winnie-the-Pooh", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "smartphone" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("Where was the Blackberry Smartphone developed", self.voice)
            self.sh.say("The answer is It was developed in Ontario, at Research In Motion's Waterloo offices", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "country" in output:
            if "record" in output or "gold" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("What country holds the record for the most gold medals at the Winter Olympics", self.voice)
                self.sh.say("The answer is Canada does! With 14 Golds at the 2010 Vancouver Winter Olympics", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "who" in output and "coin" in output:
            if "term" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Where was the Blackberry Smartphone developed", self.voice)
                self.sh.say("The answer is It was developed in Ontario, at Research In Motion's Waterloo offices", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "mounted" in output or "police" in output:
            if "royal" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("When was The Royal Canadian Mounted Police formed", self.voice)
                self.sh.say("The answer is In 1920, when The Mounted Police merged with the Dominion Police", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            else:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("When was The Mounted Police formed", self.voice)
                self.sh.say("The answer is The Mounted Police was formed in 1873", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "how" in output and "big" in output:
            if "RCMP" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("How big is the RCMP", self.voice)
                self.sh.say("The answer is Today, the RCMP has close to 30,000 members", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "canda" in output or "desert" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("How big is Canada's only desert", self.voice)
                self.sh.say("The answer is The British Columbia desert is only 15 miles long", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "first" in output or "hard" in output or "disk" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("How big was the first hard disk drive", self.voice)
                self.sh.say("The answer is The IBM 305 RAMAC hard disk weighed over a ton and stored 5 MB of data", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "montreal" in output or "else" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("What else is Montreal called", self.voice)
            self.sh.say("The answer is Montreal is often called the City of Saints or the City of a Hundred Bell Towers", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "hotel" in output:
            if "where" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Where is The Hotel de Glace located", self.voice)
                self.sh.say("The answer is The Hotel de Glace is in Quebec", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "how" in output or "many" in output:
                if "ice" in output or "see" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("How many tons of ice are required to build The Hotel de Glace", self.voice)
                    self.sh.say("The answer is The Hotel de Glace requires about 400 tons of ice", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
                if "snow" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("How many tons of snow are required to build The Hotel de Glace", self.voice)
                    self.sh.say("The answer is Every year, 12000 tons of snow are used for The Hotel de Glace", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
                if "can" in output or "visit" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("Can I visit the Hotel de Glace in summer", self.voice)
                    self.sh.say("The answer is No. Every summer it melts away, only to be rebuilt the following winter", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
        if "name" in output:
            if "famous" in output:
                if "male" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("Name 3 famous male Canadians", self.voice)
                    self.sh.say("The answer is Leonard Cohen, Keanu Reeves, and Jim Carrey", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
                if "female" in output:
                    self.sh.say("I heard the question", self.voice)
                    self.sh.say("Name 3 famous female Canadians", self.voice)
                    self.sh.say("The answer is Celine Dion, Pamela Anderson, and Avril Lavigne", self.voice)
                    self.sh.say("Okay i am ready for your next question", self.voice)
            if "robots" in output or "all" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Name all of the robots on Mars", self.voice)
                self.sh.say("The answer is There are four robots on Mars: Sojourner, Spirit, Opportunity, and Curiosity. Three more crashed on landing", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "what" in output and "origin" in output:
            if "comic" in output or "font" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("What's the origin of the Comic Sans font", self.voice)
                self.sh.say("The answer is Comic Sans is based on Dave Gibbons' lettering in the Watchmen comic books", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "nanobot" in output or "nano" in output:
            if "what" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Where is The Hotel de Glace located", self.voice)
                self.sh.say("The answer is The Hotel de Glace is in Quebec", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "how" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("How small can a nanobot be", self.voice)
                self.sh.say("The answer is A nanobot can be less than one-thousandth of a millimeter", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "why" in output and "award" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("Why wasn't Tron nominated for an award by The Motion Picture Academy", self.voice)
            self.sh.say("The answer is The Academy thought that Tron cheated by using computers", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "hard" in output and "disk" in output:
            if "which" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Which was the first computer with a hard disk drive", self.voice)
                self.sh.say("The answer is The IBM 305 RAMAC", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "when" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("When was the first computer with a hard disk drive launched", self.voice)
                self.sh.say("The answer is The IBM 305 RAMAC was launched in 1956", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
            if "how" in output or "big" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("How big was the first hard disk drive", self.voice)
                self.sh.say("The answer is The IBM 305 RAMAC hard disk weighed over a ton and stored 5 MB of data", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "what" in output and "stand" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("What does CAPTCHA stands for", self.voice)
            self.sh.say("The answer is CAPTCHA is an acronym for Completely Automated Public Turing test to tell Computers and Humans Apart", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "first" in output and "android" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("Who is the world's first android", self.voice)
            self.sh.say("The answer is Professor Kevin Warwick uses chips in his arm to operate doors, a robotic hand, and a wheelchair", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "knight" in output and "mechanical" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("What is a Mechanical Knight", self.voice)
            self.sh.say("The answer is A robot sketch made by Leonardo DaVinci", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "paradox" in output or "state" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("What does Moravec's paradox state", self.voice)
            self.sh.say("The answer is Moravec's paradox states that a computer can crunch numbers like Bernoulli, but lacks a toddler's motor skills", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "knowledge" in output or "engineering" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("What is the AI knowledge engineering bottleneck", self.voice)
            self.sh.say("The answer is It is when you need to load an AI with enough knowledge to start learning", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "why" in output:
            if "worried" in output or "impact" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Why is Elon Musk is worried about AI's impact on humanity", self.voice)
                self.sh.say("The answer is I don't know. He should worry more about the people's impact on humanity", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "you" in output or "do" in output:
            if "threat" in output or "humanity" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Do you think robots are a threat to humanity", self.voice)
                self.sh.say("The answer is No Humans are the real threat to humanity", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "what" in output and "chatbot" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("What is a chatbot", self.voice)
            self.sh.say("The answer is A chatbot is an A.I. you put in customer service to avoid paying salaries", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "car" in output and "safe" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("Are self-driving cars safe", self.voice)
            self.sh.say("The answer is Yes. Car accidents are product of human misconduct", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)
        if "robot" in output:
            if "mark" in output or "is" in output:
                self.sh.say("I heard the question", self.voice)
                self.sh.say("Is Mark Zuckerberg a robot", self.voice)
                self.sh.say("The answer is Sure. I've never seen him drink water", self.voice)
                self.sh.say("Okay i am ready for your next question", self.voice)
        if "apple" in output:
            self.sh.say("I heard the question", self.voice)
            self.sh.say("Who is the inventor of the Apple I microcomputer", self.voice)
            self.sh.say("The answer My lord and master Steve Wozniak", self.voice)
            self.sh.say("Okay i am ready for your next question", self.voice)

    def parse_output(self, output):
        # 首先收集输出中的各类关键词
        rospy.loginfo("Parsing output")
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
        
        rospy.loginfo("Detected following keywords")
        print ("Target room: "+self.target_room)
        print ("Target location: "+self.target_location)
        print ("Target object: "+self.target_object)
        print ("Target person: "+self.target_person)
        print ("Target mission: "+self.target_mission)
        
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
                    # 播放初始化的音频，并提醒操作者如何使用语音操作Jack
                    self.sh.say("Hello my name is Jack", self.voice)
                    self.sh.say("Please say Jack to wake me up before each question", self.voice)
                    self.sh.say("I am ready for your question if you hear", self.voice)
                    play_signal_sound()
                self.init_params()
        
        if self.target_room != None and self.target_object != None:
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
        return 0
        

    def cleanup(self):
		rospy.loginfo("shuting down gpsr control node ....")

if __name__ == '__main__':
    rospy.init_node("gpsr_speech_control", anonymous=True)
    ctrl = gpsr_speech_control()
    rospy.spin()




