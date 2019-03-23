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
from sound_play.libsoundplay import SoundClient
from read_xml_files import main as read_main

class gpsr_speech_control(object):
    """Class to read the recognition output of pocketsphinx"""
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # Get parameters
        self.voice = rospy.get_param("~voice", "voice_us2_mbrola")
        self.question_start_signal = rospy.get_param("~question_start_signal", "/home/kamerider/catkin_ws/src/kamerider_speech/sounds/question_start_signal.wav")
        self.cmd_files = rospy.get_param("~cmd_file", "/home/kamerider/catkin_ws/src/kamerider_speech/CommonFiles")

        # Default infomations
        self.gestures, self.locations, self.names, self.objects = read_main()

        # Type of task
        self.task_type = ['person', 'object', 'object2person', 'Q&A']

        self.sh = SoundClient()
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)
        self.start_gpsr()
        rospy.Subscriber("/xfei_output", String, self.xfeiCallback)

    def play_signale_sound(self):
        chunk = 1024
        # 打开 .wav 音频文件
        f = wave.open(self.question_start_signal, 'rb')
        # 初始化pyaudio
        p = pyaudio.PyAudio()
        # 打开一个stream
        stream = p.open(
            format = p.get_format_from_width(f.getsampwidth()),
            channels = f.getnchannels(),
            rate = f.getframerate(),
            output = True
        )
        # 读取音频文件中的数据
        data = f.readframes(chunk)

        # 播放音频文件
        while data != '':
            stream.write(data)
            data = f.readframes(chunk)
        
        # 终止stream
        stream.stop_stream()
        stream.close()
        # 关闭pyaudio
        p.terminate()

    def start_gpsr(self):
        self.sh.say("Hello my name is Jack", self.voice)
        rospy.sleep(3)
        self.sh.say("Please say Jack to wake me up before each question", self.voice)
        rospy.sleep(5)
        self.sh.say("I am ready for your question if you hear", self.voice)
        rospy.sleep(3.5)
        self.play_signale_sound()
    
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

    def parse_output(self, output):
        if "question" in output:
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
                if "name" in output or "team" in output:
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


            


            

                
                




                    


    
    def cleanup(self):
		rospy.loginfo("shuting down gpsr control node ....")

if __name__ == '__main__':
    rospy.init_node("gpsr_speech_control", anonymous=True)
    ctrl = gpsr_speech_control()
    rospy.spin()




