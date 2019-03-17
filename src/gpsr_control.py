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
import pyaudio
from sound_play.libsoundplay import SoundClient
from read_xml_files import main as read_main

class gpsr_speech_control(object):
    """Class to read the recognition output of pocketsphinx"""
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # Get parameters
        self.voice = rospy.get_param("~voice", "voice_cmu_us_clb_arctic_clunits")
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
        """
            此函数解析语音识别的结果，并转化成字符串列表的形式
        """
        string = str(msg.data)
        punc = [",", "?", "!", "."]
        if string[-1] in punc:
            string = string[:-1]
        output = []
        for words in string.lstrip().split(","):
            for word in words.split():
                output.append(word)
    
    def cleanup(self):
		rospy.loginfo("shuting down gpsr control node ....")

if __name__ == '__main__':
    rospy.init_node("gpsr_speech_control", anonymous=True)
    ctrl = gpsr_speech_control()
    rospy.spin()




