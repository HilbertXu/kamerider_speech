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
from sound_play import SoundClient
from read_xml_files import main as read_main

class gpsr_speech_control(object):
    """Class to read the recognition output of pocketsphinx"""
    def __init__(self):
        rospy.on_shutdown()
        # Get parameters
        self.voice = rospy.get_param("~voice", "voice_cmu_us_clb_arctic_clunits")
        self.question_start_signal = rospy.get_param("~question_start_signal", "/home/kamerider/catkin_ws/src/kamerider_speech/sounds")
        self.cmd_files = rospy.get_param("~cmd_file", "/home/kamerider/catkin_ws/src/kamerider_speech/CommonFiles")

        # Default infomations
        self.gestures, self.locations, self.names, self.objects = read_main()

        # Type of task
        self.task_type = ['person', 'object', 'object2person', 'Q&A']


