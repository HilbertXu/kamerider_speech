#! /usr/bin/env python

'''
Date: 2018/12/17
Author: Xu Yucheng 
GPSR中的语音模块，主要实现通过socket获取到PocketShpinx识别出来的语句块
然后重复，同时根据关键词对机器人进行一系列控制
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
from sound_play.libsoundplay import SoundClient

class general_purpose_service_robot:
    def __init__(self):
        rospy.on_shutdown (self.cleanup)

        self.object_name = None
        self.voice = rospy.get_param ("~voice", "voice_us2_mbrola")
        self.question_start_signal = rospy.get_param ("~question_start_signal", "")

        self.state = "true"
        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.soundhandle.stopAll()
        rospy.sleep(1)

        #------------------------------------
        #|   ROS Publishers & Subscribers   |
        #------------------------------------
    


    def cleanup (self):
        rospy.loginfo ("Shutting down GPSR node")