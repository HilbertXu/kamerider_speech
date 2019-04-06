#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 2019/03/07
    Author: Xu Yucheng
    Abstract: speech recognition codes with xfei api
"""

import os
import sys
import time
import requests
import json
import hashlib
import base64
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from aip import AipSpeech

class speech_recognition(object):
    def __init__(self):
        self.app_id = '15944331'
        self.api_key = 'bpNdxEhagyalZVtC6fddFeGZ'
        self.key = '1QRtmDnXAA9TUQuGH8zHL5K1OW2GnpDu'
        self.client = AipSpeech(self.app_id, self.api_key, self.key)

        self.audio_folder = "/home/kamerider/catkin_ws/src/kamerider_speech/sounds/gpsr_record/"

        self.sub_audio_topic_name = None
        self.pub_recognition_result_topic_name = None
        self.is_restart = None
        self.get_params()
        """
        # 由参数服务器中的参数判断当前是否是重启节点
        if self.is_restart != False:
            self.restart()
        else:
            if os.path.isdir(self.audio_folder) == False:
                os.makedirs(self.audio_folder)
            os.chdir(self.audio_folder)
            os.system("rm -rf *")
        """
    
    def restart(self):
        # 当讯飞因为使用时间过长而Timeout之后，通过rosnode kill和launch文件中的respawn选项重启这个节点
        # 重启这个节点之后需要读取刚刚没有识别出来的语音文件再次识别
        print ("Node restart DETECTED!")
        file_num = len(os.listdir(self.audio_folder))
        path_to_wav = self.audio_folder + 'gpsr_' + str(file_num-1) + '.wav'
        self.get_wav(path_to_wav)
    
    def get_audio_file(self, path):
        with open(path, 'rb') as fp:
            return fp.read() 

    
    def get_params(self):
        # ROS params
        self.is_restart = rospy.get_param("/is_restart", "")
        self.sub_audio_topic_name = rospy.get_param("sub_audio_topic_name", "/audio_index")
        self.pub_recognition_result_topic_name = rospy.get_param("pub_recognition_result_topic_name", "/baidu_to_speech")
        # ROS subscriber & publisher
        rospy.Subscriber(self.sub_audio_topic_name, Int8, self.audioCallback)
        self.pub_result = rospy.Publisher(self.pub_recognition_result_topic_name, String, queue_size=1)
    
    def audioCallback(self, msg):
        audio_index = str(msg.data)
        path_to_wav = self.audio_folder + "gpsr_" + audio_index + ".wav"
        result = self.client.asr(self.get_audio_file(path_to_wav), 'wav', 16000, {
            'dev_pid':1737,
        })
        print (result)

if __name__ == '__main__':
    rospy.init_node("speech_recognition")
    baidu = speech_recognition()
    rospy.spin()
    
