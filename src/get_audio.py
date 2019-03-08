#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 2019/03/06
    Author: Xu Yucheng
    Abstract: 记录麦克风的输入音频并保存
"""
import os
import sys
import time
import rospy
import wave
import pyaudio
from std_msgs.msg import Int8
from std_msgs.msg import String

class get_audio():
    def __init__(self):
        """
            初始化函数，初始化录音时的各项参数，以及ros话题和参数
            构想是使用pocketsphinx进行一些短的关键词的识别，主要是唤醒词jack以及终止词ok
            然后当pocketshpinx识别到唤醒词和终止词并通过话题输出之后，此脚本接受到话题的输出之后判断开始录音还是终止录音
        """
        # Record params
        self.CHUNK = 256
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 11025
        self.RECORD_SECONDS = 10
        # Ros params
        self.start_record = False
        self.stop_record = False
        self.project_name = '/home/kamerider/catkin_ws/src/kamerider_speech/sounds/gpsr'
        self.count = 0
        self.sub_pocketsphinx_topic_name = None
        self.pub_record_end_topic_name = None
        self.pub_record_index_topic_name = None
        self.get_params()
    
    def setup_recorder(self):
        self.recorder = pyaudio.PyAudio()
    
    def get_params(self):
        self.sub_pocketsphinx_topic_name = rospy.get_param("sub_pocketsphinx_topic_name", "/jsgf_audio")
        self.pub_record_end_topic_name    = rospy.get_param("pub_record_end_topic_name",   "/audio_record")
        self.pub_record_index_topic_name  = rospy.get_param("pub_record_index_topic_name", "/audio_index")
        
        rospy.Subscriber(self.sub_pocketsphinx_topic_name, String, self.pocketCallback)
        self.pub_record = rospy.Publisher(self.pub_record_end_topic_name, String, queue_size=1)
        self.pub_index  = rospy.Publisher(self.pub_record_index_topic_name, Int8, queue_size=1)

    def pocketCallback(self, msg):
        if msg.data.lower() == 'jack':
            self.start_record = True
            while(True):
                self.get_audio()
        
        if msg.data.lower() == 'ok':
            self.stop_record = True
        
    def get_audio(self):
        if self.start_record:
            self.setup_recorder()
            file_name = self.project_name + '_' + str(self.count) + '.wav'
            print ("Start to record input audio and save to file: %s"%(file_name))
            stream = self.recorder.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK
            )
            frames = []
            for i in range(int(self.RATE / self.CHUNK * self.RECORD_SECONDS)):
                if self.stop_record:
                    print ("Stop recording")
                    break
                data = stream.read(self.CHUNK)
                frames.append(data)
            print ("Recording finised now save .wav file")

            stream.stop_stream()
            stream.close()
            self.recorder.terminate()

            wf = wave.open(file_name, 'wb')
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.recorder.get_sample_size(self.FORMAT))
            wf.setframerate(self.RATE)
            wf.writeframes(b''.join(frames))
            wf.close()

            self.pub_index.publish(self.count)
            self.start_record = False
            self.count += 1

if __name__ == '__main__':
    rospy.init_node('get_audio')
    audio = get_audio()
    rospy.spin()
             
            