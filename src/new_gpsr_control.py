#! /usr/bin/env python
# -*- coding: utf-8 -*-
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
    #一些参数的初始化
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # Default infomations
        _, self.locations, self.names, self.objects = read_main()
        # Type of task
        self.task_type = None
        # State of task
        self.task_state = [UNSTART, PROCESS, FINISH]

        # Predefined missions
        self.location = []
        for key in self.locations.keys():
            for loc in self.locations[key]:
                self.location.append(loc)
        self.location_adjust = ['planet','TV','couch','coach','bathing','machine','arm','chair','colorado','coffee','tower','towel']
        self.location = self.location + self.location_adjust
        print(self.location)

        self.object = self.dict_to_list(self.objects)
        self.object_adjust= ['tuna','M','pair','pitch','picture','piece','page','pair','pairs']
        self.object = self.object +self.object_adjust
        self.action=['bring', 'take','took','put','give','gave','deliver','delivered','place','pick']
        self.people=['person','people','girl','boy','male','female']
        self.name=['Alex','Charlie','Elizabeth','Francis','Jennifer','Linda','Mary','Patricia','Robin','Skyler','Alex','Charlie','Francis','James','John','Michael','Robert', 'Robin','Skyler','William']
        self.gender=['boy','girl','man','woman']
        #self.gesture=['waving','left arm','right arm','pointing to the left','pointing to the right']
        self.pose=['sitting','seeking','standing','lying']
        ###########################################
        self.navi=['follow','followed','photo','guide','lead','need','lady','escort','accompany','accompanied','company','meet','meeting']
        
        self.room=['corridor','bedroom','dining','living','kitchen','exit','entrance']
        #self.location=[]
        self.gesture = ['waving', 'raising', 'pointing']
        
        # Publisher topics
        self.pub_to_nav_topic_name = None
        self.pub_to_image_topic_name = None
        self.pub_to_arm_topic_name = None
        # Subscriber topics
        self.sub_nav_back_topic_name = None
        self.sub_image_back_topic_name = None
        self.sub_arm_back_topic_name = None
        # Mission keywords
        self.target_action = []
        self.target_people = []
        self.target_room = []
        self.target_location = []
        self.target_object = []
        self.target_navi = []
        self.target_name = []
        self.target_gender = []
        self.target_gesture = []
        self.target_pose = []
        # Mission keywords##############################
        self._room = UNSTART
        self._location = UNSTART
        self._person = UNSTART
        self._object = UNSTART
        self._mission = UNSTART
        
        #self.init_params()
        self.get_params()
    
    def dict_to_list(self,Dict):
        List = Dict.keys()
        print (List)
        return List
    
    def cleanup(self):
        rospy.loginfo("system has died")
    
    #表征目前机器人状态的变量（主要是目前在哪里,在执行任务的哪一步）
    def init_params(self):
        # Mission keywords
        self.target_action = []
        self.target_people = []
        self.target_room = []
        self.target_location = []
        self.target_object = []
        self.target_navi = []
        self.target_name = []
        self.target_gender = []
        self.target_gesture = []
        self.target_pose = []

        # Mission state
        self._room = UNSTART
        self._location = UNSTART
        self._person = UNSTART
        self._object = UNSTART
        self._mission = UNSTART
    #从launch文件中获取topic_name,声明Publisher和Subscriber,播放初始音频
    def get_params(self):
        # Initialize sound client
        self.sh = SoundClient(blocking=True)
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)

        # Get parameters
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.cmd_files = rospy.get_param("~cmd_file", "/home/nvidia/catkin_ws/src/kamerider_speech/CommonFiles")
        #self.pub_to_arm_topic_name   = rospy.get_param("/speech_to_arm")????????
        self.pub_to_arm_topic_name   = rospy.get_param("pub_to_arm_topic_name"  , "/control_to_arm")
        self.pub_to_nav_topic_name   = rospy.get_param("pub_to_nav_topic_name"  , "/control_to_nav")
        self.pub_to_image_topic_name = rospy.get_param("pub_to_image_topic_name", "/control_to_image")

        self.sub_arm_back_topic_name   = rospy.get_param("sub_arm_back_topic_name"  , "/arm_to_control")
        self.sub_nav_back_topic_name   = rospy.get_param("sub_nav_back_topic_name"  , "/nav_to_control")
        self.sub_image_back_topic_name = rospy.get_param("sub_image_back_topic_name", "/image_to_control")
        ############################################接收讯飞识别到的消息
        self.sub_xfei_back_topic_name  = rospy.get_param("sub_xfei_back_topic_name","/baidu_to_control")
        
        
        rospy.Subscriber(self.sub_arm_back_topic_name, String, self.armCallback)
        rospy.Subscriber(self.sub_nav_back_topic_name, String, self.navCallback)
        rospy.Subscriber(self.sub_image_back_topic_name, String, self.imageCallback)
        #############################################
        rospy.Subscriber(self.sub_xfei_back_topic_name,String,self.xfeiCallback)

        self.arm_pub   = rospy.Publisher(self.pub_to_arm_topic_name, String, queue_size=1)
        self.nav_pub   = rospy.Publisher(self.pub_to_nav_topic_name, String, queue_size=1)
        self.image_pub = rospy.Publisher(self.pub_to_image_topic_name, String, queue_size=1)

        # Start gpsr task
        self.start_gpsr()
    # 播放初始化的音频,并提醒操作者如何使用语音操作Jack
    def start_gpsr(self):
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
        if msg.data == "object_target_found":
            self._object = FINISH
    
    #将识别到的句子转成list,然后收集关键词
    def xfeiCallback(self, msg):
        if msg.data == "failed":
            self.sh.say("Sorry i cannot understand your question please speak again", self.voice)
        string = msg.data
        symbols = ["!", "?", ".", ",", ";", ":"]
        output = []
        for part in string.lstrip().split(","):
            for word in part.split():
                output.append(word)
        output = [item.lower() for item in output]
        print (output)
        self.init_params()
        self.parse_output(output)
        
    # 首先收集输出中的各类关键词
    def parse_output(self, output):
        for room in self.room:
            if room in output:
                self.target_room.append(room)
        for location in self.location:
            if location in output or location+'s' in output:
                self.target_location.append(location)
        for person in self.people:
            if person in output:
                self.target_people.append(person)
        for name in self.name:
            if name in output:
                self.target_name.append(name)
        for Object in self.object:
            if Object in output or Object+'s' in output:
                self.target_object.append(Object)
        for action in self.action:
            if action in output:
                self.target_action.append(action)
        for navi in self.navi:
                if navi in output:
                    self.target_navi.append(navi)
        for gender in self.gender:
                if gender in output:
                    self.target_gender.append(gender)
        for gesture in self.gesture:
            # Gesture 匹配存在问题
                if gesture in output:
                    self.target_gesture.append(gesture)
        for pose in self.pose:
                if pose in output:
                    self.target_pose.append(pose)

        
        #根据上面分类的情况,将问题分成5类
        self.target_location = self.target_location + self.target_room
        if self.target_action  and self.target_object:
            #manipulation
            self.task_type = "manipulation"
            print("task type is {}".format(self.task_type))
            if self.target_people:
                if 'me' in output:
                    #find obj
                    #grasp
                    #go back
                    print("go to find {} and bring it to me".format(self.target_object))
                else:
                    #find obj
                    #grasp
                    #find person
                    #give it to the person
                    print("go to find {} and give it to the person who's feature is {}".format(self.target_object,self.target_gender+self.target_gesture+self.target_pose))

            else:
                #find obj
                #grasp
                #go to location[0] if not none
                #place obj
                if self.target_location:
                    print("go to find {} and put it at {}".format(self.target_object,self.target_location[-1]))
        else:
            if self.target_navi and self.target_location:
                #follow&guige
                self.task_type = "navigation"
                print("task type is {}".format(self.task_type))
                #把room和location当做同样的性质处理
                if len(self.target_location) ==1:
                    
                    #go to lacation[0]
                    #find person
                    if 'follow' in output or 'photo' in output:
                        print("go to {}".format(self.target_location[0]))
                        print("find person start follow until \'stop\'")
                        #start follow until hear stop
                    else:
                        print("find person and start navi to {}".format(self.target_location[0]))
                        #find person and start navi to location[0]
                if len(self.target_location) ==2:
                    if output.index(self.target_location[0]) > output.index(self.target_location[1]) and 'find' not in output:
                        self.target_location.reverse()
                    if 'follow' in output:
                        # go to loc[0]
                        print("go to {} and start follow until \'stop\'".format(self.target_location[0]))
                    else:
                        #go to loc[0] start navi to loc[1]
                        print("go to {}, start navi to {}".format(self.target_location[0],self.target_location[1]))
            else:
                if self.target_people  and self.target_location:
                    self.task_type = "people"
                    print("task type is {}".format(self.task_type))
                    if 'me' in output:
                        #go to target_loc[0], find person by feature and go back to answer
                        if 'gender' in output:
                            print("go to {}, recognize the gender of the person and go back to answer".format(self.target_location[0]))
                        if 'pose' in output or 'post' in output:
                            print("go to {}, recognize the pose of the person and go back to answer".format(self.target_location[0]))
                        if 'name' in output:
                            print("go to {}, ask the name of the person and go back to answer".format(self.target_location[0]))
                    else:
                        #go to loc[0] and talk  #talk 写成函数,输入是output #find_person_by_feature(feature)
                        print("go to {}, find the person by feature {} and talk".format(self.target_location[0],self.target_gender+self.target_gesture+self.target_pose))
                else:
                    if self.target_object and self.target_location:
                        self.task_type = "object"
                        print("task type is {}".format(self.task_type))
                        if 'find' in output or 'locate' in output:
                            #find obj
                            print("find {}".format(self.target_object))
                        if 'many' in output:
                            #find the num of obj
                            print("find how many {} are there at {}".format(self.target_object,self.target_location))
                        if 'what' in output:
                            #find the biggest obj 
                            print("find the biggest {} at {}".format(self.target_object,self.target_location))
                    else:
                        self.task_type = "Q&A"
                        print("task type is {}".format(self.task_type))
if __name__ == '__main__':
    rospy.init_node("gpsr_speech_control", anonymous=True)
    ctrl = gpsr_speech_control()
    rospy.spin()                           





                

            

        







                
        




            





                    

        
                




        
    
        
    
