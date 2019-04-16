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
from answer_question import answer_question
from speech_rec_correction import speech_rec_correction
from kamerider_image_msgs.msg import GenderDetection
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
        rospy.loginfo("GPSR control online~")
        # Type of task
        self.task_type = None
        # State of task
        self.task_state = [UNSTART, PROCESS, FINISH]

        # Predefined missions
        self.location = []
        for key in self.locations.keys():
            for loc in self.locations[key]:
                self.location.append(loc)

        self.object = self.dict_to_list(self.objects)
        self.action=['bring', 'take','took','put','give','gave','deliver','delivered','place','pick']
        self.people=['person','people','girl','boy','male','female']
        self.name=['Alex','Charlie','Elizabeth','Francis','Jennifer','Linda','Mary','Patricia','Robin','Skyler','Alex','Charlie','Francis','James','John','Michael','Robert', 'Robin','Skyler','William']
        self.gender=['boy','girl','man','woman','male','famale']
        #self.gesture=['waving','left arm','right arm','pointing to the left','pointing to the right']
        self.pose=['sitting','seeking','standing','lying']
        self.navi=['follow','followed','guide','lead','need','lady','escort','accompany','accompanied','company','meet','meeting']
        self.room=['corridor','bedroom','dining','living room','kitchen','bathroom','exit','entrance']
        #self.location=[]
        self.gesture = ['waving', 'raising left', 'raising right', 'pointing left', 'pointing right']
        self.talk = ["affiliation", "relation", "joke", "yourself"]
        
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
        return List
    
    def cleanup(self):
        rospy.loginfo("system has died")
    
    #表征目前机器人状态的变量（主要是目前在哪里,在执行任务的哪一步）
    def init_params(self):
        # Mission keywords
        self.target_detail = []
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
	
	# Person status
	self._person_pose = []
	self._male_num = 0
	self._female_num = 0
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
        rospy.Subscriber("/kamerider_speech/input", String, self.inputCallback)
	#######################3
	rospy.Subscriber("kamerider_image/gender_recognition",GenderDetection,self.image_gender_detectionCallback)

        self.arm_pub   = rospy.Publisher(self.pub_to_arm_topic_name, mission, queue_size=1)
        self.nav_pub   = rospy.Publisher(self.pub_to_nav_topic_name, mission, queue_size=1)
        self.image_pub = rospy.Publisher(self.pub_to_image_topic_name, mission, queue_size=1)

        # Start gpsr task
        self.start_gpsr()
	
    #############
    def image_gender_detectionCallback(self,msg):
	self._male_num = msg.male_num
	self._female_num = msg.female_num
    
    def publishr_message(self, pub, msg):
        pub.publish(msg)

    def inputCallback(self, msg):
        str = msg.data
        self.sh.say(str, self.voice)

    # 播放初始化的音频,并提醒操作者如何使用语音操作Jack
    def start_gpsr(self):
        self.sh.say("Hello my name is Jack", self.voice)
        self.sh.say("Please say Jack to wake me up before each question", self.voice)
        self.sh.say("I am ready for your command if you hear", self.voice)
        play_signal_sound()

    # 定义完成任务的函数
    # 我暂时使用函数来实现对每一个模块的控制
    # 如何使用import的问题，等我下午回来再一起讨论
    # 导航模块我已经写好了接口，能够直接接受这边的消息然后导航到room或者location
    # 图像模块问题比较复杂，还剩下一个pose detect没做
    # 机械臂...那一套东西直接调用就行，然后真的看缘分吧
    # 另外我们得记得去买一下去年世界赛给出来的物品，然后做一下不同环境下的训练集了，晚上去华联找找吧
    # 这几个函数的调用都是直接调用类内函数，然后参数给一个目标的名称就行，记得加上self哦
    # 回答问题的接口我也做好了，在kamerider_speech/src里面有一个answer_question.py文件里面写了注释
    # 调用方法是直接answer_question(self.sh, self.voice, output)

    def move_to_location(self, location_name):
        msg = mission()
        msg.mission_type = 'location'
        msg.mission_name = str(location_name)
        if self._location == UNSTART or self._location == FINISH:
            self.publishr_message(self.nav_pub, msg)
            self._location = PROCESS

    def find_person(self, person_name):
        msg = mission()
        msg.mission_type = 'person'
        msg.mission_name = str(person_name)
        if self._person == UNSTART or self._person == FINISH:
            self.publishr_message(self.image_pub, msg)
            self._person = PROCESS

    def find_object(self, object_name):
        # @TODO
        # 需要根据需求把mission修改为"grasp", "find"
        msg = mission()
        msg.mission_type = 'object'
        msg.mission_name = str(object_name)
        if self._object == UNSTART or self._object == FINISH:
            self.publishr_message(self.image_pub, msg)
            self._object = PROCESS
    def detect_gesture(self, gesture, Type):
        msg = mission()
        msg.mission_type = Type
        msg.mission_name = gesture
        if self._mission == UNSTART or self._mission == FINISH:
            self.publishr_message(self.image_pub, msg)
            self._mission = PROCESS
        

    def armCallback(self, msg):
        if msg.data == "object_target_grasped":
            self._object = FINISH
    
    def navCallback(self, msg):
        # if msg.data == "room_target_arrived":
        #     self._room = FINISH
        if msg.data == "loc_target_arrived":
            self._location = FINISH

    def imageCallback(self, msg):
        if msg.data == "person_target_found":
            self._person = FINISH
        if msg.data == "object_target_found":
            self._object = FINISH
        if msg.data == "target_gender_detected" or msg.data == "target_pose_detected":
            # 如果找到指定姿态的人，或者单纯的识别到目标地点人的姿态，则返回detected
            # 然后返回到起始地点回答
            self._mission = FINISH
        if msg.data == "target_gender_not_detected" or msg.data == "target_pose_not_detected":
            # 如果当前识别到的人的gesture并不是目标gesture，则重新进入找人的循环
            self._person = UNSTART

    #将识别到的句子转成list,然后收集关键词
    def xfeiCallback(self, msg):
        if msg.data == "failed":
            self.sh.say("Sorry i cannot understand your question please speak again", self.voice)
        string = msg.data
        symbols = ["!", "?", ".", ",", ";", ":"]
        output = []
        for part in string.lstrip().split(","):
            for word in part.split():
                for symbol in symbols:
                    if symbol in word:
                        word = word[:-1] #去除了可能出现的句子中间的标点符号
                output.append(word)
        output = [item.lower() for item in output]
        print (output)
        speech_rec_correction(output)
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
        for word in self.talk:
            if word in output:
                self.target_detail.append(word)

        
        #根据上面分类的情况,将问题分成5类
        # ["manipulation", "navigation", "people", "object", "Q&A"]
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
                    if 'follow' in output:
                        print("go to {}".format(self.target_location[0]))
                        print("find person start follow until \'stop\'")
                        #go to lacation[0]
                        self.sh.say("Now I start navigation", self.voice)
                        self.move_to_location(self.target_location[0])
                        #find person
                        while(True):
                            if self._location == FINISH:
                                rospy.loginfo("Start finding person")
                                self.sh.say("Now start finding person, please wait", self.voice)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection person_detection.py'") 
                                rospy.sleep(5)
                                self.find_person("none")
                                break
                        #start follow until hear stop
                        while (True):
                            if self._person == FINISH:
                                rospy.loginfo("Find person finished start to follow")
                                os.system("rosnode kill /person_detection")
                                os.system("gnome-terminal -x bash -c 'roslaunch turtlebot_follower follower.launch'")
                                self.sh.say("Please stand in front of me and lead me", self.voice)
				self.sh.say("I will stop after hearing jack stop")
                                rospy.sleep(2)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_speech follower_control.py'")
                                break
                    else:
                        print("find person and start navi to {}".format(self.target_location[0]))
                        #find person and start navi to location[0]
                        rospy.loginfo("Start finding person")
                        self.sh.say("Now start finding person, please wait", self.voice)
                        os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection person_detection.py'") 
                        rospy.sleep(5)
                        self.find_person("none")
                        #go to lacation[0]
                        while (True):
                            if self._person ==FINISH:
                                self.sh.say("Now I start navigation,please follow me",self.voice)
                                self.move_to_location(self.target_location[0])
                if len(self.target_location) ==2:
                    if output.index(self.target_location[0]) > output.index(self.target_location[1]) and 'find' not in output:
                        self.target_location.reverse()
                    if 'follow' in output:
                        # go to loc[0]
                        print("go to {} and start follow until \'stop\'".format(self.target_location[0]))
                        self.sh.say("Now I start navigation", self.voice)
                        self.move_to_location(self.target_location[0])
                        while (True):
                            if self._location == FINISH:
                                rospy.loginfo("Start finding person")
                                self.sh.say("Now start finding person, please wait", self.voice)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection person_detection.py'") 
                                rospy.sleep(5)
                                self.find_person("none")
                                break
                        while (True):
                            if self._person == FINISH:
                                rospy.loginfo("Find person finished start to follow")
                                os.system("rosnode kill /person_detection")
                                os.system("gnome-terminal -x bash -c 'roslaunch turtlebot_follower follower.launch'")
                                self.sh.say("Please stand in front of me and lead me", self.voice)
				self.sh.say("I will stop after hearing jack stop")
                                rospy.sleep(2)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_speech follower_control.py'")
                                break

                    else:
                        #go to loc[0] start navi to loc[1]
                        print("go to {}, start navi to {}".format(self.target_location[0],self.target_location[1]))
                        self.sh.say("Now I start navigation", self.voice)
                        self.move_to_location(self.target_location[0])
                        while (True):
                            if self._location == FINISH:
                                rospy.loginfo("Start finding person")
                                self.sh.say("Now start finding person, please wait", self.voice)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection person_detection.py'") 
                                rospy.sleep(5)
                                self.find_person("none")
                                break
                        while (True):
                            if self._person == FINISH:
                                os.system("rosnode kill /person_detection")
                                string = "Wow i have found you, please follow me to " + self.target_location[1]
                                self.sh.say (string, self.voice)
                                self.move_to_location(self.target_location[1])
                                break

            else:
                if self.target_people  and self.target_location:
                    self.task_type = "people"
                    print("task type is {}".format(self.task_type))
                    if 'me' in output:
                        #go to target_loc[0], recognize person by feature and go back to answer
                        if 'gender' in output:
                            print("go to {}, recognize the gender of the person and go back to answer".format(self.target_location[0]))
                            self.sh.say("Now I start navigation", self.voice)
                            self.move_to_location(self.target_location[0])
                            while (True):
                                if self._location == FINISH:
                                    rospy.loginfo("Start finding person")
                                    self.sh.say("Now start finding person, please wait", self.voice)
                                    os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection person_detection.py'") 
                                    rospy.sleep(5)
                                    self.find_person("none")
                                    break
                            while (True):
                                if self._person == FINISH:
                                    rospy.loginfo("Start gender recognition")
                                    os.system("rosnode kill /person_detection")
                                    os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection gender_recognition.py'")
                                    rospy.sleep(5)
                                    self.detect_gesture("none", "gender")
                                    break
                            while (True):
                                if self._mission == FINISH:
                                    rospy.loginfo("Back to the start point")
                                    # @TODO
                                    # 此处修改为设定好的start point
                                    rospy.sleep(12)
                                    self._location=UNSTART
                                    self.move_to_location("entrance")
                                    break
                            while (True):
                                if self._location==FINISH:
                                    rospy.sleep(5)
				    string = "the male number is " +str(self._male_num) +" the female number is " +str(self._female_num)
				    print(string)
			            self.sh.say(string,self.voice)
                                    break

                        if 'pose' in output or 'post' in output:
                            print("go to {}, recognize the pose of the person and go back to answer".format(self.target_location[0]))
                            self.sh.say("Now I start navigation", self.voice)
                            self.move_to_location(self.target_location[0])
                            while (True):
                                if self._location == FINISH:
                                    rospy.loginfo("Start finding person")
                                    self.sh.say("Now start finding person, please wait", self.voice)
                                    os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection person_detection.py'") 
                                    rospy.sleep(5)
                                    self.find_person("none")
                                    break
                            while (True):
                                if self._person == FINISH:
                                    rospy.loginfo("Start pose recognition")
                                    os.system("rosnode kill /person_detection")
                                    os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection pose_detection.py'")
                                    self.detect_gesture("none", "pose")
                                    break
                            while (True):
                                if self._mission == FINISH:
                                    rospy.loginfo("Back to the start point")
                                    # @TODO
                                    # 此处修改为设定好的start point
                                    self.move_to_location("entrance")
                                    break
                        if 'name' in output:
                            print("go to {}, ask the name of the person and go back to answer".format(self.target_location[0]))
                            while (True):
                                if self._location == FINISH:
                                    rospy.loginfo("Start finding person")
                                    self.sh.say("Now start finding person, please wait", self.voice)
                                    os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection person_detection.py'") 
                                    rospy.sleep(5)
                                    self.find_person("none")
                                    break
                            while (True):
                                if self._person == FINISH:
                                    os.system("rosnode kill /person_detection")
                                    self.sh.say("Hello my name is Jack, Please tell me your name", self.voice)
                                    # @TODO not finished
                                    break
                    else:
                        #go to loc[0] and talk  #talk 写成函数,输入是output #find_person_by_feature(feature)
                        print("go to {}, find the person by feature {} and talk".format(self.target_location[0],self.target_gender+self.target_gesture+self.target_pose))
                        self.sh.say("Now I start navigation", self.voice)
                        gesture = ''
                        if self.target_gesture:
                            gesture = self.target_gesture[0]
                        if self.target_pose:
                            gesture = self.target_pose[0]
                            
                        self.move_to_location(self.target_location[0])
                        while (True):
                            if self._location == FINISH:
                                rospy.loginfo("Start finding person")
                                self.sh.say("Now start finding person, please wait", self.voice)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection person_detection.py'") 
                                rospy.sleep(5)
                                self.find_person("none")
                                break
                        while (True):
                            # 如果没有找到指定特征的人，则将_person置为UNSTART，然后重新开始找人
                            if self._person == UNSTART:
                                self.find_person("none")
                            # 目前只考虑pose和gender， 姓名暂时无法完成
                            # 如果当前任务状态(_mission)为UNSTART, 表明还没有开启gesture识别的节点
                            # 开启节点，并且发布消息进行识别
                            if self._person == FINISH and (self.target_gesture or self.target_pose) and self._mission == UNSTART:
                                rospy.loginfo("Start gender recognition")
                                os.system("rosnode kill /person_detection")
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection pose_detection.py'")
                                rospy.sleep(5)
                                self.detect_gesture("pose", gesture)

                            if self._person == FINISH and self.target_gender and self._mission == UNSTART:
                                rospy.loginfo("Start gender recognition")
                                os.system("rosnode kill /person_detection")
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_detection gender_recognition.py'")
                                rospy.sleep(3)
                                self.detect_gesture("gender", self.target_gender)

                            # 如果任务状态(_mission)为PROCESS， 表明节点已经开启，但是没有找到指定的gesture
                            # 此时如果_person为FINISH， 表明已经找到了下一个人，
                            if self._person == FINISH and self.target_gesture and self._mission == PROCESS:
                                rospy.loginfo("Start gender recognition")
                                self.detect_gesture("pose", gesture)

                            if self._person == FINISH and self.target_gender and self._mission == PROCESS:
                                rospy.loginfo("Start gender recognition")
                                self.detect_gesture("gender", gesture)
                            
                            # 如果完成了指定gesture的识别，则进入说话环节
                            # 按照要求讲笑话，介绍自己，介绍队伍，或者回答问题
                            if self._mission == FINISH:
                                rospy.loginfo("Target gesture detected!")
                                string = "i have found the person {}".format(self.target_gender+self.target_gesture+self.target_pose)
                                self.sh.say(string)
                                if target_detail:
                                    answer_question(self.sh, self.voice, self.target_detail)
                                else:
                                    self.start_gpsr()

                else:
                    if (self.target_object or 'object' in output) and self.target_location:
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
                        # Uncomment this to answer questions
                        answer_question(self.sh, self.voice, output)
if __name__ == '__main__':
    rospy.init_node("gpsr_speech_control", anonymous=True)
    ctrl = gpsr_speech_control()
    rospy.spin()                           





                

            

        







                
        




            





                    

        
                




        
    
        
    
