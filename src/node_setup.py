#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_srvs.srv import *
from your_package.srv import *
from state_mockup import *
from utils import *

from gtts import gTTS
import os
import speech_recognition as sr

class RobotSkillSetup(object):
    def __init__(self) -> None:
        NODE_NAME = 'drink_robot'
        rospy.init_node(NODE_NAME)
        self.service_go = rospy.Service(f'{NODE_NAME}/go', GoInterface, self.go)
        self.service_talk = rospy.Service(f'{NODE_NAME}/talk', TalkInterface, self.talk)
        self.service_listen = rospy.Service(f'{NODE_NAME}/listen', ListenInterface, self.listen)
        self.service_move = rospy.Service(f'{NODE_NAME}/move', MoveInterface, self.move)

        self.utils = Utils()
        
        self.idle = True
        self.is_speaking = False     
        self.drink = None
        self.move = rospy.ServiceProxy('/basil/nav/nav_to_location', NavToLocationCommand)

        print("my turtlebot is ready !")
        rospy.spin()  
        
    def go(self, req):
        if req.method == "get":
            return GoInterfaceResponse(self.idle)

        elif req.method == "set":
            self.idle = req.idle
            if not self.idle:
                print("my robot go brrr...")
            return GoInterfaceResponse(self.idle)
            
        return Empty
    
    def talk(self, req):
        print("my robot is talking, wow!")
        # document = '''
        #         สวัสดีงับ ผมเป็นturtlebotที่โคตรฉลาด
        #         วันนี้ คุณอยากดื่มอะไรเป็นพิเศษมั้ย งับ 
        #         ถ้านึกไม่ออก ก็แดกส้นตีนแทนได้นะงับ งุงิ
        #     '''
        document = "ดื่มไร"
        
        self.utils.speak("asking", document)

        return TalkInterfaceResponse(True)
    
    def listen(self, req):
        print("my robot is listeining, how can dey do dat")
        r = sr.Recognizer()
        text = None
        
        if req.method == "listen":
            while(1):
            # for i in range(1):
                try:
                    with sr.Microphone() as source2:
                        print(">>>>>>>>>>>>>> Initializing .....")
                        r.adjust_for_ambient_noise(source2, duration=2)  
                        print(">>>>>>>>>>>>>> Listing .....")
                        
                        # listens for the user's input
                        audio2 = r.listen(source2, phrase_time_limit=4.0)
                        # audio2 = r.listen(source2)
                        
                        # Using google to recognize audio
                        MyText = r.recognize_google(audio2)
                        MyText = MyText.lower()
                        self.drink = MyText
                        print(MyText)
                        
                        if self.drink is not None:
                            break

                except sr.RequestError as e:
                    print("Could not request results; {0}".format(e))
                    
                except sr.UnknownValueError:
                    print("unknown error occurred")
                
                except sr.WaitTimeoutError:
                    print("Timed out, abort.")
                
            print("Final text:", self.drink)
            return ListenInterfaceResponse(True, "Done !")

        elif req.method == "get":
            if self.drink is not None:
                return ListenInterfaceResponse(True, self.drink)
        
            return ListenInterfaceResponse(False, 'Error !')
            
        elif req.method == "forget":
            self.drink = None
            return ListenInterfaceResponse(True, "Done !")
    
    def move(self, req):
        print("my robot is moving ! holy cow")
        isArrived = False
        while not isArrived:
            move_response = self.move(req.location_name, True)
            isArrived = move_response.success
            
        return MoveInterfaceResponse(isArrived)
    
    
if __name__ == "__main__":
    RobotSkillSetup()