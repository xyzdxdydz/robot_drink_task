#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_srvs.srv import *
from your_package.srv import *
from state_mockup import *
from node_setup import *
from utils import *
import time

NODE_NAME = 'drink_robot'

class DoSth(smach.State):
    def __init__(self, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        self.dosth_ser = rospy.ServiceProxy('robot/do_sth', Empty)
    
    def execute(self, ud):
        rospy.loginfo("Executing state DoSth")
        self.dosth_ser()
        return 'success'

class GoToPosition(smach.State):
    def __init__(self, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        self.gotopos = rospy.ServiceProxy('robot/go_to_position', GoToPositionResponse)
    
    def execute(self, ud):
        rospy.loginfo("Executing state GoToPosition")
        req = GoToPositionResponse()
        req.position_name = 'home'
        self.gotopos(req)
        return 'success' 
    
class Idle(smach.State):
    def __init__(self, outcomes=['complete', 'nothing']):
        super().__init__(outcomes)
        self.req_idle = rospy.ServiceProxy(f'{NODE_NAME}/go', GoInterface)
        self.rest = False
    
    def execute(self, ud):
        print("state:", "Idle")
        time.sleep(0.5)
        if self.rest:
            req_for_stop = self.req_idle("set", True)
            self.rest = False
            
        req_for_start = self.req_idle("get", True)
        if not req_for_start.current_idle_status:
            self.rest = True
            return 'complete'
        
        return 'nothing'
    
class WalkingToGuest(smach.State):
    def __init__(self, outcomes=['complete', 'abort']):
        super().__init__(outcomes)
        self.req_for_move = rospy.ServiceProxy(f'{NODE_NAME}/move', MoveInterface)
    
    def execute(self, ud):
        print("state:", "walk to guest")
        move_response = self.req_for_move("guest")
        
        # if move_response.result:
        print(move_response.result)
        return 'complete' 

class ToGuest(smach.State):
    def __init__(self, outcomes=['complete',]):
        super().__init__(outcomes)
    
    def execute(self, ud):
        print("state:", "arrived")
        
        return 'complete' 
   
class Asking(smach.State):
    def __init__(self, outcomes=['complete', 'abort']):
        super().__init__(outcomes)
        self.req_talk = rospy.ServiceProxy(f'{NODE_NAME}/talk', TalkInterface)
        
    def execute(self, ud):
        print("state:", "Talking")
        talk_status = self.req_talk()
    
        if talk_status.result: 
            return 'complete'
        
        return 'abort'

class Listening(smach.State):
    def __init__(self, outcomes=['complete', 'abort']):
        super().__init__(outcomes)
        self.utils = Utils()
        self.req_listen = rospy.ServiceProxy(f'{NODE_NAME}/listen', ListenInterface)
    
    def execute(self, ud):
        print("state:", "listen")
        listen_status = self.req_listen("listen")

        if listen_status.result: 
            self.utils.speak(
                "ack",
                'ขี้เกียจค่ะ แต่จะพยายามไป'
            )
            return 'complete'
        
        return 'abort'

class WalkToHost(smach.State):
    def __init__(self, outcomes=['complete']):
        super().__init__(outcomes)
        self.req_for_move = rospy.ServiceProxy(f'{NODE_NAME}/move', MoveInterface)
    
    def execute(self, ud):
        print("state:", "walk to host")
        move_response = self.req_for_move("host")
        
        # if move_response.result:
        print("Arri", move_response.result)
        return 'complete' 
    
class TellHost(smach.State):
    def __init__(self, outcomes=['complete', ]):
        super().__init__(outcomes)
        self.req_drink = rospy.ServiceProxy(f'{NODE_NAME}/listen', ListenInterface)
        self.utils = Utils()
        
    def execute(self, ud):
        print("state:", "telling host")
        time.sleep(0.5)
        drinkName = self.req_drink("get")
        
        if drinkName.result:
            self.utils.speak(
                "tell_host",
                f'เขาจะดื่ม {drinkName.message}'
            )
            self.req_drink("forget")
            
        else:
            print("nothing...")
            
        return 'complete' 
 
class MainPhase(object):
    def __init__(self) -> None:
        sm = smach.StateMachine(outcomes=['---finish---'])

        with sm:
            smach.StateMachine.add('Idle', Idle(), 
            transitions={
                'complete':'WalkingToGuest',
                'nothing': 'Idle',
            })
            smach.StateMachine.add('WalkingToGuest', WalkingToGuest(), 
            transitions={
                'complete':'ToGuest',
                'abort': 'WalkingToGuest',
            })
            smach.StateMachine.add('ToGuest', ToGuest(), 
            transitions={
                'complete':'Asking',
                    
            })
            smach.StateMachine.add('Asking', Asking(), 
            transitions={
                'complete':'Listening',
                'abort': 'WalkToHost',
            })
            smach.StateMachine.add('Listening', Listening(), 
            transitions={
                'complete':'WalkToHost',
                'abort': 'WalkToHost',
            })
            smach.StateMachine.add('WalkToHost', WalkToHost(), 
            transitions={
                'complete':'SayToHost',
                    
            })
            smach.StateMachine.add('SayToHost', TellHost(), 
            transitions={
                'complete':'Idle',
                    
            })
            
        outcome = sm.execute()

if __name__ == "__main__":
    MainPhase()