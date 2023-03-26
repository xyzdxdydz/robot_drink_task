#!/usr/bin/env python3

import csv
import rospy
import actionlib
from actionlib_msgs.msg import *
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from your_package.msg import *
from your_package.srv import *
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse
import rospkg
import tf2_ros
from tf.transformations import euler_from_quaternion

robot_name = "basil"

class NavigationLib(object):
    def __init__(self):
        rospy.init_node("navigation", anonymous=True)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        print("Wait for movebase action")
        self.move_base.wait_for_server()
        r = rospkg.RosPack()
        # self.file_name = r.get_path("skuba_ahr_navigation")+"/csv/location.csv"
        self.file_name = "/home/yo/tutorial_ws/src/your_package/location.csv"
        self.nav_to_loc_ser = rospy.Service(f"/{robot_name}/nav/nav_to_location", NavToLocationCommand, self.nav_to_loc_callback)
        self.move_relative_ser = rospy.Service(f"/{robot_name}/nav/move_relative", MoveRelativeCommand, self.move_relative_callback)
        self.save_location_ser = rospy.Service(f"/{robot_name}/nav/save_location", SaveLocationCommand, self.save_location_callback)
        self.nav_to_pos_ser = rospy.Service(f"/{robot_name}/nav/nav_to_position", NavToPositionCommand, self.nav_to_pos_callback)
        self.goal_reach_ser = rospy.Service(f"/{robot_name}/nav/goal_reached", NavGoalReachedCommand, self.goal_reach_callback)
        self.goal_cancel_ser = rospy.Service(f"/{robot_name}/nav/goal_cancel", Empty, self.goal_cancel_callback)
        self.follow_pub = rospy.Publisher(f"/{robot_name}/nav/human_following", HumanFollowingCommand, queue_size=1)
        self.follow_sub = rospy.Subscriber("/basil_follower/cmd_vel", Twist,  self.human_follow_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(10)
        print("Server is ready!")  

    def goal_cancel_callback(self, data):
        self.move_base.cancel_all_goals()
        return EmptyResponse()

    # check if goal reached (timeout = 15 sec)
    def goal_reach_callback(self, data):
        res = NavGoalReachedCommandResponse()
        success = self.move_base.wait_for_result(rospy.Duration(15))
        state = self.move_base.get_state()
        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            res.goal_reached = True
        else:
            res.goal_reached = False
        return res 

    # sub the follower cmd_vel topic to see if the robot is moving and if it is then the robot is following therefore publish that robot is following 
    def human_follow_callback(self,data):
        vel_x = data.linear.x
        vel_y = data.linear.y
        vel_th = data.angular.z

        pub_data = HumanFollowingCommand()
        pub_data.is_people_follow = False

        if abs(vel_x) > 0 or abs(vel_y) > 0 or abs(vel_th) > 0:
            pub_data.is_people_follow = True
        
        self.follow_pub.publish(pub_data)
        self.rate.sleep()

    def move_navigate(self,frame,x,y,theta):
        q = quaternion_from_euler(0,0,theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))
        self.move_base.send_goal(goal)

    # go to location name
    def nav_to_loc_callback(self, data):
        location_name = data.location_name
        wait_for_success = data.wait_for_success

        response = NavToLocationCommandResponse()
        response.success = False

        self.go_to_position(location_name)
        if wait_for_success == False:
            print("if wait_for_success == False:")
            return response

        success = self.move_base.wait_for_result(rospy.Duration(15))
        state = self.move_base.get_state()

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            response.success = True

        return response

    def nav_to_pos_callback(self, data):
        target = data.target
        ref_frame = data.ref_frame
        wait_for_success = data.wait_for_success

        self.move_navigate(ref_frame, target.x, target.y, target.theta)

        response = NavToPositionCommandResponse()
        if wait_for_success == False:
            return response

        success = self.move_base.wait_for_result(rospy.Duration(15))
        state = self.move_base.get_state()

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            response.success = True

        return response

    # x is front/back
    # y is left/right positive y is left
    def move_relative_callback(self, data):
        target = data.target
        wait_for_success = data.wait_for_success

        self.move_navigate("base_footprint", target.x, target.y, -target.theta)

        response = MoveRelativeCommandResponse()
        if wait_for_success == False:
            return response

        success = self.move_base.wait_for_result(rospy.Duration(15))
        state = self.move_base.get_state()

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            response.success = True

        return response

    def read_csv(self):
        thisdict = {}
        with open(self.file_name, "r") as csv_file:
            csv_reader = csv.reader(csv_file,delimiter=',')
            for row in csv_reader:
                #print("row",row)
                # name, x, y, theta
                thisdict[row[0]] = [row[1], row[2], row[3]]
        return thisdict

    def read_position(self, position_name):
        dict_position = self.read_csv()
        print(dict_position[position_name])
        return dict_position[position_name]

    def go_to_position(self, position_name):
        x,y,theta = self.read_position(position_name)
        x,y,theta = float(x), float(y), float(theta)
        q = quaternion_from_euler(0,0,theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))
        print(goal)
        self.move_base.send_goal(goal)

    def save_location_callback(self, data):
        ret = self.save_position(data.location_name)
        if ret:
            return SaveLocationCommandResponse(True)
        return SaveLocationCommandResponse(False)
    
    def save_position(self, position_name):
        try:
            x,y,theta = self.get_position()
            thisdict = self.read_csv()
            thisdict[position_name] = [x,y,theta]
            thislist = []
            for location_name in thisdict:    
                thislist.append([location_name,thisdict.get(location_name)[0],thisdict.get(location_name)[1],thisdict.get(location_name)[2]])
            
            with open(self.file_name, "w") as csv_file:
                csv_writer = csv.writer(csv_file,delimiter=',')
                for line in thislist:
                    csv_writer.writerow(line)
            return True

        except Exception as e:
            print(e)
            return False

    def get_position(self):             #get the position of angus
        # self.listener = tf.TransformListener() 
        self.rate = rospy.Rate(10.0)
        get_position = False

        while not rospy.is_shutdown() and not get_position:
            try:
                trans = self.tfBuffer.lookup_transform("map", "base_footprint", rospy.Time())
                if trans != None:
                    get_position = True
                    print(trans)
        
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            self.rate.sleep()
        
        rot = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        euler = euler_from_quaternion(rot)
        return  trans.transform.translation.x, trans.transform.translation.y, euler[2]

if __name__ == "__main__":
    nav = NavigationLib()
    rospy.spin()