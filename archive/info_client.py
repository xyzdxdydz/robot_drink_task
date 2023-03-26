#!/usr/bin/env python3
from your_package.srv import Info, InfoResponse
import rospy

def user_sum():
    rospy.wait_for_service('name_to_id')
    try:
        sum_id = rospy.ServiceProxy('name_to_id', Info)
        print("Please sum an id of the name for us.")
        respl = sum_id(sys.argv[0])
        print("Wow! ::", respl)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    user_sum()