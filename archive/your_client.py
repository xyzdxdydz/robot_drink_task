#!/usr/bin/env python3
from your_package.srv import Sum, SumResponse
import rospy

def user_sum():
    rospy.wait_for_service('sum')
    try:
        sum_t = rospy.ServiceProxy('sum', Sum)
        print("Please do sum for us.")
        respl = sum_t(555,1)
        print(respl)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    user_sum()