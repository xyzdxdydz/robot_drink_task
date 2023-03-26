#!/usr/bin/env python3
from your_package.srv import Info, InfoResponse
import rospy
def server_callback(req):
    name_dict = {
        "chaiyo" : "6210503543"
    }
    sum_id = sum([int(x) for x in name_dict[req.name]])
    print("Returning [%s => %s = %s]"%(req.name, name_dict[req.name], sum_id))
    return InfoResponse(sum_id)

def sum_server():
    rospy.init_node('my_server')
    s = rospy.Service('name_to_id', Info, server_callback)
    print("Ready to show the sum of id.")
    rospy.spin()
    
if __name__ == "__main__":
    sum_server()