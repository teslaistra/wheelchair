#!/usr/bin/env python

import rospy
import time
from wheelchair.srv import *

if __name__ == '__main__':
    node = rospy.init_node('test_node')
    print_service = rospy.ServiceProxy('/Print_service', batLevel)
    request = batLevelRequest()
    request.message.data = ""

    while not rospy.is_shutdown():
        ##
        res = print_service(request)
        print (res.level.data)
        time.sleep(1)