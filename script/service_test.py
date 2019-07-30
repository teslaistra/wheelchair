#!/usr/bin/env python

import rospy
import time
from wheelchair.srv import *

if __name__ == '__main__':

    print_service = rospy.ServiceProxy('/Print_service', batLevel)

    while not rospy.is_shutdown():

        res = print_service()
        print (res)