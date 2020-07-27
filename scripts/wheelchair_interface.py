#!/usr/bin/env python

import rospy
import os
import sys
from std_msgs.msg import String


def interfaceNode():
    pub = rospy.Publisher('rtabmapcoordinatesfrominterface', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #print("Please add an instruction")
        #instruction = input()
        #print(f'you entered {instruction}')
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == "__main__":
    try:
        interfaceNode()
    except rospy.ROSInterruptException:
        pass
