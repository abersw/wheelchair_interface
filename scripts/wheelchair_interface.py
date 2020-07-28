#!/usr/bin/env python

import rospy
import os
import sys
import rospkg
from std_msgs.msg import String

publishGoal = ''
rate = ''

#userInstruction = "take me to the refrigerator"
#userInstruction = "I want to go to the table"
#userInstruction = "the oven"
userInstruction = ''

#def callback(userInstructionData):
    #rospy.loginfo(rospy.get_caller_id() + " I heard %s", userInstructionData.data)
    #if (userInstruction.find("me to the") != -1):
    #	rospy.loginfo("found a match")
    #else:
    #	rospy.loginfo("no match")

#    userInstruction = userInstructionData.data

#    rospy.loginfo(userInstruction)
    #publishGoal.publish(hello_str)
    #rate.sleep()

def interfaceNode():
    publishGoal = rospy.Publisher('rtabmapcoordinatesfrominterface', String, queue_size=10)
    subscribeInstruction = rospy.Subscriber("userinstructions", String, callback)
    rospy.init_node('wheelchair_interface', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #print("Please add an instruction")
        #instruction = input()
        #print(f'you entered {instruction}')
        
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #publishGoal.publish(hello_str)
        rospy.loginfo(userInstruction)
        rate.sleep()
        

if __name__ == "__main__":
    try:
        interfaceNode()
    except rospy.ROSInterruptException:
        pass
