#import roslib
#import os, sys
#import rospy, rospkg
#from std_msgs.msg import String
#from std_msgs.msg import Float32

def main():
    print("Hello World!")
    print("------------")

    print("Please add an instruction")
    instruction = input()
    print(f'you entered {instruction}')

if __name__ == "__main__":
    main()
