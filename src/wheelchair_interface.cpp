#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h>
//#include <getopt.h>
//#include <iostream> //io library
#include <fstream> //for writing and reading files in c++
#include <ros/package.h> //find ROS packages, needs roslib dependency

#include "ros/ros.h" //main ROS library
#include "std_msgs/String.h"

#include <sstream>
using namespace std;

FILE *filePointer; //pointer for file reader/writer

std::string objectsFileLoc; //variable for storing objects file location
std::string weightingFileLoc; //variable for storing the first section of weighting file location
std::string roomListLoc; //variable for storing room list location
std::string mobilenetFileType = ".objects"; //file extention for mobilenet type
std::string weightingFileType = ".weights"; //file extention for training type

//variables and arrays for storing objects from training file
std::string softwareVersion = "Version 0.1 - Draft";

int main(int argc, char * argv[]) {

    ros::init(argc, argv, "wheelchair_interface");
    ros::NodeHandle nodeHandle;

    ros::Publisher chatter_pub = nodeHandle.advertise<std_msgs::String>("wheelchair_goal", 1000);
    ros::Rate loop_rate(10);



    int count = 0;
    while (ros::ok()) {
        //get string message from user
  	cout << "Where would you like to go?\n";
  	//string userInstruction;
  	std::string userInstruction;
  	getline(std::cin, userInstruction);
  	//std_msgs::String userInstructionROS = std_msgs::String.userInstruction.c_str();
  	ROS_INFO_STREAM("MSG: " << userInstruction);
  	//cout << userInstruction << "\n";

        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
        */
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
}
