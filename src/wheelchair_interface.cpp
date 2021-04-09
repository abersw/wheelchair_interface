/*
 * wheelchair_interface.cpp
 * wheelchair_interface
 * version: 0.0.1 Majestic Maidenhair
 * Status: Pre-Alpha
 * 
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include "ros/ros.h" //main ROS library
#include "std_msgs/String.h"
using namespace std;

ros::Publisher *ptr_publish_espeak;

/**
 * Function to shutdown ROS node safely
 */
void shutdownROSnode() {
    cout << "shutting down ROS node" << endl;
    ros::shutdown();
    exit(0);
}

/**
 * Function to return user input when prompted to enter the room name
 *
 * @return getUserInstructionRaw - returns direct input from user
 */
std::string requestUserInput() {
    //notify user via interface and speech
    std::string instruction = "Where would you like to go"; //set question for user interface
    cout << instruction << "?" << endl; //print question to interface

    std_msgs::String espeak_msg; //initialise espak ROS msg
    espeak_msg.data = instruction; //assign user question to espeak data
    ptr_publish_espeak->publish(espeak_msg); //publish espeak msg

    std::string getUserInstructionRaw; //initialise variable for user input
    getline(std::cin, getUserInstructionRaw); //read line from user interface
    transform(getUserInstructionRaw.begin(), getUserInstructionRaw.end(), getUserInstructionRaw.begin(), ::tolower); //transform string to lower case letters

    return getUserInstructionRaw; //return user input
}


int main(int argc, char * argv[]) {

    ros::init(argc, argv, "wheelchair_interface");
    ros::NodeHandle nodeHandle;
    //publish user input to wheelchair_navigation node
    ros::Publisher espeak_pub = nodeHandle.advertise<std_msgs::String>("/espeak_node/speak_line", 1000); //publisher to espeak (voice) node
    ptr_publish_espeak = &espeak_pub;
    ros::Rate loop_rate(10);

    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
        //++count;
    }
    return 0;
}
