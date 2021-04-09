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

const bool DEBUG_requestUserInput = 0;
const bool DEBUG_main = 0;

ros::Publisher *ptr_publish_espeak;
ros::Publisher *ptr_publish_userInstruction;

int wheelchair_interface_state = 1;
std::string userInstructionRaw;

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
    if (DEBUG_requestUserInput) {
        cout << "user input is " << getUserInstructionRaw << endl; //return user input
    }
    return getUserInstructionRaw; //return user input
}

/**
 * Function to publish the user instruction as ROS msg
 */
void publishUserInstruction() {
    std_msgs::String userInsMsg;
    userInsMsg.data = userInstructionRaw;

    ptr_publish_userInstruction->publish(userInsMsg);
}

int main(int argc, char * argv[]) {

    ros::init(argc, argv, "wheelchair_interface");
    ros::NodeHandle nodeHandle;
    //publish user input to wheelchair_navigation node
    ros::Publisher espeak_pub = nodeHandle.advertise<std_msgs::String>("/espeak_node/speak_line", 1000); //publisher to espeak (voice) node
    ros::Publisher userInstruction_pub = nodeHandle.advertise<std_msgs::String>("/wheelchair_robot/user/instruction", 1000);
    ptr_publish_espeak = &espeak_pub;
    ptr_publish_userInstruction = &userInstruction_pub;
    ros::Rate loop_rate(10);

    while (ros::ok()) {

        switch(wheelchair_interface_state) {
            case 0: 
                shutdownROSnode();
                break;
            case 1:
                //get user instruction
                userInstructionRaw = requestUserInput(); //request the user's input
                if (userInstructionRaw != "") { //if user instruction is not blank
                    if (userInstructionRaw == "quit") {
                        wheelchair_interface_state = 0;
                    }
                    else {
                        wheelchair_interface_state = 2; //go to state 2 for publishing room name
                    }
                }
                break;
            case 2:
                if (DEBUG_main) {
                    cout << userInstructionRaw << endl; //return room name
                }
                publishUserInstruction(); //publish user instruction as ROS topic
                wheelchair_interface_state = 1;
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
        //++count;
    }
    return 0;
}
