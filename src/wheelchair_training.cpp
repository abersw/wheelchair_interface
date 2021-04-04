/*
 * wheelchair_training.cpp
 * wheelchair_interface
 * version: 0.0.1 Majestic Maidenhair
 * Status: Pre-Alpha
 * 
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ros/ros.h" //main ROS library
#include "std_msgs/String.h"


#include <sstream>
using namespace std;

const bool DEBUG_requestUserInput = 0;
const bool DEBUG_main = 0;

ros::Publisher *ptr_publish_espeak;
ros::Publisher *ptr_publish_roomName;

int wheelchair_interface_state = 1;
std::string userInstructionRaw;

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
    std::string instruction = "What's the room name"; //set question for user interface
    cout << instruction << endl; //print question to interface

    std_msgs::String espeak_msg; //initialise espak ROS msg
    espeak_msg.data = instruction; //assign user question to espeak data
    ptr_publish_espeak->publish(espeak_msg); //publish espeak msg

    std::string getUserInstructionRaw; //initialise variable for user input
    getline(std::cin, getUserInstructionRaw); //read line from user interface
    transform(getUserInstructionRaw.begin(), getUserInstructionRaw.end(), getUserInstructionRaw.begin(), ::tolower); //transform string to lower case letters

    return getUserInstructionRaw; //return user input
}

void publishRoomName() {
    std_msgs::String roomNameMsg;
    roomNameMsg.data = userInstructionRaw;

    ptr_publish_roomName->publish(roomNameMsg);
}


int main(int argc, char * argv[]) {

    ros::init(argc, argv, "wheelchair_training_interface");
    ros::NodeHandle nodeHandle;

    ros::Publisher espeak_pub = nodeHandle.advertise<std_msgs::String>("/espeak_node/speak_line", 1000); //publisher to espeak (voice) node
    ros::Publisher roomName_pub = nodeHandle.advertise<std_msgs::String>("/wheelchair_robot/user/room_name", 10); //publisher to assign room name
    ptr_publish_espeak = &espeak_pub;
    ptr_publish_roomName = &roomName_pub;
    ros::Rate loop_rate(10);

    while (ros::ok()) {

        switch(wheelchair_interface_state) {
            case 0: 
                shutdownROSnode();
                break;
            case 1:
                //get user instruction
                userInstructionRaw = requestUserInput();
                if (userInstructionRaw != "") {
                    wheelchair_interface_state = 2; //request is not blank 
                }
                break;
            case 2:
                cout << userInstructionRaw << endl;
                publishRoomName();
                wheelchair_interface_state = 1;
                break;
        }
        
        ros::spinOnce();
        if (DEBUG_main) {
            cout << "ROS spinned" << endl;
        }
    }
    return 0;
}
