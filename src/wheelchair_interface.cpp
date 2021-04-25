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
#include "wheelchair_msgs/roomLocations.h"
#include "std_msgs/String.h"
using namespace std;

const bool DEBUG_roomLocationsCallback = 0;
const bool DEBUG_requestUserInput = 0;
const bool DEBUG_main = 0;

ros::Publisher *ptr_publish_espeak;
ros::Publisher *ptr_publish_userInstruction;

struct Rooms {
    int room_id;
    string room_name;

    float point_x; //get transform point x
    float point_y; //get transform point y
    float point_z; //get transform point z

    float quat_x; //get transform rotation quaternion x
    float quat_y; //get transform rotation quaternion y
    float quat_z; //get transform rotation quaternion z
    float quat_w; //get transform rotation quaternion w
};
struct Rooms roomsFileStruct[1000];
int totalRoomsFileStruct = 0;

int wheelchair_interface_state = 1;
int roomsFound = 0;
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
 * Function to check if more than one room is supplied from user instruction
 * set roomsFound var from found room name matches
 */
void detectNumOfRooms() {
    int roomCount = 0;
    for (int isRoom = 0; isRoom < totalRoomsFileStruct; isRoom++) {
        std::string getRoomName = roomsFileStruct[isRoom].room_name; //get room name from struct
        std::size_t foundRoomMatch = userInstructionRaw.find(getRoomName); //search for corresponding room name
        if (foundRoomMatch != std::string::npos) { //if match is found
            roomCount++; //found match, added to count var
        }
    }
    roomsFound = roomCount; //set found matches to var
}

/**
 * Function to publish the user instruction as ROS msg
 */
void publishUserInstruction() {
    std_msgs::String userInsMsg;
    userInsMsg.data = userInstructionRaw;

    ptr_publish_userInstruction->publish(userInsMsg);
}

/**
 * Callback function triggered by list of all rooms 
 *
 * @param parameter 'roomLoc' is a roomLocations msg of the rooms and associated transforms from wheelchair_dacop
 *        message belongs to wheelchair_msgs::roomLocations
 */
void roomLocationsCallback(const wheelchair_msgs::roomLocations roomLoc) {
    totalRoomsFileStruct = roomLoc.totalRooms;
    for (int isRoom = 0; isRoom < totalRoomsFileStruct; isRoom++) {
        roomsFileStruct[isRoom].room_id = roomLoc.id[isRoom];
        roomsFileStruct[isRoom].room_name = roomLoc.room_name[isRoom];

        roomsFileStruct[isRoom].point_x = roomLoc.point_x[isRoom];
        roomsFileStruct[isRoom].point_y = roomLoc.point_y[isRoom];
        roomsFileStruct[isRoom].point_z = roomLoc.point_z[isRoom];

        roomsFileStruct[isRoom].quat_x = roomLoc.quat_x[isRoom];
        roomsFileStruct[isRoom].quat_y = roomLoc.quat_y[isRoom];
        roomsFileStruct[isRoom].quat_z = roomLoc.quat_z[isRoom];
        roomsFileStruct[isRoom].quat_w = roomLoc.quat_w[isRoom];

        if (DEBUG_roomLocationsCallback) {
            cout << 
            roomsFileStruct[isRoom].room_id << ", " << 
            roomsFileStruct[isRoom].room_name << ", " << 
            
            roomsFileStruct[isRoom].point_x << ", " << 
            roomsFileStruct[isRoom].point_y << ", " << 
            roomsFileStruct[isRoom].point_z << ", " << 

            roomsFileStruct[isRoom].quat_x << ", " << 
            roomsFileStruct[isRoom].quat_y << ", " << 
            roomsFileStruct[isRoom].quat_z << ", " << 
            roomsFileStruct[isRoom].quat_w << endl;
        }
    }
}

/**
 * Main function publishes espeak node and room name topics
 *
 * @return 0 - shouldn't reach this part unless shutting down
 */
int main(int argc, char * argv[]) {

    ros::init(argc, argv, "wheelchair_interface");
    ros::NodeHandle nodeHandle;
    //publish user input to wheelchair_navigation node
    ros::Publisher espeak_pub = nodeHandle.advertise<std_msgs::String>("/espeak_node/speak_line", 1000); //publisher to espeak (voice) node
    ros::Publisher userInstruction_pub = nodeHandle.advertise<std_msgs::String>("/wheelchair_robot/user/instruction", 1000);
    ros::Subscriber room_locations_sub = nodeHandle.subscribe("/wheelchair_robot/dacop/assign_room_to_object/rooms", 10, roomLocationsCallback);
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
                detectNumOfRooms();
                if (roomsFound < 2) { //if 0 or 1 room match found
                    publishUserInstruction(); //publish user instruction as ROS topic
                }
                else {
                    //found more than one room
                    std::string errorMsg = "instruction not sent, please enter only one room name";
                    cout << errorMsg << endl; //print out error to user

                    std_msgs::String espeak_msg; //initialise espak ROS msg
                    espeak_msg.data = errorMsg; //assign user question to espeak data
                    ptr_publish_espeak->publish(espeak_msg); //publish espeak msg
                }
                wheelchair_interface_state = 1; //wait for user instruction
                break;
        }

        ros::spinOnce();
        //loop_rate.sleep();
        //++count;
    }
    return 0;
}
