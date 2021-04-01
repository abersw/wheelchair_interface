/*
 * To-do list:
 * 
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h>
//#include <getopt.h>
//#include <iostream> //io library

#include "ros/ros.h" //main ROS library
#include "std_msgs/String.h"
#include "move_base_msgs/MoveBaseActionGoal.h" //move base msg for sending map goals

#include <fstream> //for writing and reading files in c++
#include <ros/package.h> //find ROS packages, needs roslib dependency

#include <sstream>
using namespace std;

//The wheelchair interface is written by Tomos Fearn (tof7@aber.ac.uk)
//The program is in:
std::string softwareVersion = "Version 0.1 - Draft";

const bool DEBUG_main = 1;

int wheelchair_interface_state = 1;

void shutdownROSnode() {
    cout << "shutting down ROS node" << endl;
    ros::shutdown();
    exit(0);
}


int main(int argc, char * argv[]) {

    ros::init(argc, argv, "wheelchair_training");
    ros::NodeHandle nodeHandle;

    ros::Publisher wheelchairGoal_pub = nodeHandle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
    ros::Publisher espeak_pub = nodeHandle.advertise<std_msgs::String>("/espeak_node/speak_line", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok()) {

        switch(wheelchair_interface_state) {
            case 0: 
                shutdownROSnode();
                break;
            case 1:
                //get user instruction
                cout << "jump into this function" << endl;
                break;
        }
        
        ros::spinOnce();
        if (DEBUG_main) {
            cout << "ROS spinned" << endl;
        }
    }
    return 0;
}
