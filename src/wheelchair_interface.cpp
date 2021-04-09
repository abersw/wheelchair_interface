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
using namespace std;

/**
 * Function to shutdown ROS node safely
 */
void shutdownROSnode() {
    cout << "shutting down ROS node" << endl;
    ros::shutdown();
    exit(0);
}


int main(int argc, char * argv[]) {

    ros::init(argc, argv, "wheelchair_interface");
    ros::NodeHandle nodeHandle;
    //publish user input to wheelchair_navigation node
    ros::Rate loop_rate(10);

    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
        //++count;
    }
    return 0;
}
