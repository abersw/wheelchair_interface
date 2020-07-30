#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h>
//#include <getopt.h>
//#include <iostream> //io library

#include "ros/ros.h" //main ROS library
#include "std_msgs/String.h"

#include <fstream> //for writing and reading files in c++
#include <ros/package.h> //find ROS packages, needs roslib dependency

#include <sstream>
using namespace std;

//Debugging lines
int DEBUG_CALCULATE_LINES = 0;
int DEBUG_ROOM_LIST_TO_STRUCT = 0;

FILE *filePointer; //pointer for file reader/writer

std::string objectsFileLoc; //variable for storing objects file location
std::string weightingFileLoc; //variable for storing the first section of weighting file location
std::string roomListLoc; //variable for storing room list location
std::string mobilenetFileType = ".objects"; //file extention for mobilenet type
std::string weightingFileType = ".weights"; //file extention for training type

//variables and arrays for storing objects from training file
std::string softwareVersion = "Version 0.1 - Draft";

//contains list of rooms
struct Rooms {
	int id;
	std::string roomName;
	int timesTrained;
	int totalObjects;
};

struct Rooms room[10000]; //list of rooms
int totalRooms = 0;



//function for printing space sizes
void printSeparator(int spaceSize) {
	if (spaceSize == 0) {
		printf("--------------------------------------------\n");
	}
	else {
		printf("\n");
		printf("--------------------------------------------\n");
		printf("\n");
	}
}

//calculate lines from files
int calculateLines(std::string fileName) {
	printf("DEBUG: calculateLines()\n");
	ifstream FILE_COUNTER(fileName);
	std::string getlines;
	int returnCounter = 0;
	while (getline (FILE_COUNTER, getlines)) {
		returnCounter++;
  		// Output the text from the file
  		if (DEBUG_CALCULATE_LINES == 1) {
	  		cout << getlines;
  			cout << "\n";
  		}
	}
	FILE_COUNTER.close();
	return returnCounter;
}

//does the wheelchair dump package exist in the workspace?
void doesWheelchairDumpPkgExist() {
	if (ros::package::getPath("wheelchair_dump") == "") {
		cout << "FATAL:  Couldn't find package 'wheelchair_dump' \n";
		cout << "FATAL:  Closing training_context node. \n";
		printSeparator(1);
		ros::shutdown();
		exit(0);
	}
}

//get list of rooms and save to struct
void roomListToStruct(std::string fileName) {
	printf("DEBUG: roomListToStruct()\n");
	std::string roomsDelimiter = ":";
	ifstream FILE_READER(fileName);
	std::string line;
	int roomNumber = 0;
	while (getline(FILE_READER, line)) {
		int delimiterPos = 0;
		std::string getRoomName; //temporary room name
		int getRoomId; //temporary room id
		getRoomName = line.substr(0, line.find(roomsDelimiter)); //string between 0 and delimiter
		room[roomNumber].roomName = getRoomName; //set room name
		getRoomId = std::stoi(line.substr(line.find(roomsDelimiter) +1)); //get room id
		room[roomNumber].id = getRoomId; //set room id
		if (DEBUG_ROOM_LIST_TO_STRUCT == 1) {
			cout << getRoomName; //print room name
			cout << getRoomId << "\n"; //print room id
		}
		roomNumber++; //iterate to next room
	}
	FILE_READER.close();
}

int main(int argc, char * argv[]) {

    ros::init(argc, argv, "wheelchair_interface");
    ros::NodeHandle nodeHandle;

    ros::Publisher chatter_pub = nodeHandle.advertise<std_msgs::String>("wheelchair_goal", 1000);
    ros::Rate loop_rate(10);

    doesWheelchairDumpPkgExist(); //check to see if dump package exists
    std::string wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
    roomListLoc = wheelchair_dump_loc + "/dump/context_training/room.list";

    roomListToStruct(roomListLoc);
    totalRooms = calculateLines(roomListLoc);
    for (int i = 0; i < totalRooms; i++) {
    	cout << room[i].roomName << "\n";
    }

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

        //std_msgs::String msg;

        //std::stringstream ss;
        //ss << "hello world " << count;
        //msg.data = ss.str();

        //ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
        */
        //chatter_pub.publish(msg);

        ros::spinOnce();

        //loop_rate.sleep();
        //++count;
    }
    return 0;
}
