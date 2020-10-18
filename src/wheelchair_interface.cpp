/*
 * To-do list:
 * Sentence splitting
 * calculate rooms and object to navigate to
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

//Debugging lines
int DEBUG_CALCULATE_LINES = 0;
int DEBUG_ROOM_LIST_TO_STRUCT = 0;
int DEBUG_TRAINING_FILES_TO_STRUCT = 0;
int DEBUG_FIND_ROOM_MATCHES = 1;
int DEBUG_FIND_OBJECT_MATCHES = 1;

FILE *filePointer; //pointer for file reader/writer

std::string objectsFileLoc; //variable for storing objects file location
std::string weightingFileLoc; //variable for storing the first section of weighting file location
std::string roomListLoc; //variable for storing room list location
std::string mobilenetFileType = ".objects"; //file extention for mobilenet type
std::string weightingFileType = ".weights"; //file extention for training type

int questionState = 0; // 0 - ready for next question, 1 found match, 2 follow up question

//variables and arrays for storing objects from training file
int wheelchair_interface_state = 1;
std::string userInstructionRaw;
std::string userInstruction[5];
int printOnce = 1;


//contains list of rooms
struct Rooms {
    int id;
    std::string roomName;
    int timesTrained;
    int totalObjects;
};

//contains blueprint for training objects
struct Training {
    std::string objectName;
    double objectWeighting;
    int alreadyExists;
    double uniqueness;
};

struct NavigateToDecision {
    std::string roomName;
    std::string objectName;
    int objectWeighting;
    int uniqueness;
    int score;
    int id;

};

struct Rooms room[10000]; //list of rooms
//roomId followed by objects list
struct Training preTrained[1000][10000]; //saves items from file to struct
struct Training trained[1000][10000]; //struct for writing back to files
struct NavigateToDecision decisionListRooms[1000];
struct NavigateToDecision decisionListObjects[10000];
struct NavigateToDecision navigateToDecision[1];
int totalRooms = 0;

int decisionListRoomsTotal = 0;
int decisionListObjectsTotal = 0;



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
    if (DEBUG_CALCULATE_LINES == 1) {
        printf("DEBUG: calculateLines()\n");
    }
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
    if (DEBUG_ROOM_LIST_TO_STRUCT == 1) {
        printf("DEBUG: roomListToStruct()\n");
    }
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

void readTrainingFile(std::string fileName, int roomIdParam) {
    if (DEBUG_TRAINING_FILES_TO_STRUCT == 1) {
        printSeparator(0);
        printf("DEBUG: readTrainingFile()\n");
    }
    ofstream FILE_WRITER; //declare write file
    ifstream FILE_READER; //declare read file
    FILE_READER.open(fileName);
    /*if (FILE_READER.peek() == std::ifstream::traits_type::eof()) { //peek to see if file is empty
        cout << "weighting file is empty, starting to populate data. \n";
        FILE_READER.close();//closed for peeking
        FILE_WRITER.open(fileName); //open write file
        FILE_WRITER << roomNameROSParam << "\n";
        FILE_WRITER << 0; //first time training
        FILE_WRITER.close(); //close write file
        FILE_READER.open(fileName); //reopen file after peek
    }*/
    std::string line;
    int lineNumber = 0;
    int objectNumber = 0;
    while (getline(FILE_READER, line)) {
        if (lineNumber == 0) { //if line number is 0 - i.e. room name
            //do nothing room name
            if (DEBUG_TRAINING_FILES_TO_STRUCT == 1) {
                cout << "reading Room Name: " << line << "\n";
            }
        }
        else if (lineNumber == 1) { //if line number is 1 - i.e. training times
            //get times trained
            std::string getTimesTrainedString = line;
            int getTimesTrained = ::atoi(line.c_str()); //cast times trained string to int
            getTimesTrained++;
            room[roomIdParam].timesTrained = getTimesTrained; //set times trained to correponding room
            if (DEBUG_TRAINING_FILES_TO_STRUCT == 1) {
                cout << "reading Times Trained: " << room[roomIdParam].timesTrained << "\n";
            }
        }
        else if (lineNumber > 1) { //rest of the lines are trained objects
            //find delimiter positions
            std::string delimiter = ":"; //look for colon 
            int delimiterPos[5]; //set array of delimiter positions
            int delimiterNumber = 0; //current delimiter
            int lineLength = line.length(); //get length of line
            char lineArray[lineLength + 1]; //create array of chars
            strcpy(lineArray, line.c_str()); //set string to chars
            for (int charPos = 0; charPos < lineLength; charPos++) {
                if (lineArray[charPos] == ':') { //if char is colon
                    //printf("%c\n", lineArray[i]);
                    //printf("found delimiter\n");
                    delimiterPos[delimiterNumber] = charPos; //add position of delimiter to array
                    delimiterNumber++; //iterate to next delimiter
                }
            }
            //extract substrings between delimiters
            for (int section = 0; section < delimiterNumber +1; section++) { //go through line at each delimiter position
                if (section == 0) {
                    preTrained[roomIdParam][objectNumber].objectName = line.substr(0, delimiterPos[0]); //set first substring to pretrained struct
                    if (DEBUG_TRAINING_FILES_TO_STRUCT == 1) {
                        cout << "object number is " << objectNumber << "\n"; 
                        cout << "preTrained objectname is: " + preTrained[roomIdParam][objectNumber].objectName + "\n";
                    }
                }
                else if (section == 1) {
                    double weightingToDouble = std::atof(line.substr(delimiterPos[0] + 1, delimiterPos[1]).c_str()); //cast weighting from string to double
                    preTrained[roomIdParam][objectNumber].objectWeighting = weightingToDouble; //set second substring to pretrained struct and cast to double
                    if (DEBUG_TRAINING_FILES_TO_STRUCT == 1) {
                        cout << "preTrained objectWeighting is: " << preTrained[roomIdParam][objectNumber].objectWeighting << "\n";
                    }
                }
                else if (section == 2) {
                    double uniquenessToDouble = std::atof(line.substr(delimiterPos[1] + 1).c_str()); //cast uniqueness from string to double
                    preTrained[roomIdParam][objectNumber].uniqueness = uniquenessToDouble; //set third substring to pretrained struct and cast to double
                    if (DEBUG_TRAINING_FILES_TO_STRUCT == 1) {
                        cout << "preTrained uniqueness is: " << preTrained[roomIdParam][objectNumber].uniqueness << "\n";
                    }
                }
            }
            delimiterNumber = 0; //set back to 0 when finished

            objectNumber+=1;
            //totalObjectsFromWeights = objectNumber;
            room[roomIdParam].totalObjects = objectNumber; //set number of objects for room struct
            if (DEBUG_TRAINING_FILES_TO_STRUCT == 1) {
                cout << "total objects are " << room[roomIdParam].totalObjects << "\n";
            }
        }
        lineNumber++;
    }
    FILE_READER.close();
    if (DEBUG_TRAINING_FILES_TO_STRUCT == 1) {
        printSeparator(0);
    }
}

/*void sentenceSplitter() {
    if (userInstructionRaw.find(s2) != std::string::npos) {
        std::cout << "found!" << '\n';
}
}*/

std::string requestUserDestination(ros::Publisher espeak_pub) {
    //notify user via interface and speech
    std_msgs::String espeak_msg;
    espeak_msg.data = "Where would you like to go";
    espeak_pub.publish(espeak_msg);

    cout << "Where would you like to go?\n";
    std::string getUserInstructionRaw;
    getline(std::cin, getUserInstructionRaw);

    return getUserInstructionRaw;
}

void findObjectOrRoom() {
    int numberOfRoomsDetected = 0;
    for (int isRoom = 0; isRoom < totalRooms; isRoom++) {
        string getRoomName = room[isRoom].roomName;
        if (userInstructionRaw.find(getRoomName) != string::npos) {
            if (DEBUG_FIND_ROOM_MATCHES == 1) {
                cout << "found " << getRoomName << " in string\n";
            }
            decisionListRooms[numberOfRoomsDetected].roomName = getRoomName;
            numberOfRoomsDetected++;
        }
    }
    decisionListRoomsTotal = numberOfRoomsDetected;
    numberOfRoomsDetected = 0;
    if (decisionListRoomsTotal > 1) {
        wheelchair_interface_state = 400;
    }
    else {
        //read in pre trained data - room name | weighting | uniqueness - this should be available from context calculation
        int numberOfObjectsDetected = 0;
        for (int isRoom = 0; isRoom < totalRooms; isRoom++) {
            string getRoomName = room[isRoom].roomName;
            for (int isObject = 0; isObject < room[isRoom].totalObjects; isObject++) {
                string getObjectName = preTrained[isRoom][isObject].objectName;
                if (userInstructionRaw.find(getObjectName) != string::npos) {
                    if (DEBUG_FIND_OBJECT_MATCHES == 1) {
                        cout << "found " << getObjectName << " in " << getRoomName << "\n";
                    }
                    decisionListObjects[numberOfObjectsDetected].roomName = getRoomName;
                    decisionListObjects[numberOfObjectsDetected].objectName = getObjectName;
                    numberOfObjectsDetected++;
                }
            }
        }
        decisionListObjectsTotal = numberOfObjectsDetected;
        numberOfObjectsDetected = 0;
        cout << "finished room and object structing\n";
        wheelchair_interface_state = 3; //finished structing matching data
    }
}


/*void findObjectOrRoom() {
    //if user instruction is room then skip to other function, if not call find object matches
    int userInstructionMatch = 0;
    //user instruction is not a room, therefore must be an object...
    for (int isRoom = 0; isRoom < totalRooms; isRoom++) {
        int objectsFound = 0;
        if (userInstructionRaw == room[isRoom].roomName) {
            cout << "found room match \n";
            //navigateTo(something);
            userInstructionMatch = 1;
            navigateToDecision[0].roomName = room[isRoom].roomName;
        }
        else {
            for (int isObject = 0; isObject < room[isRoom].totalObjects; isObject++) {
                if (userInstruction == preTrained[isRoom][isObject].objectName) {
                    userInstructionMatch = 1;
                    cout << "found match " << userInstruction << " in " << room[isRoom].roomName << "\n";
                    trained[isRoom][objectsFound].objectName = preTrained[isRoom][isObject].objectName;
                    trained[isRoom][objectsFound].objectWeighting = preTrained[isRoom][isObject].objectWeighting;
                    trained[isRoom][objectsFound].uniqueness = preTrained[isRoom][isObject].uniqueness;
                    if (DEBUG_FIND_OBJECT_MATCHES == 1) {
                        cout << "found object " << objectsFound <<
                        trained[isRoom][objectsFound].objectName << ":" <<
                        trained[isRoom][objectsFound].objectWeighting << ":" <<
                        trained[isRoom][objectsFound].uniqueness << "\n";
                    }
                    objectsFound++;
                }
            }
        }
    }
    if (userInstructionMatch == 0) {
        wheelchair_interface_state = 404; //match not found, go to error state
    }
    else if (userInstructionMatch == 1) {
        wheelchair_interface_state = 3;
    }
}*/

int main(int argc, char * argv[]) {

    ros::init(argc, argv, "wheelchair_interface");
    ros::NodeHandle nodeHandle;

    ros::Publisher wheelchairGoal_pub = nodeHandle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
    ros::Publisher espeak_pub = nodeHandle.advertise<std_msgs::String>("/espeak_node/speak_line", 1000);
    ros::Rate loop_rate(10);

    doesWheelchairDumpPkgExist(); //check to see if dump package exists
    std::string wheelchair_dump_loc = ros::package::getPath("wheelchair_dump");
    weightingFileLoc = wheelchair_dump_loc + "/dump/context_training/";
    roomListLoc = wheelchair_dump_loc + "/dump/context_training/room.list";




    roomListToStruct(roomListLoc);
    totalRooms = calculateLines(roomListLoc);
    for (int i = 0; i < totalRooms; i++) {
    	//cout << room[i].roomName << "\n";
    }
	
    //populate 2d array of [room][objects] -> pass this to readtrainingfile as parameter
    for (int i = 0; i < totalRooms; i++) {
        //get roomname from corresponding position in for loop, add file extention and pass to function
        std::string generateRoomWeightFile = weightingFileLoc + room[i].roomName + weightingFileType;
        readTrainingFile(generateRoomWeightFile, i); //2nd param is room id
    }

    //find matching keywords from training file
    //0 is exit program
    //1 is ready for new question
    //2 is finding match
    //3 is need more info?
    

    int count = 0;
    while (ros::ok()) {

        switch(wheelchair_interface_state) {
            case 0: 
                ros::shutdown();
                exit(0);
                break;
            case 400:
                //too many rooms
                cout << "too many rooms detected, please choose one\n";
                wheelchair_interface_state = 0;
                break;
            case 404:
                //exit the program due to a problem
                cout << "match not found - restarting process.\n";
                wheelchair_interface_state = 0;
                break;
            case 1:
                //get user instruction
                userInstructionRaw = requestUserDestination(espeak_pub);
                if (userInstructionRaw != "") {
                    wheelchair_interface_state = 2; //request is not blank
                }
                //state stays at 0, wait for request
                break;
            case 2:
                findObjectOrRoom(); //struct matching rooms and objects
                break;
            case 3:
                //calculate context and work out which room and or object to navigate to
                //cout << "jumped into case 3\n";
                if (printOnce == 1) {
                    cout << "navigating to object\n";
                    printOnce = 0;
                }
                break;
            case 100:
                //start context to location conversion
                cout << "room name is " << navigateToDecision[0].roomName << "\n";
                break;
        }
        //get string message from user
        //cout << "Where would you like to go?\n";
        //publish question to espeak

        //string userInstruction;
        //std::string userInstruction;
        //getline(std::cin, userInstruction);
        //std_msgs::String userInstructionROS = std_msgs::String.userInstruction.c_str();
        ///////ROS_INFO_STREAM("MSG: " << userInstruction);
        //cout << userInstruction << "\n";

        move_base_msgs::MoveBaseActionGoal chosenGoal;

        chosenGoal.header;
        chosenGoal.header.seq = 0;
        chosenGoal.header.stamp;
        chosenGoal.header.stamp.sec = 0;
        chosenGoal.header.stamp.nsec = 0;
        chosenGoal.header.frame_id = "";

        chosenGoal.goal_id;
        chosenGoal.goal_id.stamp.sec = 0;
        chosenGoal.goal_id.stamp.nsec = 0;
        chosenGoal.goal_id.id;

        chosenGoal.goal;
        chosenGoal.goal.target_pose;
        chosenGoal.goal.target_pose.header;
        chosenGoal.goal.target_pose.header.seq = 0;
        chosenGoal.goal.target_pose.header.stamp;
        chosenGoal.goal.target_pose.header.stamp.sec = 0;
        chosenGoal.goal.target_pose.header.stamp.nsec = 0;
        chosenGoal.goal.target_pose.header.frame_id = "map";

        chosenGoal.goal.target_pose.pose;
        chosenGoal.goal.target_pose.pose.position;
        chosenGoal.goal.target_pose.pose.position.x = -3.003546478188607;
        chosenGoal.goal.target_pose.pose.position.y = 1.0007878321900925;
        chosenGoal.goal.target_pose.pose.position.z = 1.427680642769602e-07;
        chosenGoal.goal.target_pose.pose.orientation.x = -4.32232932053e-07;
        chosenGoal.goal.target_pose.pose.orientation.y = 4.866748911062099e-09;
        chosenGoal.goal.target_pose.pose.orientation.z = -0.010309099663378125;
        chosenGoal.goal.target_pose.pose.orientation.w = 0.9999468598200324;

        //std::stringstream ss;
        //ss << "hello world " << count;
        //msg.data = ss.str();
        /////msg.data = userInstruction;

        //ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
        */
        
        wheelchairGoal_pub.publish(chosenGoal);
        //cout << "ROS spinned \n";
        ros::spinOnce();

        //loop_rate.sleep();
        //++count;
    }
    return 0;
}
