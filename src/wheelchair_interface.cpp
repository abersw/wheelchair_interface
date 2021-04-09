#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
using namespace std;


int main(int argc, char * argv[]) {

    ros::init(argc, argv, "wheelchair_interface");
    ros::NodeHandle nodeHandle;

    ros::Publisher wheelchairGoal_pub = nodeHandle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
    ros::Publisher espeak_pub = nodeHandle.advertise<std_msgs::String>("/espeak_node/speak_line", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
        //++count;
    }
    return 0;
}
