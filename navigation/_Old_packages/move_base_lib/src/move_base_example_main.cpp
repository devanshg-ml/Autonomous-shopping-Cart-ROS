//example using move_base library
#include<move_base_lib/move_base.h>
using namespace std;
int main(int argc, char** argv) {
    ros::init(argc, argv, "example_move_base_main"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
   MoveBase moveBase;
   geometry_msgs::Pose result_pose;
   //move to the gearbox table:
   bool success = moveBase.move_to_location_code(GEAR_TABLE, result_pose);
   ROS_INFO("move_to_location_code (TABLE NAME) is done");
   ROS_INFO_STREAM("result_pose = "<<result_pose<<endl);
}

