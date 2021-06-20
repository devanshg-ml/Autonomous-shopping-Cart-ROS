//example main fnc for using move_part library
#include<ros/ros.h>
#include <move_part_lib/move_part.h>



int main(int argc, char** argv) {
    ros::init(argc, argv, "example_move_part_main"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
   MovePart movePart;
   movePart.preset_arm(); //move arm to preset pose
   //hard-code a part type and its pose for blind manipulation
   //in higher-level code, invoke object_finder first
   
   //WANT to move object codes to a neutral package, so same codes can be used
   // by object-finder
   
   //int partCode = object_finder::objectFinderGoal::GEARBOX_TOP;
   int partCode = part_codes::part_codes::GEARBOX_TOP;

   geometry_msgs::PoseStamped source_pose;
   source_pose.header.frame_id = "torso_lift_link";
   source_pose.pose.position.x = PART_X_VAL;
   source_pose.pose.position.y = PART_Y_VAL;   
   source_pose.pose.position.z = GRASP_HEIGHT;
   source_pose.pose.orientation.x = 0.0;
   source_pose.pose.orientation.y = 0.0;
   source_pose.pose.orientation.z = 0.0;
   source_pose.pose.orientation.w = 1.0;
   
   //invoke the  "get_part()" function to acquire specified part from specified  pose
   bool success = movePart.get_part(partCode,source_pose);
   

}

