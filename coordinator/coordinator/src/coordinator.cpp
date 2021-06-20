//coordinator.cpp
// wsn, Feb 2019
#include<ros/ros.h>
#include<move_part_lib/move_part.h>
#include<move_base_lib/move_base.h>
#include<object_finder_lib/object_finder.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinator"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
   MoveBase moveBase;
   MovePart movePart;
   FindPart findPart;

   std::vector <geometry_msgs::PoseStamped> part_poses; //for perception
   geometry_msgs::PoseStamped part_pose;
   geometry_msgs::Pose result_pose; //for navigation
   int partCode = part_codes::part_codes::GEARBOX_TOP;
   geometry_msgs::PoseStamped source_pose; //for part acquisition

   //move to the gearbox table:
   ROS_INFO("attempting navigation to gearbox table");
   bool success = moveBase.move_to_location_code(GEARBOX_TABLE, result_pose);
   ROS_INFO("move_to_location_code GEARBOX_TABLE is done");
   ROS_INFO_STREAM("result_pose = "<<result_pose<<endl);
   //settle for a while:
   ros::Duration(2.0).sleep();
   moveBase.unfreeze(); //unfreeze to stop Gazebo jitter
   ros::Duration(2.0).sleep();
   
   //look for the part of interest:
   ROS_INFO("attempting to locate gearbox parts on table");
   success = findPart.find_part(partCode,part_poses);
   int nparts = part_poses.size();
   ROS_INFO("found %d parts",nparts);
   for  (int i=0;i<nparts;i++) {
     ROS_INFO_STREAM(part_poses[i]<<endl);
   }


   if (nparts<1) {
      ROS_WARN("DID NOT FIND ANY PARTS; QUITTING");
      return 0;
   }

   ROS_INFO("attempting to acquire a perceived part using pose: ");

   source_pose= part_poses[0];      //choose the first part found:
   ROS_INFO_STREAM(source_pose<<endl);
   findPart.display_triad(source_pose);
   
   int ans;
   //cout<<"enter 1 to attempt grasp: ";
   //cin>>ans;
   //invoke the  "get_part()" function to acquire specified part from specified  pose
   success = movePart.get_part(partCode,source_pose);
   
   success = movePart.preset_arm();
   ros::Duration(1.0).sleep(); //not necessary?
   success = movePart.move_to_dropoff_kit1();
   
   

   ROS_INFO("done");
   return 0;
}
