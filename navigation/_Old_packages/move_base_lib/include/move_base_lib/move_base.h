#ifndef MOVE_BASE_H_
#define MOVE_BASE_H_
#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <mobot_pub_des_state/integer_query.h>

#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>


//location codes for navigation:
const int GEARBOX_TABLE = 1;
const int BOLT_TABLE = 2;
const int KIT_PICKUP_TABLE= 3;
const int GEAR_TABLE = 4;
const int KIT_DROPOFF_DEPOT = 5;

class MoveBase 
{
public:
	MoveBase();
        geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
        //cmd to move to a coded location (e.g. GEARBOX_TABLE); returns the actual pose at end of move
    bool move_to_location_code(int location_code, geometry_msgs::Pose &result_pose);
    void wait_for_path_done();
    void freeze(void); //tell Gazebo to freeze the robot
    void unfreeze(void); //tell Gazebo to unfreeze the robot

private:
        ros::NodeHandle nh_; 
    ros::ServiceClient path_client_;// = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    ros::ServiceClient queue_client_;// = n.serviceClient<mobot_pub_des_state::integer_query>("path_queue_query_service");
    ros::ServiceClient path_set_end_state_client_;// = n.serviceClient<mobot_pub_des_state::integer_query>("path_queue_query_service");

    ros::ServiceClient freeze_base_client_;// = n.serviceClient<mobot_pub_des_state::integer_query>("path_queue_query_service");
    std_srvs::SetBool freeze_robot_,unfreeze_robot_;
    ros::Subscriber gazebo_state_subscriber_;    
    void gazeboStateCallback(const geometry_msgs::Pose);
    geometry_msgs::Pose gazebo_pose_;
    geometry_msgs::PoseStamped gazebo_pose_stamped_;
    mobot_pub_des_state::path path_srv_msg_;
    mobot_pub_des_state::path path_srv_set_end_state_msg_;
    mobot_pub_des_state::integer_query int_query_srv_msg_;
    geometry_msgs::PoseStamped gearbox_table_approach_pose_;
    geometry_msgs::PoseStamped gear_table_approach_pose_;
    geometry_msgs::PoseStamped bolt_table_approach_pose_;
    geometry_msgs::PoseStamped kit_pickup_approach_pose_;    
    geometry_msgs::PoseStamped kit_dropoff_depot_pose_;    
    geometry_msgs::PoseStamped startup_pose_;   
};
#endif
