/** grab_tote.cpp
 * twa16
 * 4/24/19
 * 
 * File for FetchIt! to test:
 * - navigating the robot to the tote table,
 * - seeing a tote,
 * - and picking it up.
 */

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
//#include <move_part_lib/move_part.h>                // Manipulation
#include <mobot_pub_des_state/key_pose_move.h>      // Navigation
#include <mobot_pub_des_state/integer_query.h>      // Navigation
#include <object_finder_lib/object_finder.h>        // Perception

using namespace std;

geometry_msgs::PoseStamped kit_pose_;

/**
 * Callback function for the kit perception service.
 */
void kitCB(geometry_msgs::PoseStamped pose_received) {
    kit_pose_ = pose_received;
    ROS_INFO_STREAM("received pose is " << pose_received << endl);
}

/**
 * Main method.
 */
int main(int argc, char** argv) {
    /* Node Setup */
    // Initialize this node with a standard node handle
    ros::init(argc, argv, "grab_tote");
    ros::NodeHandle nh;

    // Create Manipulation and Perception objects
    //MovePart movePart;
    FindPart findPart;

    // Connect to Navigation service
    ros::ServiceClient set_key_pose_client = nh.serviceClient<mobot_pub_des_state::key_pose_move>("set_key_pose_index");
    while (!set_key_pose_client.exists()) {
        ROS_INFO("Waiting for service from set_key_pose_index ...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Connected client to set_key_pose_index service");

    // Connect to Path Queue Service
    ros::ServiceClient queue_client = nh.serviceClient<mobot_pub_des_state::integer_query>("path_queue_query_service");

    // Connect to Perception publisher
    ros::Subscriber kit_pose_sub = nh.subscribe<geometry_msgs::PoseStamped> ("kit_location", 1, kitCB);

    /* Navigation */
    // Manual pause for testing
    int ans;
    cout << "Enter 1 to move to TOTE_TABLE" << endl;
    cin >> ans;

    // Create Nagivation service message
    mobot_pub_des_state::key_pose_move key_pose_move_srv;

    // Set desination to the tote table
    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::TOTE_TABLE;

    ROS_INFO("Attempting navigation to tote table");
    set_key_pose_client.call(key_pose_move_srv);

    // Create Path Queue service message
    mobot_pub_des_state::integer_query integer_query_srv;

    // Wait until finished moving
    int npts = 1;
    while (npts > 0) {
        ros::Duration(0.5).sleep();
        queue_client.call(integer_query_srv);
        npts = integer_query_srv.response.int_val;
        ROS_INFO("waiting... %d points left in path queue", npts);
    }
    ROS_INFO("finished requested move");

    /* Perception */
    // Manual pause for testing
    cout << "Enter 1 to look for totes on the table" << endl;
    cin >> ans;

    while(ans != 0) {
        ros::spinOnce();
        ROS_INFO_STREAM("kit_pose found is " << kit_pose_ << endl);
        cout << "Enter 1 to look for totes on the table, 0 to quit" << endl;
        cin >> ans;
    }

    // Vector to contain Perceived parts
    std::vector <geometry_msgs::PoseStamped> part_poses;

    // Set Tote part code
    int partCode = 0;

    //ROS_INFO("Attempting to locate tote(s) on table");
    //bool success = findPart.find_part(partCode, part_poses);
/*
    int num_parts = part_poses.size();
    if (num_parts < 1) {
        ROS_WARN("DID NOT FIND ANY PARTS; QUITTING");
        return 0;
    }

    ROS_INFO("Found %d parts", num_parts);
    for (int i=0; i < num_parts; i++) {
        ROS_INFO_STREAM(part_poses[i] << endl);
    }

    // Choose the first part
    geometry_msgs::PoseStamped source_pose = part_poses[0];
    ROS_INFO_STREAM("Chosen part pose " << source_pose << endl);
    ROS_INFO("Triad pose: ");
    findPart.display_triad(source_pose);
*/

    /* Manipulation */
    // Manual pause for testing
    // TODO uncomment following lines once tote pose is found
   /* cout<<"Enter 1 to attempt grasp: ";
    cin>>ans;

    while(ans != 0) {

        ROS_INFO("Attempting to grasp chosen part");
        //bool success = movePart.get_part(partCode, source_pose);
        bool success = movePart.get_part(partCode, kit_pose_);

        ROS_INFO("Attempting to move arm to preset before place");
        success = movePart.preset_arm();

        // TODO uncomment following lines once arm function exists
        // ROS_INFO("Attempting to place kit");
        // success = movePart.place_kit()? function to place kit

        ROS_INFO("Attempting to move arm to preset after place");
        success = movePart.preset_arm();

        cout<<"Enter 1 to attempt grasp, 0 to quit: ";
        cin>>ans;

    }*/


    /* END */
    ROS_INFO("done");
    return 0;
}
