/** visit_all.cpp
 * twa16; 
 * revised twa16 4/29/19
 * 
 * File for FetchIt! to test:
 * - navigating to each table
 */

#include <ros/ros.h>
#include <mobot_pub_des_state/key_pose_move.h>      // Navigation

using namespace std;

int ans;

int main(int argc, char **argv) {
    /* Node Setup */
    // Initialize this node with a standard node handle
    ros::init(argc, argv, "visit_all");
    ros::NodeHandle n;

    // Connect to Navigation service
    ros::ServiceClient set_key_pose_client = n.serviceClient<mobot_pub_des_state::key_pose_move>("set_key_pose_index");
    while (!set_key_pose_client.exists()) {
        ROS_INFO("waiting for service from set_key_pose_index ...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");


    /* Navigation */
    // Start at home
    cout << "enter 1 to move to HOME_POSE, 0 to exit program" << endl;
    cin >> ans;
    if(ans == 0){
        ROS_INFO("exiting");
        return 0;
    }

    // Set next key pose
    mobot_pub_des_state::key_pose_move key_pose_move_srv;
    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::HOME_POSE;
    set_key_pose_client.call(key_pose_move_srv);


    // Move to tote table
    cout << "enter 1 to move to TOTE_TABLE, 0 to exit program" << endl;
    cin >> ans;
    if(ans == 0){
        ROS_INFO("exiting");
        return 0;
    }

    // Set next key pose
    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::TOTE_TABLE;
    set_key_pose_client.call(key_pose_move_srv);
    
    
    // Then to gearbox table
    cout << "enter 1 to move to GEARBOX_TABLE, 0 to exit program" << endl;
    cin >> ans;
    if(ans == 0){
        ROS_INFO("exiting");
        return 0;
    }

    // Set next key pose
    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::GEARBOX_TABLE;
    set_key_pose_client.call(key_pose_move_srv);


    //Then to shunk table
    cout << "enter 1 to move to SHUNK_TABLE, 0 to exit program" << endl;
    cin >> ans;
    if(ans == 0){
        ROS_INFO("exiting");
        return 0;
    }

    // Set next key pose
    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::SHUNK_TABLE;
    set_key_pose_client.call(key_pose_move_srv);


    // Then to bolt bin
    cout << "enter 1 to move to BOLT_BIN, 0 to exit program" << endl;
    cin >> ans;
    if(ans == 0){
        ROS_INFO("exiting");
        return 0;
    }

    // Set next key pose
    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::BOLT_BIN;
    set_key_pose_client.call(key_pose_move_srv);


    // Then to kit dropoff
    cout << "enter 1 to move to KIT_DROPOFF, 0 to exit program" << endl;
    cin >> ans;
    if(ans == 0){
        ROS_INFO("exiting");
        return 0;
    }

    // Set next key pose
    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::KIT_DROPOFF;
    set_key_pose_client.call(key_pose_move_srv);

    
    ROS_INFO("done");
    return 0;
}
