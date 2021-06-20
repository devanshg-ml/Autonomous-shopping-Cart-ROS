/** grab_object_interactive.cpp
 wsn, 5/2/19
 * pgm prompts for object ID, then attempts object finder and manipulation (grab then drop)
 */

#include <ros/ros.h>
#include <move_part_lib/move_part.h>                // Manipulation
#include <mobot_pub_des_state/key_pose_move.h>      // Navigation
#include <object_finder_lib/object_finder.h>        // Perception
#include <part_codes/part_codes.h>

using namespace std;

int ans;

int main(int argc, char** argv) {
    /* Node Setup */
    // Initialize this node with a standard node handle
    ros::init(argc, argv, "grab_tote");
    ros::NodeHandle nh;

    // Create Manipulation and Perception objects
    MovePart movePart;
    FindPart findPart;

    ROS_WARN("This node does not do navigation...");
    // Connect to Navigation service
/*
    ros::ServiceClient set_key_pose_client = nh.serviceClient<mobot_pub_des_state::key_pose_move>("set_key_pose_index");
    while (!set_key_pose_client.exists()) {
        ROS_INFO("Waiting for service from set_key_pose_index ...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Connected client to set_key_pose_index service");
*/


    /* Navigation */
    // Manual pause for testing
/*

    cout << "Enter 1 to move to TOTE_TABLE" << endl;
    cin >> ans;

    // Create Nagivation service message
    mobot_pub_des_state::key_pose_move key_pose_move_srv;

    // Set desination to the tote table
    key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::TOTE_TABLE;

    ROS_INFO("Attempting navigation to tote table");
    set_key_pose_client.call(key_pose_move_srv);
*/


    /* Perception */
    // Manual pause for testing
/* 
    cout << "Enter 1 to look for totes on the table" << endl;
    cin >> ans;   
*/

    // Vector to contain Perceived parts
    std::vector <geometry_msgs::PoseStamped> part_poses;

    // Set Tote part code
    int partCode,part_index;
    
    
    cout<<"enter index for desired part:"<<endl<<"GEARBOX_BOTTOM = 1"<<endl<<"GEARBOX_TOP = 2"<<endl<<"BOLT = 3"<<endl<<"SMALL_GEAR = 4"<<endl<<"LARGE_GEAR = 5"<<endl<<"TOTE=6"<<endl;
    cout<<"enter  0 for fake part (hardcoded pose: "<<endl;
    cout<<"enter part index: ";    
    cin>>part_index;
    switch(part_index) {
        case 0: partCode = part_codes::part_codes::FAKE_PART; //object_finder_goal.object_id = part_codes::part_codes::FAKE_PART;
        break;
        case 1: partCode = part_codes::part_codes::GEARBOX_TOP;
        break;
        case 2: partCode = part_codes::part_codes::GEARBOX_BOTTOM;
        break;
        case 3: partCode = part_codes::part_codes::BOLT;
        break;
        case 4: partCode = part_codes::part_codes::SMALL_GEAR;
        break;
        case 5: partCode = part_codes::part_codes::LARGE_GEAR;
        break;
        case 6: partCode = part_codes::part_codes::TOTE;
        break;
        default:
            ROS_WARN("unknown part code; quitting");
            return 1;
    }
    
    //object_finder_goal.object_id = partCode;
    

    ROS_INFO("Attempting to locate object(s) on table");
    bool success = findPart.find_part(partCode, part_poses);

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
    int i_part;
    cout<<"enter index of part to grasp, 0 through "<<num_parts-1<<": ";
    cin>>i_part;
    geometry_msgs::PoseStamped source_pose = part_poses[i_part];
    ROS_INFO_STREAM("Chosen part pose " << source_pose << endl);
    ROS_INFO("Triad pose: ");
    findPart.display_triad(source_pose); //it will be useful to log topic: triad_display_pose

    cout<<"breakpoint to bag data; enter 1 when ready to bag; publications will include triad_display_pose for 3 sec: ";
    cin>>ans;
    for (int i=0;i<30;i++) {
        findPart.display_triad(source_pose);
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("done w/ 3-second triad publications");
    

    /* Manipulation */
    // Manual pause for testing
    cout<<"Enter 1 to attempt grasp: ";
    cin>>ans;

    ROS_INFO("Attempting to grasp chosen part");
    success = movePart.get_part(partCode, source_pose);
    
    movePart.release_grasped_part(); //drop the part back on the table
    ros::Duration(3.0).sleep();

    ROS_INFO("Attempting to move arm to preset");
    success = movePart.preset_arm();

    // TODO uncomment following lines once arm function exists
    //ROS_INFO("Attempting to place kit");
    //success = movePart.place_kit()? function to place kit


    /* END */
    ROS_INFO("done");
    return 0;
}
