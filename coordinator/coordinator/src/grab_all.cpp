/** grab_all.cpp
 * twa16 5/1/19
 * Edited twa16 5/3 to place parts instead of releasing them
 */

#include <ros/ros.h>
#include <move_part_lib/move_part.h>                // Manipulation
#include <mobot_pub_des_state/key_pose_move.h>      // Navigation
#include <mobot_pub_des_state/integer_query.h>      // Navigation
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

    // Connect to Navigation service
    ros::ServiceClient set_key_pose_client = nh.serviceClient<mobot_pub_des_state::key_pose_move>("set_key_pose_index");
    while (!set_key_pose_client.exists()) {
        ROS_INFO("Waiting for service from set_key_pose_index ...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Connected client to set_key_pose_index service");

    // Connect to the Navigation feedback service
    ros::ServiceClient queue_client = nh.serviceClient<mobot_pub_des_state::integer_query>("path_queue_query_service");
    while (!queue_client.exists()) {
        ROS_INFO("Waiting for service from path_queue_query_service ...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Connected client to path_queue_query_service service");


    /* Competition */
    /*
        int32 HOME_POSE = 0
        int32 TOTE_TABLE = 1 //Table 5 in the Map_goal_poses.pdf
        int32 GEARBOX_TABLE = 2 // Table 2 in the Map_goal_poses.pdf
        int32 SHUNK_TABLE = 3 // Table 1 in the Map_goal_poses.pdf
        int32 BOLT_BIN = 4 //Table 4 in the Map_goal_poses.pdf
        int32 KIT_DROPOFF = 5 // Table 3 in the Map_goal_poses.pdf
    */
    cout << "Choose a movement operation: 0 for HOME \n1 for moving to Tote table and grabbing a tote\n2 for moving to Gearbox table and grabbing a gearbox\n3 for moving to the Shunk table and machining a part\n4 for moving to the Bolt table and grabbing a bolt from the bin\n5 for dropping off the kit" << endl;
    cin >> ans;


    
    /* Navigation */
    // Create Nagivation service message
    mobot_pub_des_state::key_pose_move key_pose_move_srv;

    // Set desination to the table as noted by ans
    switch(ans){
        case 0:
            key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::HOME_POSE;
            break;
        case 1:
            key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::TOTE_TABLE;
            break;
        case 2:
            key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::GEARBOX_TABLE;
            break;
        case 3:
            key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::SHUNK_TABLE;
            break;
        case 4:
            key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::BOLT_BIN;
            break;
        case 5:
            key_pose_move_srv.request.key_pose_code = mobot_pub_des_state::key_pose_move::Request::KIT_DROPOFF;
            break;
        default:
            ROS_WARN("Unrecognized navigation operation");
            break;
    }

    ROS_INFO("Attempting navigation...");
    set_key_pose_client.call(key_pose_move_srv);

    // Wait for movement to finish
    mobot_pub_des_state::integer_query integer_query_srv;
    int npts = 1;
    while (npts > 0) {
        ros::Duration(0.5).sleep();
        queue_client.call(integer_query_srv);
        npts = integer_query_srv.response.int_val;
        ROS_INFO("Waiting, %d points left in path queue", npts);
    }
    ROS_INFO("Finished requested move");



    /* Perception */
    cout << "Choose a perception operation: 0 for Fake part \n1 for Gearbox top\n2 Gearbox bottom\n3 for Bolt\n4 for Small gear\n5 for Large gear\n6 for Tote" << endl;
    cin >> ans;  

    // Vector to contain Perceived parts
    std::vector <geometry_msgs::PoseStamped> part_poses;

    // Create part code
    int partCode;

    // Set Part code as noted by ans
    switch(ans){
        case 0:
            partCode = part_codes::part_codes::FAKE_PART;
            break;
        case 1:
            partCode = part_codes::part_codes::GEARBOX_TOP;
            break;
        case 2:
            partCode = part_codes::part_codes::GEARBOX_BOTTOM;
            break;
        case 3:
            partCode = part_codes::part_codes::BOLT;
            break;
        case 4:
            partCode = part_codes::part_codes::SMALL_GEAR;
            break;
        case 5:
            partCode = part_codes::part_codes::LARGE_GEAR;
            break;
        case 6:
            partCode = part_codes::part_codes::TOTE;
            break;
        default:
            ROS_WARN("Unrecognized perception operation");
            break;
    }

    ROS_INFO("Attempting to locate part(s) on table");
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
    geometry_msgs::PoseStamped source_pose = part_poses[0];
    ROS_INFO_STREAM("Chosen part pose " << source_pose << endl);
    ROS_INFO("Triad pose: ");
    findPart.display_triad(source_pose);



    /* Manipulation */
    // Manual pause for testing
    // sleep statements are for testing purposes
    cout<<"Enter 1 to attempt grasp: ";
    cin>>ans;

    ROS_INFO("Attempting to grasp chosen part");
    success = movePart.get_part(partCode, source_pose);
  
    if(!success){
        ROS_ERROR("Failed to get part");
        return 0;
    }
    ros::Duration(4.0).sleep();

    // movePart.release_grasped_part(); //drop the part back on the table
    // ros::Duration(3.0).sleep();

    ROS_INFO("Attempting to move arm to preset");
    success = movePart.preset_arm();
    if(!success){
        ROS_ERROR("Failed to move to preset");
        return 0;
    }
    ros::Duration(3.0).sleep();

    cout<<"enter 1 to attempt stowing grasped  part in kit: ";
    cin>>ans;
  
    // Place the part in the kit
    switch(partCode){
        case part_codes::part_codes::GEARBOX_TOP:
        case part_codes::part_codes::GEARBOX_BOTTOM:
            ROS_INFO("Attempting to place gearbox part in kit zone 1");
            success = movePart.move_to_dropoff_kit1();

            break;
        case part_codes::part_codes::BOLT:
            ROS_INFO("Attempting to place bolt part in kit zone 2");
            success = movePart.move_to_dropoff_kit2();
            
            break;
        case part_codes::part_codes::LARGE_GEAR:
        case part_codes::part_codes::SMALL_GEAR:
            ROS_INFO("Attempting to place gear part in kit zone 3");
            success = movePart.move_to_dropoff_kit3();
            
            break;
        case part_codes::part_codes::TOTE:
            ROS_INFO("Attempting to pickup kit");
            success = movePart.move_to_pickup_tote();
            if(!success){
                ROS_ERROR("Failed to pickup tote part");
                return 0;
            }

            ros::Duration(3.0).sleep();

            ROS_INFO("Attempting to dropoff kit");
            success = movePart.move_to_dropoff_tote();
            
            break;
        default:
            ROS_WARN("no dropoff case for this part! ");
            movePart.release_grasped_part(); //drop the part back on the table
            return 0;
    }

    if(!success){
        ROS_ERROR("Failed to place part");
        return 0;
    }

    ros::Duration(3.0).sleep();

    ROS_INFO("Attempting to recover from dropoff");
    success = movePart.recover_from_dropoff();



    /* END */
    ROS_INFO("done");
    return 0;
}
