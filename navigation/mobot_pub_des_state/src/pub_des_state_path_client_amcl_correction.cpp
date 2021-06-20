//pub_des_state_path_client_amcl_correction
//wsn, 4/20/19:


#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <mobot_pub_des_state/integer_query.h>
#include <mobot_pub_des_state/key_pose_move.h>

#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <xform_utils/xform_utils.h>

using namespace std;

std::vector <geometry_msgs::PoseStamped> g_vec_of_poses;
XformUtils xform_utils;

bool g_got_new_key_pose = false;
int g_key_pose_code = 0;
int g_nposes = 0;

//geometry_msgs::PoseStamped des_key_pose_wrt_map, base_link_pose_wrt_map, pub_des_state_end_pose --> pub_des_state_new_goal_pose


geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void set_hardcoded_poses() {
  /*int32 HOME_POSE = 0
    int32 TOTE_TABLE = 1 //Table 5 in the Map_goal_poses.pdf
    int32 GEARBOX_TABLE = 2 // Table 2 in the Map_goal_poses.pdf
    int32 SHUNK_TABLE = 3 // Table 1 in the Map_goal_poses.pdf
    int32 BOLT_BIN = 4 //Table 4 in the Map_goal_poses.pdf
    int32 KIT_DROPOFF = 5 // Table 3 in the Map_goal_poses.pdf
   */
    geometry_msgs::PoseStamped home_pose,tote_table,gearbox_table,shunk_table,bolt_bin,kit_dropoff;

    home_pose.header.frame_id = "world";
    g_vec_of_poses.clear();
    geometry_msgs::Pose pose;
    pose.position.x = 0.0; // 
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation = convertPlanarPhi2Quaternion(0); //really, the point-and-go traj builder ignores the pose; just put in something legal
    
    home_pose.pose = pose;
    g_vec_of_poses.push_back(home_pose);
    home_pose.pose.position.x += 0.01;
    home_pose.pose.position.y += -0.0025;
    g_vec_of_poses.push_back(home_pose); //extra push, so robot will face forward    
    tote_table = home_pose; // fill in legal values for each key pose, but still need to put in the true desired vals
    gearbox_table = home_pose;
    shunk_table = home_pose;
    bolt_bin = home_pose;
    kit_dropoff = home_pose;
    
    //tote_table key pose: 
    tote_table.pose.position.x = 0.25; 
    tote_table.pose.position.y = 0.53;// 0.59
    tote_table.pose.position.z = 0.0; 
    tote_table.pose.orientation = convertPlanarPhi2Quaternion(1.578); 
    g_vec_of_poses.push_back(tote_table);  
    //facing tote_table: 
    tote_table.pose.position.x+= 0.0025; 
    tote_table.pose.position.y+= 0.01; 
    tote_table.pose.orientation.z = -0.24; 
    g_vec_of_poses.push_back(tote_table); 
    
    
    //gearbox_table key pose: 
    gearbox_table.pose.position.x = 0.18;
    gearbox_table.pose.position.y = -0.75;//-0.80
    gearbox_table.pose.position.z = 0.0; 
    gearbox_table.pose.orientation = convertPlanarPhi2Quaternion(-1.578); 
    g_vec_of_poses.push_back(gearbox_table); 
    //facing gearbox_table: 
    gearbox_table.pose.position.x+= -0.0025;
    gearbox_table.pose.position.y+= -0.01;
    gearbox_table.pose.orientation.z = -0.24; 
    g_vec_of_poses.push_back(gearbox_table);
    
    
    //shunk_table key pose: 
    shunk_table.pose.position.x = 0.58;//0.63
    shunk_table.pose.position.y = 0.20;
    shunk_table.pose.position.z = 0.0; 
    shunk_table.pose.orientation = convertPlanarPhi2Quaternion(0); 
    g_vec_of_poses.push_back(shunk_table);
    //facing shunk_table: 
    shunk_table.pose.position.x+= 0.01;
    shunk_table.pose.position.y+= -0.0025;
    shunk_table.pose.orientation.z = -0.26; 
    g_vec_of_poses.push_back(shunk_table);
    
    
    //bolt_bin key pose: 
    bolt_bin.pose.position.x = -0.42;//-0.45
    bolt_bin.pose.position.y = 0.66;
    bolt_bin.pose.position.z = 0.0; 
    bolt_bin.pose.orientation = convertPlanarPhi2Quaternion(-3.1415);
    g_vec_of_poses.push_back(bolt_bin);
    //facing bolt_bin
    bolt_bin.pose.position.x+= -0.01;
    bolt_bin.pose.position.y+= 0.0025;
    bolt_bin.pose.orientation.z = -0.25;
    g_vec_of_poses.push_back(bolt_bin);
    
    
    //kit_dropoff table: 
    kit_dropoff.pose.position.x = -0.88;
    kit_dropoff.pose.position.y = -0.48;//-0.54
    kit_dropoff.pose.position.z = 0.0; 
    kit_dropoff.pose.orientation = convertPlanarPhi2Quaternion(-1.578);
    g_vec_of_poses.push_back(kit_dropoff);
    //facing kit_dropoff
    kit_dropoff.pose.position.x+= -0.0025;
    kit_dropoff.pose.position.y+= -0.01;
    kit_dropoff.pose.orientation.z = -0.24; 
    g_vec_of_poses.push_back(kit_dropoff);
}

bool setKeyPoseCB(mobot_pub_des_state::key_pose_moveRequest& request, mobot_pub_des_state::key_pose_moveResponse& response) {
    g_key_pose_code = request.key_pose_code;
    ROS_INFO_STREAM("setting key pose to: "<< g_key_pose_code<<endl);
    //make sure this code is viable
    if ( (g_nposes>g_key_pose_code) && (g_key_pose_code>-1 ) ) {
    g_got_new_key_pose = true;
    response.status = true; 
    }
    else {
        response.status = false; 
    }
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    ros::ServiceClient queue_client = n.serviceClient<mobot_pub_des_state::integer_query>("path_queue_query_service");
    //ros::ServiceClient get_end_pose_client = n.serviceClient<mobot_pub_des_state::path>("path_get_end_state_service");
    ros::ServiceClient set_current_pose_client = n.serviceClient<mobot_pub_des_state::path>("set_current_pose_service");
    ros::ServiceServer set_key_pose_index = n.advertiseService("set_key_pose_index",setKeyPoseCB);   


    //tf::TransformListener tfListener;   
    tf::TransformListener* tfListener_ptr;
    tfListener_ptr = new tf::TransformListener;

    tf::StampedTransform stfOdomWrtMap, stfBaseLinkWrtMap;


    //geometry_msgs::Quaternion quat;
    //geometry_msgs::Transform tf_base_link_wrt_map;
    //tf::Vector3 current_tf_origin_base_link_wrt_map;
    //tf::Vector3 tfVec;
    //double x_current_base_link_wrt_map, y_current_base_link_wrt_map;
    //double x_des_base_link_wrt_map, y_des_base_link_wrt_map;
    //double prev_x_goal, prev_y_goal;
    //double dx_move, dy_move;

    //geometry_msgs::PoseStamped pose_stamped1, pose_stamped2, pose_stamped3, pose_stamped4;
    //geometry_msgs::PoseStamped pub_des_state_end_pose, pub_des_state_new_goal_pose;
    geometry_msgs::PoseStamped des_key_pose_wrt_map, des_key_pose_wrt_map2, wiggle_pose_cmd;

    set_hardcoded_poses();

    g_nposes = g_vec_of_poses.size();

    while (!client.exists()) {
        ROS_INFO("waiting for service from pub_des_state ...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    mobot_pub_des_state::path path_srv;
    mobot_pub_des_state::path get_end_pose_srv;
    mobot_pub_des_state::path set_current_pose_srv;

    mobot_pub_des_state::integer_query integer_query_srv;

    bool tferr = true;
    ROS_INFO("waiting for tf between odom and map...");
    while (tferr) {
        tferr = false;
        try {
            tfListener_ptr->lookupTransform("map", "odom", ros::Time(0), stfOdomWrtMap);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("map to odom tf is good");

    tferr = true;
    ROS_INFO("waiting for tf between base_link and map...");
    while (tferr) {
        tferr = false;
        try {
            tfListener_ptr->lookupTransform("map", "base_link", ros::Time(0), stfBaseLinkWrtMap);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("got base_link w/rt map transform");
    
    ROS_WARN("making small motions to improve AMCL: ");

    Eigen::Affine3d affine_base_link_wrt_map = xform_utils.transformStampedTfToEigenAffine3d(stfBaseLinkWrtMap);
    geometry_msgs::PoseStamped base_link_pose_wrt_map = xform_utils.transformEigenAffine3dToPoseStamped(affine_base_link_wrt_map);
    ROS_INFO_STREAM("base_link_pose_wrt_map" << endl << base_link_pose_wrt_map << endl);

    set_current_pose_srv.request.set_current_pose = base_link_pose_wrt_map;
    set_current_pose_client.call(set_current_pose_srv);

    //------------  DO A WIGGLE ON START-UP,  JUST TO HELP OUT AMCL LOCALIZATION
    wiggle_pose_cmd = base_link_pose_wrt_map;
    wiggle_pose_cmd.pose.position.x += 0.1;
    wiggle_pose_cmd.pose.position.y += 0.1;


    path_srv.request.path.poses.push_back(wiggle_pose_cmd);
    client.call(path_srv);

    int npts = 1;
    while (npts > 0) {
        ros::Duration(0.5).sleep();
        queue_client.call(integer_query_srv);
        npts = integer_query_srv.response.int_val;
        ROS_INFO("waiting... %d points left in path queue", npts);

    }

    wiggle_pose_cmd.pose.position.x += 0.1;
    wiggle_pose_cmd.pose.position.y -= 0.1;

    path_srv.request.path.poses.push_back(wiggle_pose_cmd);
    client.call(path_srv);

    npts = 1;
    while (npts > 0) {
        ros::Duration(0.5).sleep();
        queue_client.call(integer_query_srv);
        npts = integer_query_srv.response.int_val;
        ROS_INFO("waiting... %d points left in path queue", npts);

    }
    wiggle_pose_cmd.pose.position.x += 0.1;
    //wiggle_pose_cmd.pose.position.y -= 0.1;

    path_srv.request.path.poses.push_back(wiggle_pose_cmd);
    client.call(path_srv);

    npts = 1;
    while (npts > 0) {
        ros::Duration(0.5).sleep();
        queue_client.call(integer_query_srv);
        npts = integer_query_srv.response.int_val;
        ROS_INFO("waiting... %d points left in path queue", npts);

    }    
    //------------  END OF START-UP WIGGLE ------------
    ROS_INFO("end of initialization; ready to service motion requests");
    
    //int key_pose_index = 0;  //want to set this with a service

    while (true) { //repeat forever
        path_srv.request.path.poses.clear();

        //---------------change this! ----------------
        //the coordinator should enter a desired pose, or key-pose code to set goal pose
        //this should probably be a service or action server
        //service might be OK...robot does not have anything to do until it reaches the next destination

        //cout<<"enter a key pose num, 0 to "<<nposes-1<<": ";  //option to prompt for key pose index
        //cin>>key_pose_index;

        //key_pose_index = rand() % 4; //option to select key poses at random
        while (!g_got_new_key_pose) {
            ros::spinOnce();
            ros::Duration(0.2).sleep();
        }


        //----------preserve this-----------
        // can hard-code key poses via set_hardcoded_poses(), then refer to them by index or mnemonic
        ROS_INFO("received request to move to key pose number %d",g_key_pose_code);        
        des_key_pose_wrt_map = g_vec_of_poses[g_key_pose_code*2];
        des_key_pose_wrt_map2 = g_vec_of_poses[g_key_pose_code*2+1];

        //make adjustment before sending next point:
        tferr = true;
        //ROS_INFO("waiting for tf between base_link and map...");

        //find current pose of robot on map; this  ONLY works if have map loaded and amcl running!
        while (tferr) {
            tferr = false;
            try {
                tfListener_ptr->lookupTransform("map", "base_link", ros::Time(0), stfBaseLinkWrtMap);
            } catch (tf::TransformException &exception) {
                ROS_WARN("%s; retrying listen for  tf between base_link and map...", exception.what());
                tferr = true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();
            }
        }
        //ROS_INFO("map to base_link tf is good");
        affine_base_link_wrt_map = xform_utils.transformStampedTfToEigenAffine3d(stfBaseLinkWrtMap);
        base_link_pose_wrt_map = xform_utils.transformEigenAffine3dToPoseStamped(affine_base_link_wrt_map);

        ROS_INFO_STREAM("base_link_pose_wrt_map" << endl << base_link_pose_wrt_map << endl);

        //tell pub_des_state that this is the new starting point:
        set_current_pose_srv.request.set_current_pose = base_link_pose_wrt_map;
        set_current_pose_client.call(set_current_pose_srv);

        //tell pub_des_state this is where we want to go:
        path_srv.request.path.poses.push_back(des_key_pose_wrt_map);
        path_srv.request.path.poses.push_back(des_key_pose_wrt_map2);
        
        
        client.call(path_srv);

        int npts = 1;
        while (npts > 0) {
            ros::Duration(0.5).sleep();
            queue_client.call(integer_query_srv);
            npts = integer_query_srv.response.int_val;
            ROS_INFO("waiting... %d points left in path queue", npts);
        }
        ROS_INFO("finished requested move");
        g_got_new_key_pose = false; //reset trigger
    }
    return 0;
}