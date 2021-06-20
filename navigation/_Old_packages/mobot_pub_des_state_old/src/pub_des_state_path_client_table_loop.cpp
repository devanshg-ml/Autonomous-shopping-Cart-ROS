
#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <mobot_pub_des_state/integer_query.h>

#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    ros::ServiceClient queue_client = n.serviceClient<mobot_pub_des_state::integer_query>("path_queue_query_service");

    geometry_msgs::Quaternion quat;

    while (!client.exists()) {
        ROS_INFO("waiting for service...");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    mobot_pub_des_state::path path_srv;
    mobot_pub_des_state::integer_query integer_query_srv;
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    while (true) { //repeat forever
        path_srv.request.path.poses.clear();
	//go to table 1:
		pose.position.x = 0.50;//0.80 
        	pose.position.y = 0.30;//0.17,
       	 	pose.position.z = 0.0;
		quat = convertPlanarPhi2Quaternion(0);
        	pose.orientation = quat;
        	pose_stamped.pose = pose;
        	path_srv.request.path.poses.push_back(pose_stamped);
                //table 1.1
                pose.position.x = 0.501; 
        	pose.position.y = 0.30;
       	 	pose.position.z = 0.0;
		quat = convertPlanarPhi2Quaternion(0);
        	pose.orientation = quat;
        	pose_stamped.pose = pose;
        	path_srv.request.path.poses.push_back(pose_stamped);
                       
       //go to table 2:
		pose.position.x = 0.50;//0.5
		pose.position.y = -0.35;//-0.35
                quat = convertPlanarPhi2Quaternion(-1.5708);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
                
                //table 2.1
                pose.position.x = 0.50;//0.5
		pose.position.y = -0.351;//-0.351
                quat = convertPlanarPhi2Quaternion(-1.5708);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
               
                
        //go to table 3:                
                pose.position.x = -0.50;//-0.54
		pose.position.y = -0.351;//-0.26
                quat = convertPlanarPhi2Quaternion(-1.5708);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
                
                //table 3.1
                pose.position.x = -0.50;//-0.54
		pose.position.y = -0.352;//-0.261
                quat = convertPlanarPhi2Quaternion(-1.5708);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
                
               
                
       //go to table 4:           
		pose.position.x = -0.50;//-0.51
		pose.position.y = 0.10;//-0.07
                quat = convertPlanarPhi2Quaternion(-3.1416);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
                
                //table 4.1
                pose.position.x = -0.501;
		pose.position.y = 0.10;
                quat = convertPlanarPhi2Quaternion(-3.1416);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
                
                
		
       //go to table 5:               
		pose.position.x = -0.13;//-0.13
		pose.position.y = 0.35;//0.32
                quat = convertPlanarPhi2Quaternion(1.5708);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
		//pose 5.1
                pose.position.x = -0.13;
		pose.position.y = 0.351;
                quat = convertPlanarPhi2Quaternion(1.5708);
		pose.orientation = quat;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);

        client.call(path_srv);

        int npts = 1;
        while (npts > 0) {
            queue_client.call(integer_query_srv);
            npts = integer_query_srv.response.int_val;
            ROS_INFO("waiting... %d points left in path queue", npts);
            ros::Duration(1.0).sleep();
        }
    }
    return 0;
}
