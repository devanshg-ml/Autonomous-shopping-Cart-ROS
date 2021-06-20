#include<object_finder_lib/object_finder.h>

FindPart::FindPart() : object_finder_ac_("/object_finder_action_service", true) {
  //constructor
   ROS_INFO("attempting to connect to object-finder action server");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac_.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server");
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
}

void FindPart::objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
            const object_finder::objectFinderResultConstPtr& result) {
    geometry_msgs::PoseStamped perceived_object_pose;
    perceived_object_poses_.clear();
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    found_object_code_=result->found_object_code;
    ROS_INFO("got object code response = %d; ",found_object_code_);
    if (found_object_code_==object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED) {
        ROS_WARN("object code not recognized");
    }
    else if (found_object_code_==object_finder::objectFinderResult::OBJECT_FOUND) {
        //ROS_INFO("found objects!");
        int n_objects = result->object_poses.size();
        for (int i_object=0;i_object<n_objects;i_object++) {
         //g_perceived_object_pose= result->object_pose; //MAKE MORE GENERAL FOR  POSES
            ROS_INFO("object %d: ",i_object);
            perceived_object_pose = result->object_poses[i_object];
            ROS_INFO("   pose x,y,z = %f, %f, %f",perceived_object_pose.pose.position.x,
                 perceived_object_pose.pose.position.y,
                 perceived_object_pose.pose.position.z);

            ROS_INFO("   quaternion x,y,z, w = %f, %f, %f, %f",perceived_object_pose.pose.orientation.x,
                 perceived_object_pose.pose.orientation.y,
                 perceived_object_pose.pose.orientation.z,
                 perceived_object_pose.pose.orientation.w);
            perceived_object_poses_.push_back(perceived_object_pose);
        }
         //g_pose_publisher->publish(g_perceived_object_pose);
    }
    else {
        ROS_WARN("object not found!");
    }
}


bool FindPart::find_part(int part_code, std::vector <geometry_msgs::PoseStamped> &part_poses) {

     ROS_INFO("sending goal to find part %d",part_code);
     object_finder_goal_.object_id = part_code;
     

     object_finder_ac_.sendGoal(object_finder_goal_,
                        boost::bind(&FindPart::objectFinderDoneCb, this, _1, _2));
     
        
     bool finished_before_timeout = object_finder_ac_.waitForResult(ros::Duration(10.0));
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }       
    int nparts =  perceived_object_poses_.size();
    if (nparts<1) return false;

    part_poses.clear();
    for (int i=0;i<nparts;i++) {
       part_poses.push_back(perceived_object_poses_[0]);
    }    
    
 return true;

}

void   FindPart::display_triad(geometry_msgs::PoseStamped triad_pose)  {
        //publish the desired frame pose to be displayed as a triad marker:   
    for (int i=0;i<3;i++) {
        pose_publisher_.publish(triad_pose);  
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
}

