#include <move_part_lib/move_part.h>

MovePart::MovePart() :
robot_arm_motion_action_client_("/arm_controller/follow_joint_trajectory",true),
torso_action_client_("/torso_controller/follow_joint_trajectory", true),
head_action_client_("/head_controller/follow_joint_trajectory", true)
//object_finder_ac_("/object_finder_action_service", true)
{ //constructor
  ROS_INFO("MovePart constructor");

    b_des_ << 0, 0, -1;
    n_des_ << 1, 0, 0;
    t_des_ = b_des_.cross(n_des_);

    R_gripper_down_.col(0) = n_des_;
    R_gripper_down_.col(1) = t_des_;
    R_gripper_down_.col(2) = b_des_;            
}

void MovePart::initializeSubscribers() {

}

void MovePart::initializePublishers() {

}


void MovePart::armMotionCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {

}

void MovePart::torsoDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {

}

void MovePart::headDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {

}

bool MovePart::preset_arm() {
    cart_motion_commander_.plan_jspace_traj_current_to_waiting_pose(2, 2.0); 
    int result = cart_motion_commander_.execute_planned_traj();
    ROS_ERROR("Return code is: %d", result);
    ros::Duration(2.0).sleep();// wait for execution
    return true; //! A dirty fix!
}

bool MovePart::move_to_dropoff_kit1() {
    cart_motion_commander_.plan_jspace_traj_current_to_kit_dropoff1(2, 2.0);  //(nsteps, arrival time) tinker with for optimization
    cart_motion_commander_.execute_planned_traj();
    //ros::Duration(2.0).sleep();// wait for execution
    gripper_interface_.releaseObject();
}

bool MovePart::move_to_dropoff_kit2() {
    cart_motion_commander_.plan_jspace_traj_current_to_kit_dropoff2(2, 2.0);  //(nsteps, arrival time) tinker with for optimization
    cart_motion_commander_.execute_planned_traj();
    //ros::Duration(2.0).sleep();// wait for execution
    gripper_interface_.releaseObject();
}

bool MovePart::move_to_dropoff_kit3() {
    cart_motion_commander_.plan_jspace_traj_current_to_kit_dropoff3(2, 2.0);  //(nsteps, arrival time) tinker with for optimization
    cart_motion_commander_.execute_planned_traj();
    //ros::Duration(2.0).sleep();// wait for execution
    gripper_interface_.releaseObject();
}

bool MovePart::move_to_dropoff_tote() {
    cart_motion_commander_.plan_jspace_traj_current_to_tote_dropoff(2, 2.0);  //(nsteps, arrival time) tinker with for optimization
    cart_motion_commander_.execute_planned_traj();
    //ros::Duration(2.0).sleep();// wait for execution
    gripper_interface_.releaseObject();
}

bool MovePart::move_to_pickup_tote() {
    cart_motion_commander_.plan_jspace_traj_current_to_tote_pickup(2, 2.0);  //(nsteps, arrival time) tinker with for optimization
    cart_motion_commander_.execute_planned_traj();
    //ros::Duration(2.0).sleep();// wait for execution
    gripper_interface_.graspObject();
}

bool MovePart::recover_from_dropoff() {
    cart_motion_commander_.plan_jspace_traj_recover_from_dropoff(2, 2.0); //(nsteps, arrival time) tinker with for optimization
    cart_motion_commander_.execute_planned_traj();    
    
}

bool MovePart::recover_from_tote() {
    cart_motion_commander_.plan_jspace_traj_recover_from_tote(2, 2.0); //(nsteps, arrival time) tinker with for optimization
    cart_motion_commander_.execute_planned_traj();    
    
}

Eigen::Matrix3d MovePart::compute_rot_z(double angle) {
    Eigen::Matrix3d Rotz;
    Eigen::Vector3d n,t,b;
    n<<cos(angle),sin(angle),0;
    t<<-sin(angle),cos(angle),0;
    b<<0,0,1;
    Rotz.col(0)=n;
    Rotz.col(1)=t;
    Rotz.col(2)=b;
    return Rotz;
}


//use this function to compute where the hand should be (w/rt torso) to grasp the specified part that is at part_pose

Eigen::Affine3d MovePart::compute_grasp_affine(int part_code, geometry_msgs::PoseStamped part_pose) {
    //assume we will grasp by centroid,  e.g.:
    //O_des_ << part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z + TOTE_GRASP_HEIGHT_WRT_TOTE_ORIGIN;

    Eigen::Affine3d grasp_pose;
    Eigen::Matrix3d Rotz, Rot_gripper;
    Eigen::Vector3d O_part;
    double object_angle;
    //int32 FAKE_PART=1
//int32 GEARBOX_TOP=101
//int32 GEARBOX_BOTTOM=102
//int32 BOLT = 103
//int32 SMALL_GEAR =104
//int32 LARGE_GEAR = 105
//int32 TOTE = 106
    switch (part_code) {
        case part_codes::part_codes::TOTE:
            //grab part at centroid:
            //hard-coded grasp heigth from Fetch remote experiments
            O_part << part_pose.pose.position.x, part_pose.pose.position.y, TOTE_GRASP_HEIGHT; //part_pose.pose.position.z + TOTE_GRASP_HEIGHT_WRT_TOTE_ORIGIN;
            //double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
            object_angle = xformUtils.convertPlanarQuat2Phi(part_pose.pose.orientation) + M_PI / 2; //gripper must be parallel to tote x-axis

            Rotz = compute_rot_z(object_angle);
            ROS_INFO_STREAM("Rotz: "<<endl<<Rotz<<endl);
            ROS_INFO_STREAM("R_gripper_down: "<<endl<<R_gripper_down_);
            Rot_gripper = Rotz*R_gripper_down_;
            ROS_INFO_STREAM("specifying gripper orientation:  " << endl << Rot_gripper << endl);
            grasp_pose.linear() = Rot_gripper;
            grasp_pose.translation() = O_part;
            break;
        case part_codes::part_codes::GEARBOX_TOP:
            //grab part at centroid:
            //hard-coded grasp heigth from Fetch remote experiments
            O_part << part_pose.pose.position.x, part_pose.pose.position.y, GEARBOX_GRASP_HEIGHT; //part_pose.pose.position.z + TOTE_GRASP_HEIGHT_WRT_TOTE_ORIGIN;
            //double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
            object_angle = xformUtils.convertPlanarQuat2Phi(part_pose.pose.orientation); // align gripper w/rt major axis

            Rotz = compute_rot_z(object_angle);
            ROS_INFO_STREAM("Rotz: "<<endl<<Rotz<<endl);
            ROS_INFO_STREAM("R_gripper_down: "<<endl<<R_gripper_down_);
            Rot_gripper = Rotz*R_gripper_down_;
            ROS_INFO_STREAM("specifying gripper orientation:  " << endl << Rot_gripper << endl);
            grasp_pose.linear() = Rot_gripper;
            grasp_pose.translation() = O_part;
            break;
        case part_codes::part_codes::GEARBOX_BOTTOM:
            //grab part at centroid:
            //hard-coded grasp heigth from Fetch remote experiments
            O_part << part_pose.pose.position.x, part_pose.pose.position.y, GEARBOX_GRASP_HEIGHT; //part_pose.pose.position.z + TOTE_GRASP_HEIGHT_WRT_TOTE_ORIGIN;
            //double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
            object_angle = xformUtils.convertPlanarQuat2Phi(part_pose.pose.orientation); 

            Rotz = compute_rot_z(object_angle);
            ROS_INFO_STREAM("Rotz: "<<endl<<Rotz<<endl);
            ROS_INFO_STREAM("R_gripper_down: "<<endl<<R_gripper_down_);
            Rot_gripper = Rotz*R_gripper_down_;
            ROS_INFO_STREAM("specifying gripper orientation:  " << endl << Rot_gripper << endl);
            grasp_pose.linear() = Rot_gripper;
            grasp_pose.translation() = O_part;
            break;
        case part_codes::part_codes::BOLT:
            //grab part at centroid:
            //hard-coded grasp heigth from Fetch remote experiments
            O_part << part_pose.pose.position.x, part_pose.pose.position.y, BOLT_GRASP_HEIGHT; //part_pose.pose.position.z + TOTE_GRASP_HEIGHT_WRT_TOTE_ORIGIN;
            //double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
            object_angle = xformUtils.convertPlanarQuat2Phi(part_pose.pose.orientation); 

            Rotz = compute_rot_z(object_angle);
            ROS_INFO_STREAM("Rotz: "<<endl<<Rotz<<endl);
            ROS_INFO_STREAM("R_gripper_down: "<<endl<<R_gripper_down_);
            Rot_gripper = Rotz*R_gripper_down_;
            ROS_INFO_STREAM("specifying gripper orientation:  " << endl << Rot_gripper << endl);
            grasp_pose.linear() = Rot_gripper;
            grasp_pose.translation() = O_part;
            break;
        case part_codes::part_codes::SMALL_GEAR:
            //grab part at centroid:
            //hard-coded grasp heigth from Fetch remote experiments
            O_part << part_pose.pose.position.x, part_pose.pose.position.y, GEAR_GRASP_HEIGHT; //part_pose.pose.position.z + TOTE_GRASP_HEIGHT_WRT_TOTE_ORIGIN;
            //double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
            object_angle = xformUtils.convertPlanarQuat2Phi(part_pose.pose.orientation); 
            Rotz = compute_rot_z(object_angle);
            ROS_INFO_STREAM("Rotz: "<<endl<<Rotz<<endl);
            ROS_INFO_STREAM("R_gripper_down: "<<endl<<R_gripper_down_);
            Rot_gripper = Rotz*R_gripper_down_;
            ROS_INFO_STREAM("specifying gripper orientation:  " << endl << Rot_gripper << endl);
            grasp_pose.linear() = Rot_gripper;
            grasp_pose.translation() = O_part;
            break;
        case part_codes::part_codes::LARGE_GEAR:
            //grab part at centroid:
            //hard-coded grasp heigth from Fetch remote experiments
            O_part << part_pose.pose.position.x, part_pose.pose.position.y, GEAR_GRASP_HEIGHT; //part_pose.pose.position.z + TOTE_GRASP_HEIGHT_WRT_TOTE_ORIGIN;
            //double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
            object_angle = xformUtils.convertPlanarQuat2Phi(part_pose.pose.orientation); 

            Rotz = compute_rot_z(object_angle);
            ROS_INFO_STREAM("Rotz: "<<endl<<Rotz<<endl);
            ROS_INFO_STREAM("R_gripper_down: "<<endl<<R_gripper_down_);
            Rot_gripper = Rotz*R_gripper_down_;
            ROS_INFO_STREAM("specifying gripper orientation:  " << endl << Rot_gripper << endl);
            grasp_pose.linear() = Rot_gripper;
            grasp_pose.translation() = O_part;
            break;            
        default:
            ROS_WARN("unknown part code in compute_grasp_affine:");

    }

    return grasp_pose;

}
//specified source_pose should use torso_lift_link frame to specify part pose
//THIS FNC SHOULD TRANSFORM from  source reference to torso_lift_link reference frame...
// NOT YET IMPLEMENTED, SO USER BEWARE
// part major axis should be x-axis; gripper will be oriented s.t. line between fingers
// is perpendicular to part major axis
//ALSO, derive desired gripper approach orientation from part orientation...NOT DONE YET!!!

//OOPS! Tote frame is defined such that handle is along y-axis, so need gripper axis s.t.
// line from thumb to forefinger is PARALLEL to x-axis 
bool MovePart::get_part(int part_code, geometry_msgs::PoseStamped source_pose) {

    //O_des_ << PART_X_VAL, PART_Y_VAL, APPROACH_HT; //0.5, 0.4, 0.2; //0.3,-0.1,0.0;
    //O_des_ << perceived_object_pose.pose.position.x,perceived_object_pose.pose.position.y,APPROACH_HT;
    //O_des_ << source_pose.pose.position.x,source_pose.pose.position.y,source_pose.pose.position.z+APPROACH_CLEARANCE;
 
    //FIX ME!!!  need correct orientation
    //part_codes::part_codes::TOTE:
    //use this function to compute where the hand should be (w/rt torso) to grasp the specified part that is at part_pose
    //Eigen::Affine3d grasp_affine = compute grasp_affine(int part_code, geometry_msgs::PoseStamped part_pose)
    geometry_msgs::PoseStamped grasp_pose,approach_pose;
    Eigen::Affine3d grasp_affine,approach_affine;
    grasp_affine = compute_grasp_affine( part_code, source_pose); //this is the gripper pose suitable for grasp of indicated object
    //approach_affine = grasp_affine;
    
    //tool_affine_.linear() = R_gripper_down_;
    //tool_affine_.translation() = O_des_;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    //tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine,"system_ref_frame");

    grasp_pose = xformUtils.transformEigenAffine3dToPoseStamped(grasp_affine, "torso_lift_link");  
    approach_pose = grasp_pose;  //set z-height of approach pose to be higher than grasp pose
    approach_pose.pose.position.z = approach_pose.pose.position.z+APPROACH_HT; //APPROACH_CLEARANCE; //descend to grasp pose
    
    bool traj_is_valid = false;
    int rtn_code;

    int nsteps = 10;
    double arrival_time = 2.0;
    int rtn_val;
    
    //Eigen::Affine3d approach_affine;
    //approach_pose = 

    ROS_INFO("requesting plan to approach pose:");
    rtn_val = cart_motion_commander_.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, approach_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander_.execute_planned_traj();
        //ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        return false;
    }
    

    //tool_pose_.pose.position.z = GRASP_HEIGHT; //descend to grasp pose
    ROS_INFO("requesting plan descend to grasp pose:");
    xformUtils.printPose(grasp_pose);
    rtn_val = cart_motion_commander_.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, grasp_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander_.execute_planned_traj();
        //ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        return false;
    }    
    gripper_interface_.graspObject();
    ros::Duration(3.0).sleep();  //have to wait on gripper; get some feedback??
    
    //   tool_pose_.pose.position.z = source_pose.pose.position.z+APPROACH_CLEARANCE; //descend to grasp pose
    ROS_INFO("requesting plan depart with grasped object:");
    xformUtils.printPose(approach_pose);
    rtn_val = cart_motion_commander_.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, approach_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander_.execute_planned_traj();
        //ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        return false;
    }  
        
return true;
}

bool MovePart::place_grasped_part(int part_code, geometry_msgs::PoseStamped destination_pose){
    gripper_interface_.releaseObject();
return false;
}

bool MovePart::release_grasped_part(){
    gripper_interface_.releaseObject();
return false;
}

bool MovePart::grasp_part(){
    gripper_interface_.graspObject();
return false;
}


bool MovePart::stow_grasped_part(int part_code) {
 return false;
}


