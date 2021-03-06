// example_generic_cart_move_ac: 
// wsn, Nov, 2016; updated 12/17
// illustrates use of a generic action client that communicates with
// an action server called "cartMoveActionServer"
// the actual action server can be customized for a specific robot, whereas
// this client is robot agnostic

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_arm_cart_move_ac"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    CartMotionCommander cart_motion_commander;
    XformUtils xformUtils;
    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    int nsteps;
    double arrival_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;

    bool traj_is_valid = false;
    int rtn_code;
    
    nsteps = 10;
    arrival_time = 2.0;
    

    Eigen::Vector3d b_des,n_des,t_des,O_des;
    Eigen::Matrix3d R_gripper;
    b_des << 0, 0, -1;
    n_des << 1, 0, 0;
    t_des = b_des.cross(n_des);
    
    R_gripper.col(0) = n_des;
    R_gripper.col(1) = t_des;
    R_gripper.col(2) = b_des;
    
    //pre-pose: rosrun tf tf_echo system_ref_frame generic_gripper_frame
    //  0.435, 0.414, 0.604
    //rosrun tf tf_echo torso_lift_link generic_gripper_frame:  0.522, 0.414, 0.226
    O_des<<0.5, 0.4, 0.2; //0.3,-0.1,0.0;
    Eigen::Affine3d tool_affine;
    tool_affine.linear() = R_gripper;
    tool_affine.translation()= O_des;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    //tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine,"system_ref_frame");
    tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine,"torso_lift_link");
    ROS_INFO("requesting plan to gripper-down pose:");
    xformUtils.printPose(tool_pose);
    ROS_INFO_STREAM("Rdes = "<<endl<<tool_affine.linear()<<endl);
    rtn_val=cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps,arrival_time,tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val=cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time+0.2).sleep(); 
    }
    else {
        ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
    }    
    
    //start multi-traj planning:
    int nsegs = 0;
    vector<double> arrival_times;
    
    tool_pose.pose.position.y+=0.2;
     ROS_INFO("requesting plan move along y axis:");
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    } 
    else {
       nsegs++;
       arrival_times.push_back(arrival_time);   
    }
    
//    int append_multi_traj_cart_segment(int nsteps, double arrival_time, geometry_msgs::PoseStamped des_pose);
    tool_pose.pose.position.x+=0.1;    
    rtn_val=cart_motion_commander.append_multi_traj_cart_segment(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    }
    else {
       nsegs++;
       arrival_times.push_back(arrival_time);   
    }
    tool_pose.pose.position.y-=0.2;
     ROS_INFO("requesting plan move along -y axis:");
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.append_multi_traj_cart_segment(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    }     
    else {
       nsegs++;
       arrival_times.push_back(arrival_time);   
    }
    tool_pose.pose.position.x+=0.1;    
    rtn_val=cart_motion_commander.append_multi_traj_cart_segment(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    }
    else {
       nsegs++;
       arrival_times.push_back(arrival_time);   
    }
    tool_pose.pose.position.y-=0.2;
     ROS_INFO("requesting plan move along -y axis:");
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.append_multi_traj_cart_segment(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    }     
    else {
       nsegs++;
       arrival_times.push_back(arrival_time);   
    }    

    //O_des<<0.5, 0.0, 0.2; //this is too close; arm hits head
    tool_pose.pose.position.x = 0.7;
    tool_pose.pose.position.y = 0.0;
    tool_pose.pose.position.z = 0.2;
    ROS_INFO("requesting plan move to 0.7, 0.9, 0.2");
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.append_multi_traj_cart_segment(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    }     
    else {
       nsegs++;
       arrival_times.push_back(arrival_time);   
    }    

    tool_pose.pose.position.z -= 0.1;
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.append_multi_traj_cart_segment(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    }     
    else {
       nsegs++;
       arrival_times.push_back(arrival_time);   
    }   

    //execute multi-traj:
    double wait_time;
    for (int iseg=0;iseg<nsegs;iseg++) {
        ROS_INFO("commanding seg %d",iseg);
        //wait_time = arrival_times[iseg];
        rtn_val = cart_motion_commander.execute_traj_nseg(iseg);
        //ros::Duration(wait_time).sleep();
    }

    int xyz_num;
   xyz_num=0;
   while(xyz_num>=0)  {
    cout<<"enter 0,1,2 for incremental move in x,y,z: ";
    cin>>xyz_num;
    cout<<"enter desired ds (in m): ";
    double ds;
    cin>>ds;
    if (xyz_num==0) tool_pose.pose.position.x += ds;
    if (xyz_num==1) tool_pose.pose.position.y += ds;
    if (xyz_num==2) tool_pose.pose.position.z += ds;
    ROS_INFO("desired tool pose: ");
    xformUtils.printPose(tool_pose);
    arrival_times.clear();
    //note: bug in plan_cartesian_traj_qprev_to_des_tool_pose(); if no valid IK soln along path, action server crashes!
    //...first call only???
    // for single move (not multisegment), use plan_cartesian_traj_current_to_des_tool_pose
    // and TODO: find/fix this bug
    rtn_val=cart_motion_commander.plan_cartesian_traj_current_to_des_tool_pose(nsteps, arrival_time,tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val=cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time+0.2).sleep(); 
    }
    else {
        ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
    }    
 }

    return 0;
}

