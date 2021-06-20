// example action client for Fetch robot action  server 
//routine to pre-position arm for suitable grasp version
// wsn,  2/2019

//pose for arm out of camera view, but ready to swing into position above table
//(should pre-position camera to tilt down, e.g. 1.0 rad, and rais up torso to max, 0.385)
//q_in:   1.57     -1      0 1.5707      0      1      0: gripper is down, arm is to left; should be safe to swing to:
//actually, q_shoulder_pan = 0.8 is good enough to be out of view
//q_in:      0     -1      0 1.5707      0      1      0: gripper is in front, pointing down, above table;
//   the above pose is safe for Cartesian moves in x,y, but it blocks the camera view; need to make move gently to not displace robot
//rosrun tf tf_echo base_link gripper_link:  
//- Translation: [0.610, 0.000, 0.989]
//- Rotation: in Quaternion [-0.000, 0.707, 0.000, 0.707]


#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Eigen>
#include <fetch_fk_ik/fetch_kinematics.h>
using namespace std;

int g_done_count = 0; //flag to indicate action server has returned a result

void doneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    g_done_count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fetch_arm_client_node");
    ros::NodeHandle nh;
    Fetch_fwd_solver fwd_solver;
    Fetch_IK_solver ik_solver;
    std::vector<Eigen::VectorXd> q_solns;
    Eigen::Vector3d O_7,O_7_des;
    
    Eigen::Affine3d A_fwd_DH;
    Eigen::VectorXd q_ARM_INIT,q_ARM_INIT2; //
    q_ARM_INIT.resize(7); 
    q_ARM_INIT2.resize(7);
    q_ARM_INIT<<1.5,     2,      -1, 1.57,      0,      1.5,      0;
    q_ARM_INIT2<<0.8,     -1,      0, 1.5707,      0,      1,      0;
    Eigen::VectorXd q_in;
    q_in.resize(7);
    control_msgs::FollowJointTrajectoryGoal robot_goal;
    //instantiate a goal message compatible with robot action server
    trajectory_msgs::JointTrajectory des_trajectory;
    //instantiate an action client of the robot-arm motion action server:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
            robot_motion_action_client("/arm_controller/follow_joint_trajectory", true);

    // attempt to connect to the server:
    /**/
    ROS_INFO("waiting for arm server: ");
    bool server_exists = robot_motion_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = robot_motion_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to arm action server"); // if here, then we connected to the server;  

    //populate a  trajectory message:
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.resize(8);
    for (int i = 0; i < 7; i++) 
        trajectory_point.positions[i] = q_ARM_INIT[i]; 

    des_trajectory.points.clear(); //not really necessary; just paranoid
    des_trajectory.joint_names.clear(); //ditto
    //set the joint names:
    //[shoulder_pan_joint, shoulder_lift_joint, upperarm_roll_joint, elbow_flex_joint, forearm_roll_joint,
    // wrist_flex_joint, wrist_roll_joint
  
    des_trajectory.joint_names.push_back("shoulder_pan_joint");
    des_trajectory.joint_names.push_back("shoulder_lift_joint");
    des_trajectory.joint_names.push_back("upperarm_roll_joint");
    des_trajectory.joint_names.push_back("elbow_flex_joint");
    des_trajectory.joint_names.push_back("forearm_roll_joint");
    des_trajectory.joint_names.push_back("wrist_flex_joint");
    des_trajectory.joint_names.push_back("wrist_roll_joint");


    //set arrival time for a single point in the trajectory:
    trajectory_point.time_from_start = ros::Duration(2.0); //allow 2 sec for this move
    des_trajectory.points.push_back(trajectory_point);
    
    
    //one one or more trajectory points to the trajectory:
        //q_ARM_INIT:    
    
    for (int i = 0; i < 7; i++) { //copy over the joint-command values
        trajectory_point.positions[i] = q_ARM_INIT2[i];
    }
    trajectory_point.time_from_start = ros::Duration(5.0); //allow 3 sec for this move
    des_trajectory.points.push_back(trajectory_point);
    
    
    
    //put traj in goal message
    robot_goal.trajectory = des_trajectory;

    /**/
    ROS_INFO("sending goal to arm: ");
    robot_motion_action_client.sendGoal(robot_goal, &doneCb);
    g_done_count = 0;
    while (g_done_count < 1) {
        ROS_INFO("waiting to finish pre-pose..");
        ros::Duration(1.0).sleep();
    }
 
    ROS_INFO("arm is in pre-pose");



    return 0;
}

