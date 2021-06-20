// example action client for Fetch robot action  server 
///torso_controller/follow_joint_trajectory/goal
// wsn,  2/2019

//max value of torso joint is 0.386; min value is 0


#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Eigen>
using namespace std;

int g_done_count = 0; //flag to indicate action server has returned a result

void doneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    g_done_count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fetch_torso_client_node");
    ros::NodeHandle nh;
    
    Eigen::VectorXd q_HEAD_INIT,q_HEAD_INIT2,q_HEAD_INIT3; //
    q_HEAD_INIT.resize(2); 
    q_HEAD_INIT2.resize(2);
    q_HEAD_INIT3.resize(2);
    q_HEAD_INIT<<0.0,0.0;
    q_HEAD_INIT2<<0.0,1;
    q_HEAD_INIT3<<1.0,0;
    
    control_msgs::FollowJointTrajectoryGoal robot_goal;
    //instantiate a goal message compatible with robot action server
    trajectory_msgs::JointTrajectory des_trajectory;
    //instantiate an action client of the robot-arm motion action server:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
            robot_motion_action_client("/torso_controller/follow_joint_trajectory", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for torso server: ");
    bool server_exists = robot_motion_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on torso server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = robot_motion_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to torso action server"); // if here, then we connected to the server;  
    double torso_jnt_q;
    
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.resize(1);
    des_trajectory.points.clear(); //not really necessary; just paranoid
    des_trajectory.joint_names.clear(); //ditto
    //set the joint names:
    des_trajectory.joint_names.push_back("torso_lift_joint");
    //des_trajectory.joint_names.push_back("head_tilt_joint");

    //set arrival time for a single point in the trajectory:
    trajectory_point.time_from_start = ros::Duration(2.0); //allow 2 sec for this move
    
    while(true) {
        cout<<"enter a torso joint value: ";
        cin>>torso_jnt_q;
        trajectory_point.positions[0] = torso_jnt_q;
        des_trajectory.points.clear();
        des_trajectory.points.push_back(trajectory_point);
        //put traj in goal message
        robot_goal.trajectory = des_trajectory;

        ROS_INFO("sending goal to torso: ");
        robot_motion_action_client.sendGoal(robot_goal, &doneCb);        
    }
 

    return 0;
}
