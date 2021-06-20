// example action client for Fetch robot action  server 
//interactive version
// wsn,  2/2019

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
    q_ARM_INIT<<1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0;
    q_ARM_INIT2<<0.0, -0.62, 0.0, 0.0, 0.0, 0.62, 0.0;
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
        trajectory_point.positions[i] = q_ARM_INIT2[i]; 

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
        trajectory_point.positions[i] = q_ARM_INIT[i];
    }
    trajectory_point.time_from_start = ros::Duration(5.0); //allow 3 sec for this move
    des_trajectory.points.push_back(trajectory_point);
    
    
    
    //put traj in goal message
    robot_goal.trajectory = des_trajectory;

    /**/
    ROS_INFO("sending goal to arm: ");
    robot_motion_action_client.sendGoal(robot_goal, &doneCb);

    while (g_done_count < 1) {
        ROS_INFO("waiting to finish pre-pose..");
        ros::Duration(1.0).sleep();
    }
 
        int jnum;
        double qval;
    trajectory_point.time_from_start = ros::Duration(2.0);
    for (int i=0;i<7;i++) q_in[i]=trajectory_point.positions[i];
    while (true) {
        cout<<"enter jnt num, 0-6: ";
        cin>>jnum;
        cout<<"enter jnt val: ";
        cin>>qval;
        des_trajectory.points.clear();
        trajectory_point.positions[jnum] = qval;
        q_in[jnum]=qval;
        des_trajectory.points.push_back(trajectory_point);
        robot_goal.trajectory = des_trajectory;
        robot_motion_action_client.sendGoal(robot_goal, &doneCb);
        A_fwd_DH = fwd_solver.fwd_kin_solve(q_in); //fwd_kin_solve

        std::cout << "q_in: " << q_in.transpose() << std::endl;        

        std::cout << "A rot: " << std::endl;
        std::cout << A_fwd_DH.linear() << std::endl;
        std::cout << "A origin (torso_lift_link to gripper_frame): " << A_fwd_DH.translation().transpose() << std::endl;
        
        double q_shoulder_pan = q_in[0];
        //int nsolns = ik_solver.ik_solve(A_fwd_DH,q_shoulder_pan,q_solns);
        //following ASSUMES q1=0
        //int nsolns = ik_solver.ik_solve(A_fwd_DH,q_solns);
        
        //int nsolns = ik_solver.ik_solve_elbow_up_given_q1(A_fwd_DH,q_shoulder_pan,q_solns);     
        int nsolns = ik_solver.ik_solve_simple_reach(A_fwd_DH,q_solns);      

        //ROS_INFO_STREAM("desired wrist point: "<<wrist_pt.transpose()<<endl);

        
        
        nsolns = q_solns.size();
        std::cout << "number of IK solutions: " << nsolns << std::endl;    
        
        //test fwd kin:
        std::cout << "q_in: " << q_in.transpose() << std::endl;        
        

        O_7_des = A_fwd_DH.translation();
        ROS_INFO_STREAM("desired hand position: " <<O_7_des.transpose() <<endl);
        

        ROS_INFO("test solns: ");
        for (int i=0;i<nsolns;i++) {
            ROS_INFO_STREAM("q_soln: "<<q_solns[i].transpose()<<endl);
            A_fwd_DH = fwd_solver.fwd_kin_solve(q_solns[i]);
            O_7 = A_fwd_DH.translation();
            double hand_err = (O_7_des-O_7).norm();
            ROS_INFO_STREAM("fwd kin hand position err: " <<hand_err <<endl);
        }        
    }

    return 0;
}
