// fetch_cartesian_interactive_ac.cpp: 
// wsn, April, 2019

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

const double GRASP_HEIGHT = 0.055;
const double PART_X_VAL = 0.625; //0.64;
const double PART_Y_VAL = 0.2; //0.059; //minus-sign error w/ object-finder?
const double APPROACH_HT = 0.15;
//- Translation: [0.625, 0.500, 0.250]
const double WAITING_X_VAL = 0.625;
const double WAITING_Y_VAL = 0.5;
const double WAITING_Z_VAL = 0.25;


Eigen::Matrix3d compute_rot_z(double angle) {
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


int main(int argc, char** argv) {
    ros::init(argc, argv, "fetch_cartesian_interactive_ac"); // name this node 
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


    Eigen::Vector3d b_des, n_des, t_des, O_des;
    Eigen::Matrix3d R_gripper_down,Rot_gripper,Rotz;
    b_des << 0, 0, -1;
    n_des << 1, 0, 0;
    t_des = b_des.cross(n_des);

    R_gripper_down.col(0) = n_des;
    R_gripper_down.col(1) = t_des;
    R_gripper_down.col(2) = b_des;
    

    //pre-pose: rosrun tf tf_echo system_ref_frame generic_gripper_frame
    //  0.435, 0.414, 0.604
    //rosrun tf tf_echo torso_lift_link generic_gripper_frame:  0.522, 0.414, 0.226
    //O_des << PART_X_VAL, PART_Y_VAL, APPROACH_HT; //0.5, 0.4, 0.2; //0.3,-0.1,0.0;
    O_des <<WAITING_X_VAL,WAITING_Y_VAL,WAITING_Z_VAL;
    Eigen::Affine3d tool_affine;
    tool_affine.linear() = R_gripper_down;
    tool_affine.translation() = O_des;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    //tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine,"system_ref_frame");
    tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "torso_lift_link");

    ROS_INFO("requesting plan to approach pose:");
    xformUtils.printPose(tool_pose);
    ROS_INFO_STREAM("Rdes = " << endl << tool_affine.linear() << endl);
    rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }
    /*
    int nsegs = 0;
    vector<double> arrival_times;

    tool_pose.pose.position.z = GRASP_HEIGHT; //descend to grasp pose
    ROS_INFO("requesting plan descend to grasp pose:");
    xformUtils.printPose(tool_pose);
    rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }

    ROS_INFO("requesting plan move depart:");

    tool_pose.pose.position.z = APPROACH_HT;
    rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }
     */
    string xyz("x");
    double ds;
    bool valid_code;
    int cart_code;
    while (true) {
        cout << "enter x, y, z or r for desired Cartesian relative motion: " << endl;
        cin>>xyz;
        valid_code = true;
        if (xyz == "x") cart_code = 0;
        else if (xyz == "y") cart_code = 1;
        else if (xyz == "z") cart_code = 2;
        else if (xyz == "r") cart_code = 3;
        else {
            ROS_WARN("not a valid input: %s", xyz.c_str());
            valid_code = false;
        }
        if (valid_code) {
            cout<<" using code "<<cart_code<<endl;  
            cout << "enter desired displacement value (in m or radians):  ";
            cin>>ds;

            if (cart_code == 0) O_des[0] += ds; //tool_pose.pose.position.x += ds;
            if (cart_code == 1) O_des[1] += ds; //tool_pose.pose.position.y += ds;
            if (cart_code == 2) O_des[2] += ds; //tool_pose.pose.position.z += ds;
            //ROS_INFO("desired tool pose: ");

            if (cart_code == 3) {
               Rotz = compute_rot_z(ds);
               // ROS_INFO_STREAM("Rotz: "<<endl<<Rotz<<endl);
               // ROS_INFO_STREAM("R_gripper_down: "<<endl<<R_gripper_down_);
                Rot_gripper = Rotz*R_gripper_down;//R_gripper_down
                ROS_INFO_STREAM("specifying gripper orientation:  " << endl << Rot_gripper << endl);
                tool_affine.linear() = Rot_gripper; //grasp_pose.linear() = Rot_gripper;
                
            }
            tool_affine.translation() = O_des;
            tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "torso_lift_link");
            xformUtils.printPose(tool_pose);
            rtn_val = cart_motion_commander.plan_cartesian_traj_current_to_des_tool_pose(nsteps, arrival_time, tool_pose);
            if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
                ROS_INFO("successful plan; command execution of trajectory");
                rtn_val = cart_motion_commander.execute_planned_traj();
                //ros::Duration(arrival_time + 0.2).sleep();
            } else {
                ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
            }
        }
    }
    return 0;
}

