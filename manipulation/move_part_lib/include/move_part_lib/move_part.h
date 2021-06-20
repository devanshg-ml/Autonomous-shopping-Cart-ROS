#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fetch_fk_ik/fetch_kinematics.h>

//#include <object_finder/objectFinderAction.h>
#include <gripper_interface/gripper_interface.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <xform_utils/xform_utils.h>
#include <gripper_interface/gripper_interface.h>
#include <part_codes/part_codes.h>
#include <xform_utils/xform_utils.h>
#include <math.h>

using namespace std;
XformUtils xformUtils;

const double GRASP_HEIGHT = 0.055;
const double PART_X_VAL = 0.625; //0.64; for testing only
const double PART_Y_VAL = 0.2; //0.059; //minus-sign error w/ object-finder?
const double APPROACH_HT = 0.15;
const double APPROACH_CLEARANCE = 0.08;

const double TOTE_TABLE_HEIGHT_WRT_TORSO_LIFT_LINK = 0.065; //0.065 is simu value

const double TOTE_GRASP_HEIGHT_WRT_TOTE_ORIGIN = 0.095; //0.11; //TUNE ME! choose 0.095 for simu

//the following values are from remote Fetch experiments:
const double TOTE_GRASP_HEIGHT = 0.190; //handle is relatively high; elevate to about 0.280 to clear handle
const double GEAR_GRASP_HEIGHT = 0.120; //shunk station
const double GEARBOX_GRASP_HEIGHT = 0.140; // must approach close to table to grab gearbox part lying flat; also works on edge??
const double BOLT_GRASP_HEIGHT = 0.121; 

using namespace std;

class MovePart 
{
public:
	MovePart(); //constructor
        //public member fncs
        //GripperInterface gripperInterface_;
	bool get_part(int part_code, geometry_msgs::PoseStamped source_pose);
        bool place_grasped_part(int part_code, geometry_msgs::PoseStamped destination_pose);
        bool stow_grasped_part(int part_code);
        bool move_to_dropoff_kit1();
        bool move_to_dropoff_kit2();
        bool move_to_dropoff_kit3();
        bool move_to_dropoff_tote();
        bool move_to_pickup_tote();
        
        bool recover_from_dropoff();
        bool recover_from_tote();

        bool preset_arm();
        bool release_grasped_part();
        bool grasp_part();
        
        Eigen::Affine3d  compute_grasp_affine(int part_code, geometry_msgs::PoseStamped part_pose);
        Eigen::Matrix3d compute_rot_z(double angle);
        //bool preset_torso();
        //bool preset_head();
	CartMotionCommander cart_motion_commander_; //need this to talk to arm behavior server
        XformUtils xformUtils;
        GripperInterface gripper_interface_;
private:
        //private fncs and data
        ros::NodeHandle nh_;
    	void initializeSubscribers();
    	void initializePublishers();

        /*  REMOVE object finder dependency
       //object finder:
        actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac_;
        void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
            const object_finder::objectFinderResultConstPtr& result);
        object_finder::objectFinderGoal object_finder_goal_;
        bool found_object_code_; 
        std::vector <geometry_msgs::PoseStamped> perceived_object_poses_;
        */
    //for arm control:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> robot_arm_motion_action_client_;
    void armMotionCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result); 

    Fetch_fwd_solver fwd_solver_;
    Fetch_IK_solver ik_solver_;
    std::vector<Eigen::VectorXd> q_solns_;
    Eigen::Vector3d O_7_,O_7_des_;
    
    Eigen::Affine3d A_fwd_DH_;
    Eigen::VectorXd q_ARM_INIT_,q_ARM_INIT2_; //
    control_msgs::FollowJointTrajectoryGoal robot_arm_goal_;
    //goal message compatible with robot action server
    trajectory_msgs::JointTrajectory arm_des_trajectory_;

    Eigen::Matrix3d R_gripper_down_;
    Eigen::Vector3d b_des_, n_des_, t_des_, O_des_;
    Eigen::Affine3d tool_affine_;
    geometry_msgs::PoseStamped tool_pose_;//, tool_pose_home;

    //for torso:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_action_client_;
   void torsoDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result); 
    control_msgs::FollowJointTrajectoryGoal robot_torso_goal;
    //instantiate a goal message compatible with robot action server
    trajectory_msgs::JointTrajectory torso_des_trajectory;



    //for head:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_action_client_;
    void headDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    control_msgs::FollowJointTrajectoryGoal head_goal;
    //instantiate a goal message compatible with robot action server
    trajectory_msgs::JointTrajectory head_des_trajectory;


};
