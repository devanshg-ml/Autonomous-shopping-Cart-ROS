/* 
 * File:   fetch_kinematics.h
 * Author: wsn
 *
 * Created Feb, 2019
 */

/*
fetch params:
link	a	d	alpha	q_offset
1	0.117	0.06	pi/2	-pi/2 	lower=\"-1.6056\" upper=\"1.6056\
2	0	0	-pi/2	0	lower=\"-1.221\" upper=\"1.518\
3	0	d3	-pi/2	0	continuous
4	0	0	pi/2	0	lower=\"-2.251\" upper=\"2.251
5	0	d5	-pi/2	0	continuous
6	0	0	pi/2	0	lower= -2.16\" upper=\"2.16
7	0	d7	pi	0	continuous
 * 



vel limits, 1.256, 1.454, 1.571, 1.521, 1.571, 2.268, 2.268

<joint name=\"shoulder_pan_joint\"\
  \ type=\"revolute\">\n    <origin rpy=\"0 0 0\" xyz=\"0.119525 0 0.34858\"/>\n \
  \   <parent link=\"torso_lift_link\"/>\n    <child link=\"shoulder_pan_link
<joint name=\"shoulder_lift_joint\"\
  \ type=\"revolute\">\n    <origin rpy=\"0 0 0\" xyz=\"0.117 0 0.0599999999999999\"\
  />\n    <parent link=\"shoulder_pan_link\"/>\n    <child link=\"shoulder_lift_link

 choose base frame consistent with shoulder_pan_link
 make fk consistent with:
 rosrun tf tf_echo shoulder_pan_link generic_gripper_frame (w/ origin same as gripper_link, but reoriented)
 * 
 2/14/19: added A_shoulder_wrt_torso_, so fk is computed with respect to torso_lift_link
 
 rosrun tf tf_echo torso_lift_link gripper_link  origin is now consistent with fk
 BUT, hand to fix alphas: roll positive rotation is NOT as shown in fig!!


*/

#ifndef FETCH_IK_H
#define	FETCH_IK_H
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>


const int NJNTS=7;
const int NJNTS_TO_WRIST=5;
const int MAX_SOLNS_RETURNED=100; //tune me!
const double ELBOW_SINGULARITY_THRESHOLD = 0.001; // outstretched is singularity
const double W_ERR_THRESHOLD = 0.0001; // test for correct IK solns

const double DZ_TORSO_LIFT_TO_SHOULDER_PAN = 0.34858;
const double DX_TORSO_LIFT_TO_SHOULDER_PAN = 0.119525;

// a fwd kin solver...
// list DH params here

const double DH_a1=0.117;
const double DH_a2=0.0;
const double DH_a3=0.0;
const double DH_a4=0.0;
const double DH_a5=0.0;
const double DH_a6=0.0;
const double DH_a7=0.0;


const double DH_d1 = 0.06; //
const double DH_d2 = 0.0;
const double DH_d3 = 0.352;
const double DH_d4 = 0.0;
const double DH_d5 = 0.3215;
const double DH_d6 = 0.0;
const double DH_d7 = 0.30495;


const double DH_alpha1 = -M_PI/2.0;
const double DH_alpha2 = -M_PI/2.0;
const double DH_alpha3 = M_PI/2.0;
const double DH_alpha4 = -M_PI/2.0;
const double DH_alpha5 = M_PI/2.0;
const double DH_alpha6 = -M_PI/2.0;
const double DH_alpha7 = 0.0; //M_PI;

//robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
const double DH_q_offset1 = 0;
const double DH_q_offset2 = -M_PI/2.0;
const double DH_q_offset3 = 0.0;
const double DH_q_offset4 = 0.0;
const double DH_q_offset5 = 0.0;
const double DH_q_offset6 = 0;
const double DH_q_offset7 = 0;

const double deg2rad = M_PI/180.0;

const double DH_q_max1 = 1.6056;
const double DH_q_max2 = 1.518;
const double FETCH_qmax2 = 1.518;
const double DH_q_max3 = M_PI; //continuous
const double DH_q_max4 = 2.251;
const double FETCH_ELBOW_MAX = DH_q_max4; //synonym
const double DH_q_max5 = M_PI; //continuous
const double DH_q_max6 = 2.16; 
const double FETCH_WRIST_BEND_MAX = DH_q_max6;
const double DH_q_max7 = M_PI; //continuous

const double DH_q_min1 = -1.6056;
const double DH_q_min2 = -1.221; //THIS IS ACTUALLY FETCH Q_MIN
const double FETCH_qmin2 = -1.22; 
const double DH_q_min3 = -M_PI; //
const double DH_q_min4 = -2.251;
const double FETCH_ELBOW_MIN = DH_q_min4;
const double DH_q_min5 = -M_PI;
const double DH_q_min6 = -2.16; 
const double FETCH_WRIST_BEND_MIN = DH_q_min6;
const double DH_q_min7 = -M_PI; 


const double DH_a_params[]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5,DH_a6,DH_a7};
const double DH_d_params[] = {DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6, DH_d7};
const double DH_alpha_params[] = {DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6, DH_alpha7};
const double DH_q_offsets[] = {DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6,DH_q_offset7};
const double q_lower_limits[] = {DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6, DH_q_min7};
const double q_upper_limits[] = {DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6, DH_q_max7};
const double g_qdot_max_vec[] = {1.256, 1.454, 1.571, 1.521, 1.571, 2.268, 2.268}; //values per URDF 
const double g_q_home_pose[] = {0,0,0,0,0,0,0};
const double g_q_waiting_pose[] = {0.8,     -1,      0, 1.5707,      0,      1,      0};
//put these in planner_joint_weights.h
//const double jspace_planner_weights[] = {5,5,3,0.5,0.2,0.2}; //default weights for jspace planner (changeable in planner)


class Fetch_fwd_solver {
public:
    Fetch_fwd_solver(); //constructor; //const hand_s& hs, const atlas_frame& base_frame, double rot_ang);
    //atlas_hand_fwd_solver(const hand_s& hs, const atlas_frame& base_frame);
    //Eigen::Affine3d fwd_kin_solve(const Eigen::VectorXd& q_vec); // given vector of q angles, compute fwd kin
    Eigen::Affine3d fwd_kin_solve(const Eigen::VectorXd& q_vec); 
    Eigen::Matrix4d get_wrist_frame();
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q_vec);
    //Eigen::MatrixXd get_Jacobian(const Eigen::VectorXd& q_vec);
    //void  test_q1234(std::vector<Eigen::VectorXd> q_solns);
    void  test_q1234(Eigen::VectorXd q_soln);

    Eigen::Vector3d compute_wrist_point(Eigen::VectorXd q_vec);
    Eigen::Vector4d compute_O5_wrt_3(double q_elbow);
    Eigen::Vector4d compute_O5_wrt_2(Eigen::Vector4d O_5_wrt_3,  double q_humerus_roll);
    Eigen::Vector4d compute_O5_wrt_1(Eigen::Vector4d O_5_wrt_2,  double q_shoulder_pitch);
    Eigen::Matrix4d compute_A_of_DH(int i, double q_fetch);
    Eigen::Affine3d Affine_shoulder_wrt_torso_inv_;


private:
    Eigen::Matrix4d fwd_kin_solve_(const Eigen::VectorXd& q_vec);
    Eigen::Matrix4d A_mats_[NJNTS], A_mat_products_[NJNTS], A_tool_,A_shoulder_wrt_torso_,A_shoulder_wrt_torso_inv_; // note: tool A must also handle diff DH vs URDF frame-7 xform
    Eigen::MatrixXd Jacobian_;

};

class Fetch_IK_solver: public Fetch_fwd_solver {
public:
    Fetch_IK_solver(); //constructor; 

    // return the number of valid solutions; actual vector of solutions will require an accessor function
    //int ik_solve(Eigen::Affine3d const& desired_hand_pose); // given desired pose, compute IK
    int ik_solve(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns);
    int ik_solve(Eigen::Affine3d const& desired_hand_pose, double q_shoulder_pan, std::vector<Eigen::VectorXd> &q_ik_solns);
    int ik_solve_elbow_up_given_q1(Eigen::Affine3d const& desired_hand_pose, double q_shoulder_pan, std::vector<Eigen::VectorXd> &q_ik_solns);
    //the next version assumes q1 computed to point arm towards desired hand position, then solve w/ elbow up
    int ik_solve_simple_reach(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns);
 
    void get_solns(std::vector<Eigen::VectorXd> &q_solns);
    bool fit_joints_to_range(Eigen::VectorXd &qvec);
    bool solve_Asin_plus_Bcos_eqC(double A,double B,double C, double thetas[]);
    
    //Eigen::MatrixXd get_Jacobian(const Eigen::VectorXd& q_vec);
private:
    bool fit_q_to_range(double q_min, double q_max, double &q);    

    Eigen::Matrix4d A_mats[NJNTS], A_mat_products[NJNTS], A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    double L_humerus_;
    double L_forearm_;
    double L_wrist_;
    double phi_elbow_;
    std::vector<Eigen::VectorXd> q_solns_fit_,q_solns_;
    //given desired hand pose and indexed value of shoulder_pan angle= q1,
    // fill up solns for  q2, q3, q4 based on wrist position  
    // populate q_solns, including specified q1
    bool compute_q234_solns(Eigen::Affine3d const& desired_hand_pose, double q_shoulder_pan, std::vector<Eigen::VectorXd> &q_solns);
    //restrict consideration to positive elbow solns:
    bool compute_q234_solns_elbow_up(Eigen::Affine3d const& desired_hand_pose, double q_shoulder_pan, std::vector<Eigen::VectorXd> &q_solns);

    bool solve_for_elbow_theta(double r_goal, double q_elbow_solns[]);  //
    bool solve_for_q_humerus_roll(Eigen::Vector3d w_wrt_1,double q_elbow_soln,double q_humerus_roll_solns[]);
    //next fnc not ready...
    //bool solve_for_shoulder_flex(Eigen::Vector3d w_wrt_1, double r_goal, double q_shoulder_solns[]);
    //bool solve_for_shoulder_flex(Eigen::Vector3d w_wrt_1, double q_elbow_solns[], double q_shoulder_solns[]);
    bool solve_for_shoulder_lift(Eigen::Vector3d w_wrt_1, double q_elbow, double q_shoulder_roll, double q_shoulder_lift[]);


    bool solve_spherical_wrist(Eigen::VectorXd q_in,Eigen::Matrix3d R_des, std::vector<Eigen::VectorXd> &q_solns);    
    //Eigen::MatrixXd Jacobian;
};


#endif	/* FETCH_IK_H */

