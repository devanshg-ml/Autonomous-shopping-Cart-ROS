// fetch arm kinematics implementation file; start w/ fwd kin
// NOTE: DH 0 frame goes through z0 = shoulder pan joint axis
// choose a reference frame consistent w/ URDF torso_lift_link
// note that torso_lift_link is offset from base link by  xyz=-0.086875 0 0.37743
// AND additional z-height of torso_lift_joint displacement
// but should be able to reference point clouds to torso_lift_link frame and
// do arm kinematics w/rt torso_lift_frame



#include <fetch_fk_ik/fetch_kinematics.h>
using namespace std;

Fetch_fwd_solver::Fetch_fwd_solver() { 

    ROS_INFO("fwd_solver constructor");
    //A_shoulder_wrt_torso_
    Eigen::Vector4d p_shoulder_wrt_torso;
    p_shoulder_wrt_torso<<DX_TORSO_LIFT_TO_SHOULDER_PAN,0,DZ_TORSO_LIFT_TO_SHOULDER_PAN,1.0;
    A_shoulder_wrt_torso_<< Eigen::Matrix4d::Identity(4,4);
    A_shoulder_wrt_torso_.block<4,1>(0,3) = p_shoulder_wrt_torso;    
    //ROS_INFO_STREAM("A_shoulder_wrt_torso_:"<<endl<<A_shoulder_wrt_torso_<<endl);
    A_shoulder_wrt_torso_inv_ = A_shoulder_wrt_torso_.inverse();
    Eigen::Affine3d A(A_shoulder_wrt_torso_inv_);
    Affine_shoulder_wrt_torso_inv_ = A;
}

// function for use w/ both fwd and inv kin

//compute 4x4 transform matrices; just takes q in FETCH joint space and index of frame, 0 to 6, and uses global DH vars
Eigen::Matrix4d Fetch_fwd_solver::compute_A_of_DH(int i, double q_fetch) {
    Eigen::Matrix4d A;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    double a = DH_a_params[i];
    double d = DH_d_params[i];
    double alpha = DH_alpha_params[i];
    double q = q_fetch + DH_q_offsets[i];

    A = Eigen::Matrix4d::Identity();
    R = Eigen::Matrix3d::Identity();
    //ROS_INFO("compute_A_of_DH: a,d,alpha,q = %f, %f %f %f",a,d,alpha,q);

    double cq = cos(q);
    double sq = sin(q);
    double sa = sin(alpha);
    double ca = cos(alpha);
    R(0, 0) = cq;
    R(0, 1) = -sq*ca; //% - sin(q(i))*cos(alpha);
    R(0, 2) = sq*sa; //%sin(q(i))*sin(alpha);
    R(1, 0) = sq;
    R(1, 1) = cq*ca; //%cos(q(i))*cos(alpha);
    R(1, 2) = -cq*sa; //%	
    //%R(3,1)= 0; %already done by default
    R(2, 1) = sa;
    R(2, 2) = ca;
    p(0) = a * cq;
    p(1) = a * sq;
    p(2) = d;
    A.block<3, 3>(0, 0) = R;
    A.col(3).head(3) = p;
    return A;
}



Eigen::MatrixXd Fetch_fwd_solver::jacobian(const Eigen::VectorXd& q_vec) {
  Eigen::MatrixXd jacobian;
// ... do some work here FINISH ME!
  //jacobian = Fetch_fwd_solver.jacobian(q_vec);
  return jacobian;
}

/*  IN CASE WANT JACOBIAN LATER...
Eigen::MatrixXd irb140_hand_fwd_solver::get_Jacobian(const Vectorq6x1& q_vec) {
    solve(q_vec);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd J_ang = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd J_trans = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd zvecs = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd rvecs = Eigen::MatrixXd::Zero(3, 6);
    Eigen::Matrix4d Apalm = A_mat_products[7];
    Eigen::MatrixXd O_palm = Apalm.block<3, 1>(0, 3);
    Eigen::Matrix4d Ai;
    Eigen::MatrixXd zvec, rvec;
    Eigen::Vector3d t1, t2;
    for (int i = 0; i < 6; i++) {
        Ai = A_mat_products[i];
        zvec = Ai.block<3, 1>(0, 2); //%strip off z axis of each movable frame
        zvecs.block<3, 1>(0, i) = zvec; //%and store them
        rvec = O_palm - Ai.block<3, 1>(0, 3); //%vector from origin of i'th frame to palm 
        rvecs.block<3, 1>(0, i) = rvec;
        J_ang.block<3, 1>(0, i) = zvecs.block<3, 1>(0, i);

        t1 = zvecs.block<3, 1>(0, i);
        t2 = rvecs.block<3, 1>(0, i);
        J_trans.block<3, 1>(0, i) = t1.cross(t2);
    }

    J.block<3, 6>(0, 0) = J_trans;
    J.block<3, 6>(3, 0) = J_ang;
    if (is_lhand(hs_))return mirror_J_to_lhand(J);
    return J;
}

 */


/*
//return soln out to tool flange; would still need to account for tool transform for gripper
Eigen::Affine3d Fetch_fwd_solver::fwd_kin_solve(const Vectorq6x1& q_vec) {
    Eigen::Matrix4d M;
    M = fwd_kin_solve_(q_vec);
    Eigen::Affine3d A(M);
    return A;
}
*/


Eigen::Matrix4d Fetch_fwd_solver::get_wrist_frame() {
    return A_mat_products_[5];
}

Eigen::Vector3d Fetch_fwd_solver::compute_wrist_point(Eigen::VectorXd q_vec) {
    //Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    //%compute A matrix from frame i to frame i-1:
    Eigen::Matrix4d A_i_iminusi;
    //Eigen::Matrix3d R;
    Eigen::Vector3d w;
    for (int i = 0; i < NJNTS_TO_WRIST; i++) {
        //A_i_iminusi = compute_A_of_DH(DH_a_params[i],DH_d_params[i],DH_alpha_params[i], q_vec[i] + DH_q_offsets[i] );
        A_i_iminusi = compute_A_of_DH(i, q_vec[i]); 
        A_mats_[i] = A_i_iminusi;
        //std::cout << "A_mats[" << i << "]:" << std::endl;
        //std::cout << A_mats_[i] << std::endl;
    }

    A_mat_products_[0] = A_mats_[0];
    for (int i = 1; i < NJNTS_TO_WRIST; i++) {
        A_mat_products_[i] = A_mat_products_[i - 1] * A_mats_[i];
        //std::cout<<"A_mat_products_["<<i<<"]:"<<std::endl;
        //std::cout << A_mat_products_[i]<<std::endl;
    }
    w =  (A_mat_products_[NJNTS_TO_WRIST-1]).col(3).head(3); //wrist point
    return w;
}

Eigen::Vector4d    Fetch_fwd_solver::compute_O5_wrt_3(double q_elbow) {
    Eigen::Matrix4d A43 = compute_A_of_DH(3, q_elbow); 
    Eigen::Vector4d O5_wrt_4,O5_wrt_3;
    O5_wrt_4<<0,0,DH_d5,1;
    O5_wrt_3 = A43*O5_wrt_4;
    return O5_wrt_3;
}

Eigen::Vector4d Fetch_fwd_solver::compute_O5_wrt_2(Eigen::Vector4d O_5_wrt_3,  double q_humerus_roll) {
    Eigen::Matrix4d A32 = compute_A_of_DH(2, q_humerus_roll); 
    Eigen::Vector4d O5_wrt_2;
    O5_wrt_2 = A32*O_5_wrt_3;
    return O5_wrt_2;    
}

Eigen::Vector4d Fetch_fwd_solver::compute_O5_wrt_1(Eigen::Vector4d O_5_wrt_2,  double q_shoulder_pitch) {
    Eigen::Matrix4d A21 = compute_A_of_DH(1, q_shoulder_pitch); 
    Eigen::Vector4d O5_wrt_1;
    O5_wrt_1 = A21*O_5_wrt_2;
    return O5_wrt_1;    
}



//fwd kin fnc: accepts arg of type Eigen::VectorXd 
//USES FETCH COORDS
Eigen::Affine3d Fetch_fwd_solver::fwd_kin_solve(const Eigen::VectorXd& q_vec) {
    //ROS_INFO("called fwd_kin_solve...");
    Eigen::Matrix4d M;
    M = fwd_kin_solve_(q_vec);
    Eigen::Affine3d A(M);
    return A;
}

void  Fetch_fwd_solver::test_q1234(Eigen::VectorXd q_soln) {
  //int nsolns = q_solns.size();
  //Eigen::VectorXd qsoln;
  //Eigen::Matrix4d A_wrist;
    Eigen::Vector3d wrist_pt;

     ROS_INFO_STREAM("q_soln: "<<q_soln.transpose()<<endl);
     wrist_pt = compute_wrist_point(q_soln);
     cout<<"computed wrist point: "<<wrist_pt.transpose()<<endl;
}

//this fnc assumes q_vec in fetch coords (only diff is shoulder flex)s
Eigen::Matrix4d Fetch_fwd_solver::fwd_kin_solve_(const Eigen::VectorXd& q_vec) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d A_gripper_wrt_torso_link;
    //%compute A matrix from frame i to frame i-1:
    Eigen::Matrix4d A_i_iminusi;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    for (int i = 0; i < NJNTS; i++) {
        //A_i_iminusi = compute_A_of_DH(DH_a_params[i],DH_d_params[i],DH_alpha_params[i], q_vec[i] + DH_q_offsets[i] );
        A_i_iminusi = compute_A_of_DH(i, q_vec[i]); 
        A_mats_[i] = A_i_iminusi;
        //std::cout << "A_mats[" << i << "]:" << std::endl;
        //std::cout << A_mats_[i] << std::endl;
    }

    A_mat_products_[0] = A_mats_[0];
    for (int i = 1; i < NJNTS; i++) {
        A_mat_products_[i] = A_mat_products_[i - 1] * A_mats_[i];
        //std::cout<<"A_mat_products["<<i<<"]:"<<std::endl;
        //std::cout << A_mat_products_[i]<<std::endl;
    }
    //return A_mat_products_[NJNTS-1]; //gripper or flange frame
    A_gripper_wrt_torso_link = A_shoulder_wrt_torso_*A_mat_products_[NJNTS-1];
    
    return A_gripper_wrt_torso_link;
}

//for IK, assume a reference frame of torso_lift_link;
//transform to DH-0 frame before computing IK solns
Fetch_IK_solver::Fetch_IK_solver() {
    //constructor: 
    L_humerus_ = fabs(DH_d3); //DH_a_params[1];
    //double L3 = DH_d_params[3];
    //double A2 = DH_a_params[2];
    L_forearm_ = fabs(DH_d5); //L3; //sqrt(A2 * A2 + L3 * L3);
    L_wrist_ = fabs(DH_d7);
    q_solns_fit_.resize(NJNTS);
    q_solns_.resize(NJNTS);
    
    //for A2 = 0, do  not need this angle correction; set it to zero
    phi_elbow_=0.0; //acos((A2*A2+L_forearm*L_forearm-L3*L3)/(2.0*A2*L_forearm));
}


//return solns in Fetch joint space
int Fetch_IK_solver::ik_solve(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns) {
    //int nsolns =ik_solve(desired_hand_pose);
    //q_ik_solns = q_solns_fit_;
    int nsolns;
    q_ik_solns.clear();
    std::vector<Eigen::VectorXd> q_ik_solns_index_q1;
    //Eigen::Affine3d desired_hand_pose_wrt_DH0 = Affine_shoulder_wrt_torso_inv_*desired_hand_pose;
    //ROS_INFO_STREAM("desired hand origin w/rt DH0: "<<desired_hand_pose_wrt_DH0.translation().transpose()<<endl);
    //here, step through samples of q1 to get redundant options
    //for (double q1= -0.1; q1< 0.1; q1+= 0.1) {
    double q1 =0; {// TEST TEST TEST    
        ROS_INFO("trying q1 = %f",q1);
        ik_solve(desired_hand_pose,q1,q_ik_solns_index_q1); //solve q2 through q7 given q1
        int nsolns = q_ik_solns_index_q1.size();
        for (int j_soln=0;j_soln<nsolns;j_soln++) {
            q_ik_solns.push_back(q_ik_solns_index_q1[j_soln]);
        }
    }
    nsolns = q_ik_solns.size();
  return nsolns;
}

//for this case, have the shoulder-pan joint point towards the hand goal
int Fetch_IK_solver::ik_solve_simple_reach(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns) {
        q_ik_solns.clear();
        std::vector<Eigen::VectorXd> q_ik_solns1234;   
        q_ik_solns1234.clear();
        Eigen::Affine3d desired_hand_pose_wrt_DH0 = Affine_shoulder_wrt_torso_inv_*desired_hand_pose;
        Eigen::Vector3d desired_hand_origin_wrt_DH0 =  desired_hand_pose_wrt_DH0.translation();
        double q_shoulder_pan = atan2(desired_hand_origin_wrt_DH0[1],desired_hand_origin_wrt_DH0[0]);
        ROS_INFO("choosing q_shoulder_pan = %f",q_shoulder_pan);
        //ik_solve_elbow_up_given_q1(desired_hand_pose, q_shoulder_pan, q_ik_solns1234);
    bool reachable = compute_q234_solns_elbow_up(desired_hand_pose_wrt_DH0, q_shoulder_pan, q_ik_solns1234);
    if (!reachable) {
        return 0;
    }
    int    nsolns = q_ik_solns1234.size();
    //for (int isoln=0;isoln<nsolns;isoln++) {
    for (int isoln=0;isoln<1;isoln++) {        
            test_q1234(q_ik_solns1234[isoln]);
    }
    Eigen::VectorXd q_soln;
    q_soln.resize(NJNTS);
    std::vector<Eigen::VectorXd> q_wrist_solns;
    Eigen::Matrix3d R_des;
    R_des = desired_hand_pose_wrt_DH0.linear();
    reachable=false; //guilty until proven innocent
    for (int i=0;i<nsolns;i++) {
        q_soln = q_ik_solns1234[i];
        solve_spherical_wrist(q_soln,R_des, q_wrist_solns);
        int n_wrist_solns = q_wrist_solns.size();
        if (n_wrist_solns>0) {  
                for (int iwrist=0;iwrist<1;iwrist++) { //ONLY consider positive wrist bend...single soln
                    q_soln = q_wrist_solns[iwrist];
                    q_ik_solns.push_back(q_soln);
                    reachable = true; // note that we have at least one reachable solution
                }
            }
            //else {
            //    ROS_WARN("no wrist solns");
                //ROS_WARN("singularity in wrist solns!");
            //}
    }
    if (!reachable) {
        return 0;
    }
    
    //if here, have reachable solutions in q_ik_solns
    nsolns = q_ik_solns.size();
    return nsolns;        
}


//uses q_shoulder_pan provided and solves for remaining 6 jnt angles:
int Fetch_IK_solver::ik_solve(Eigen::Affine3d const& desired_hand_pose, double q_shoulder_pan, std::vector<Eigen::VectorXd> &q_ik_solns) {
    //q_solns_.clear();
    q_ik_solns.clear();
    std::vector<Eigen::VectorXd> q_ik_solns1234;
    Eigen::Affine3d desired_hand_pose_wrt_DH0 = Affine_shoulder_wrt_torso_inv_*desired_hand_pose;

    bool reachable = compute_q234_solns(desired_hand_pose_wrt_DH0, q_shoulder_pan, q_ik_solns1234);
    if (!reachable) {
        return 0;
    }
    int    nsolns = q_ik_solns1234.size();
    for (int isoln=0;isoln<nsolns;isoln++) {
            test_q1234(q_ik_solns1234[isoln]);
    }
    Eigen::VectorXd q_soln;
    q_soln.resize(NJNTS);
    std::vector<Eigen::VectorXd> q_wrist_solns;
    Eigen::Matrix3d R_des;
    R_des = desired_hand_pose_wrt_DH0.linear();
    reachable=false; //guilty until proven innocent
    for (int i=0;i<nsolns;i++) {
        q_soln = q_ik_solns1234[i];
               solve_spherical_wrist(q_soln,R_des, q_wrist_solns);
        int n_wrist_solns = q_wrist_solns.size();
        if (n_wrist_solns>0) {  
                for (int iwrist=0;iwrist<1;iwrist++) { //ONLY consider positive wrist bend...single soln
                    q_soln = q_wrist_solns[iwrist];
                    q_ik_solns.push_back(q_soln);
                    reachable = true; // note that we have at least one reachable solution
                }
            }
            //else {
            //    ROS_WARN("no wrist solns");
                //ROS_WARN("singularity in wrist solns!");
            //}
    }

    if (!reachable) {
        return 0;
    }
    
    //if here, have reachable solutions in q_ik_solns
    nsolns = q_ik_solns.size();
    return nsolns;    
    
}


//restricts solns to q_elbow>0
//will return 2 solns (or 0 solns); first soln will be positive wrist bend
int Fetch_IK_solver::ik_solve_elbow_up_given_q1(Eigen::Affine3d const& desired_hand_pose, double q_shoulder_pan, std::vector<Eigen::VectorXd> &q_ik_solns) {
        q_ik_solns.clear();
    std::vector<Eigen::VectorXd> q_ik_solns1234;
        Eigen::Affine3d desired_hand_pose_wrt_DH0 = Affine_shoulder_wrt_torso_inv_*desired_hand_pose;

    bool reachable = compute_q234_solns_elbow_up(desired_hand_pose_wrt_DH0, q_shoulder_pan, q_ik_solns1234);
    if (!reachable) {
        return 0;
    }
    int    nsolns = q_ik_solns1234.size();
    for (int isoln=0;isoln<nsolns;isoln++) {
            test_q1234(q_ik_solns1234[isoln]);
    }
    Eigen::VectorXd q_soln;
    q_soln.resize(NJNTS);
    std::vector<Eigen::VectorXd> q_wrist_solns;
    Eigen::Matrix3d R_des;
    R_des = desired_hand_pose_wrt_DH0.linear();
    reachable=false; //guilty until proven innocent
    for (int i=0;i<nsolns;i++) {
        q_soln = q_ik_solns1234[i];
            if (solve_spherical_wrist(q_soln,R_des, q_wrist_solns)) {  
                int n_wrist_solns = q_wrist_solns.size();
                for (int iwrist=0;iwrist<n_wrist_solns;iwrist++) {
                    q_soln = q_wrist_solns[iwrist];
                    q_ik_solns.push_back(q_soln);
                    reachable = true; // note that we have at least one reachable solution
                }
            }
            else {
                ROS_WARN("singularity in wrist solns!");
            }
    }
    if (!reachable) {
        return 0;
    }
    
    //if here, have reachable solutions in q_ik_solns
    nsolns = q_ik_solns.size();
    return nsolns;    
}



//accessor function to get all solutions

void Fetch_IK_solver::get_solns(std::vector<Eigen::VectorXd> &q_solns) {
    q_solns = q_solns_fit_; //q_solns;
}

//given q1 of J1 (input index), and given desired hand pose--> desired wrist point, 
//  compute solns q2,q3,q4 to place wrist at desired wrist position
// trig ambiguities--> 8 combinations, of which only 2 are real
// tests all 8 solns to find the 2 viable ones
//NOTE: assumes desired_hand_pose is expressed w/rt DH frame 0!!
// may require A_shoulder_wrt_torso_inv_ in calling fnc to convert from torso_lift_link frame
bool Fetch_IK_solver::compute_q234_solns(Eigen::Affine3d const& desired_hand_pose, double q_shoulder_pan, std::vector<Eigen::VectorXd> &q_solns) {
    double r_goal;
    bool reachable;
    //double L_hand = DH_d7;
    
    Eigen::Vector3d p_des = desired_hand_pose.translation();
    Eigen::Matrix3d R_des = desired_hand_pose.linear();
    Eigen::Vector3d z_des = R_des.col(2); // direction of desired z-vector
    Eigen::Vector3d w_des = p_des - L_wrist_*z_des; // desired wrist position w/rt frame0
    //std::cout<<"z_des: "<<z_des.transpose()<<std::endl;
    //std::cout<<"w_des: "<<w_des.transpose()<<std::endl;
    //std::cout<<"p_des: "<<p_des.transpose()<<std::endl;
    
    q_solns.clear();
    Eigen::VectorXd q_soln;
    q_soln.resize(NJNTS);
    
    Eigen::Matrix4d A10;
    //compute_A_of_DH(double a,double d,double q, double alpha); use q_vec(i) + DH_q_offsets(i)
    A10 = compute_A_of_DH(0, q_shoulder_pan); 
    
    Eigen::Matrix3d R10;
    Eigen::Vector3d p1_wrt_0, w_wrt_1_des;
    R10 = A10.block<3, 3>(0, 0);
    //std::cout << "compute_q123, R10: " << std::endl;
    //std::cout << R10 << std::endl;
    p1_wrt_0 = A10.col(3).head(3); ///origin of frame1 w/rt frame0
    //std::cout<<"p1_wrt_0: "<<p1_wrt_0.transpose()<<std::endl;
    
    //compute A10_inv * w_wrt_0, = R'*w_wrt_0 -R'*p1_wrt_0
    w_wrt_1_des = R10.transpose() * w_des - R10.transpose() * p1_wrt_0; //desired wrist pos w/rt frame1
    std::cout << "w_wrt_1_des = " << w_wrt_1_des.transpose() << std::endl;
    r_goal = w_wrt_1_des.norm(); //sqrt(w_wrt_1[0] * w_wrt_1[0] + w_wrt_1[1] * w_wrt_1[1]);

    ROS_INFO("r_goal = %f", r_goal);
    //is the desired wrist position reachable? if not, return false
    // does not yet consider joint limits
    if (r_goal >= L_humerus_ + L_forearm_) {
        ROS_WARN("goal is too far away!");
        return false;
    }
    // can also have problems if wrist goal is too close to shoulder
    if (r_goal <= fabs(L_humerus_ - L_forearm_)) {
        ROS_WARN("goal is too close!");
        return false;
    }
    double q_elbow_solns[2];
    double q_humerus_roll_solns[2];    
    double q_shoulder_lift_solns[2];    
    //BOTH elbow solns should be viable (if within joint limits)
    reachable = solve_for_elbow_theta(r_goal, q_elbow_solns);
    //ROS_INFO("q_elbow solns: %f,  %f",q_elbow_solns[0],q_elbow_solns[1]);
    if (!reachable) {
        ROS_WARN("logic error computing elbow angle!");
        return false;
    } 
    /*else {
        ROS_INFO("elbow solns: %f, %f", q_elbow_solns[0], q_elbow_solns[1]);
    }*/
    
   double q_elbow_soln,q_humerus_roll_soln,q_shoulder_lift_soln;
   Eigen::Vector4d O5_wrt_3,O5_wrt_2,O5_wrt_1;    
   Eigen::Vector3d O5_wrt_1_3d;
   double w_err;
    for (int i_elbow=0;i_elbow<2;i_elbow++) {
       q_elbow_soln = q_elbow_solns[i_elbow]; //consider first elbow soln:
       //ROS_INFO("q_elbow soln: %f",q_elbow_soln);
       O5_wrt_3=compute_O5_wrt_3(q_elbow_soln);
       solve_for_q_humerus_roll(w_wrt_1_des,q_elbow_soln,q_humerus_roll_solns);
       //ROS_INFO("q_humerus_roll solns: %f, %f",q_humerus_roll_solns[0],q_humerus_roll_solns[1]);
       for (int i_humerus_roll=0;i_humerus_roll<2;i_humerus_roll++) {      //test both roll solns:
           q_humerus_roll_soln=q_humerus_roll_solns[i_humerus_roll];
           //ROS_INFO("q_humerus_roll_soln soln: %f",q_humerus_roll_soln);
           O5_wrt_2=compute_O5_wrt_2(O5_wrt_3,q_humerus_roll_soln);           
           solve_for_shoulder_lift(w_wrt_1_des, q_elbow_soln, q_humerus_roll_soln, q_shoulder_lift_solns); 
           //ROS_INFO("q_shoulder_lift_solns: %f, %f",q_shoulder_lift_solns[0],q_shoulder_lift_solns[1]);

           for (int i_shoulder_lift=0;i_shoulder_lift<2;i_shoulder_lift++) {
               q_shoulder_lift_soln=q_shoulder_lift_solns[i_shoulder_lift];
               if ((FETCH_qmin2<q_shoulder_lift_soln)&&(FETCH_qmax2>q_shoulder_lift_soln)) {
               //ROS_INFO("q_shoulder_lift_soln soln: %f",q_shoulder_lift_soln);

               O5_wrt_1=compute_O5_wrt_1(O5_wrt_2,q_shoulder_lift_soln);  
               O5_wrt_1_3d = O5_wrt_1.head(3);
               w_err = (O5_wrt_1_3d-w_wrt_1_des).norm();
               //ROS_INFO("q2, q3, q4 = %f, %f, %f; w_err = %f",q_shoulder_lift_soln,q_humerus_roll_soln,q_elbow_soln,w_err);
               if (w_err<W_ERR_THRESHOLD) {
                 q_soln<<q_shoulder_pan,q_shoulder_lift_soln,q_humerus_roll_soln,q_elbow_soln,0,0,0;
                 ROS_INFO_STREAM("q234_test: "<<q_soln.transpose()<<endl);
                 ROS_WARN("wrist err = %f",w_err);                   
                   //ROS_WARN("saving this soln");
                   q_solns.push_back(q_soln);
                   ROS_INFO("\n \n");
               }
           }
               else ROS_WARN("shoulder soln %f out of range",q_shoulder_lift_soln);
          }
       }
    }
   int nsolns = q_solns.size();
   if (nsolns>0) return true;
   else return false;
}  

//same as above, but only consider positive elbow angle solutions
//should result in a unique soln for q234
bool Fetch_IK_solver::compute_q234_solns_elbow_up(Eigen::Affine3d const& desired_hand_pose, double q_shoulder_pan, std::vector<Eigen::VectorXd> &q_solns) {
    double r_goal;
    bool reachable;
    //double L_hand = DH_d7;
    
    Eigen::Vector3d p_des = desired_hand_pose.translation();
    Eigen::Matrix3d R_des = desired_hand_pose.linear();
    Eigen::Vector3d z_des = R_des.col(2); // direction of desired z-vector
    Eigen::Vector3d w_des = p_des - L_wrist_*z_des; // desired wrist position w/rt frame0
    std::cout<<"z_des: "<<z_des.transpose()<<std::endl;
    std::cout<<"w_des: "<<w_des.transpose()<<std::endl;
    std::cout<<"p_des: "<<p_des.transpose()<<std::endl;
    
    q_solns.clear();
    Eigen::VectorXd q_soln;
    q_soln.resize(NJNTS);
    
    Eigen::Matrix4d A10;
    //compute_A_of_DH(double a,double d,double q, double alpha); use q_vec(i) + DH_q_offsets(i)
    A10 = compute_A_of_DH(0, q_shoulder_pan); 
    
    Eigen::Matrix3d R10;
    Eigen::Vector3d p1_wrt_0, w_wrt_1_des;
    R10 = A10.block<3, 3>(0, 0);
    //std::cout << "compute_q123, R10: " << std::endl;
    //std::cout << R10 << std::endl;
    p1_wrt_0 = A10.col(3).head(3); ///origin of frame1 w/rt frame0
    std::cout<<"p1_wrt_0: "<<p1_wrt_0.transpose()<<std::endl;
    
    //compute A10_inv * w_wrt_0, = R'*w_wrt_0 -R'*p1_wrt_0
    w_wrt_1_des = R10.transpose() * w_des - R10.transpose() * p1_wrt_0; //desired wrist pos w/rt frame1
    std::cout << "w_wrt_1_des = " << w_wrt_1_des.transpose() << std::endl;
    r_goal = w_wrt_1_des.norm(); //sqrt(w_wrt_1[0] * w_wrt_1[0] + w_wrt_1[1] * w_wrt_1[1]);

    ROS_INFO("r_goal = %f", r_goal);
    //is the desired wrist position reachable? if not, return false
    // does not yet consider joint limits
    if (r_goal >= L_humerus_ + L_forearm_) {
        ROS_WARN("goal is too far away!");
        return false;
    }
    // can also have problems if wrist goal is too close to shoulder
    if (r_goal <= fabs(L_humerus_ - L_forearm_)) {
        ROS_WARN("goal is too close!");
        return false;
    }
    double q_elbow_solns[2];
    double q_humerus_roll_solns[2];    
    double q_shoulder_lift_solns[2];    
    //BOTH elbow solns should be viable (if within joint limits)
    reachable = solve_for_elbow_theta(r_goal, q_elbow_solns);
    ROS_INFO("q_elbow solns: %f,  %f",q_elbow_solns[0],q_elbow_solns[1]);
    if (!reachable) {
        ROS_WARN("logic error computing elbow angle!");
        return false;
    } 
    /*else {
        ROS_INFO("elbow solns: %f, %f", q_elbow_solns[0], q_elbow_solns[1]);
    }*/
    
   double q_elbow_soln,q_humerus_roll_soln,q_shoulder_lift_soln;
   Eigen::Vector4d O5_wrt_3,O5_wrt_2,O5_wrt_1;    
   Eigen::Vector3d O5_wrt_1_3d;
   double w_err;
    for (int i_elbow=0;i_elbow<1;i_elbow++) { //only consider first elbow soln
       q_elbow_soln = q_elbow_solns[i_elbow]; //consider first elbow soln:
       if ((q_elbow_soln<FETCH_ELBOW_MAX)&&(q_elbow_soln>FETCH_ELBOW_MIN))
       //ROS_INFO("q_elbow soln: %f",q_elbow_soln);
       O5_wrt_3=compute_O5_wrt_3(q_elbow_soln);
       solve_for_q_humerus_roll(w_wrt_1_des,q_elbow_soln,q_humerus_roll_solns);
       //ROS_INFO("q_humerus_roll solns: %f, %f",q_humerus_roll_solns[0],q_humerus_roll_solns[1]);
       for (int i_humerus_roll=0;i_humerus_roll<2;i_humerus_roll++) {      //test both roll solns:
           q_humerus_roll_soln=q_humerus_roll_solns[i_humerus_roll];
           //ROS_INFO("q_humerus_roll_soln soln: %f",q_humerus_roll_soln);
           O5_wrt_2=compute_O5_wrt_2(O5_wrt_3,q_humerus_roll_soln);           
           solve_for_shoulder_lift(w_wrt_1_des, q_elbow_soln, q_humerus_roll_soln, q_shoulder_lift_solns); 
           //ROS_INFO("q_shoulder_lift_solns: %f, %f",q_shoulder_lift_solns[0],q_shoulder_lift_solns[1]);

           for (int i_shoulder_lift=0;i_shoulder_lift<2;i_shoulder_lift++) {
               q_shoulder_lift_soln=q_shoulder_lift_solns[i_shoulder_lift];
               //ROS_INFO("q_shoulder_lift_soln soln: %f",q_shoulder_lift_soln);
               if ((FETCH_qmin2<q_shoulder_lift_soln)&&(FETCH_qmax2>q_shoulder_lift_soln)) {
               O5_wrt_1=compute_O5_wrt_1(O5_wrt_2,q_shoulder_lift_soln);  
               O5_wrt_1_3d = O5_wrt_1.head(3);
               w_err = (O5_wrt_1_3d-w_wrt_1_des).norm();
               //ROS_INFO("q2, q3, q4 = %f, %f, %f; w_err = %f",q_shoulder_lift_soln,q_humerus_roll_soln,q_elbow_soln,w_err);
               if (w_err<W_ERR_THRESHOLD) {
                 q_soln<<q_shoulder_pan,q_shoulder_lift_soln,q_humerus_roll_soln,q_elbow_soln,0,0,0;
                 //ROS_INFO_STREAM("q234_test: "<<q_soln.transpose()<<endl);
                 //ROS_WARN("wrist err = %f",w_err);                   
                   //ROS_WARN("saving this soln");
                   q_solns.push_back(q_soln);
                   //ROS_INFO("\n \n");
               }
           }
           }
       }
    }
   int nsolns = q_solns.size();
   if (nsolns>0) return true;
   else return false;
}  
    


//reachable = solve_for_theta2(q1a,w_wrt_1,r_goal,q2a_solns);
bool Fetch_IK_solver::solve_for_q_humerus_roll(Eigen::Vector3d w_wrt_1,double q_elbow_soln,double q_humerus_roll_solns[]) {
    //w_5/1,z = s3*s4*d5
    //double q4 = q_elbow_solns[0];
    double s4 = sin(q_elbow_soln);
    if (fabs(q_elbow_soln) < ELBOW_SINGULARITY_THRESHOLD) {
        ROS_WARN("elbow singularity!");
        return false;     
    }
    double w_5wrt1_z = w_wrt_1[2];
    double s4d5 = s4*DH_d5;
    double q3 = asin(w_5wrt1_z/s4d5);
    //ROS_INFO("q_elbow = %f, q_humerus_roll = %f",q_elbow_soln,q3);
    q_humerus_roll_solns[0]=q3;
    q_humerus_roll_solns[1]=M_PI/2-q3;
    return false;
}

bool Fetch_IK_solver::solve_for_shoulder_lift(Eigen::Vector3d w_wrt_1, double q_elbow, double q_shoulder_roll, double q_shoulder_lift_solns[]) {
    //ROS_INFO("solve_for_shoulder_lift: using q_elbow = %f, q_humerus_roll= %f",q_elbow,q_shoulder_roll);
    double w_wrt_1_y = w_wrt_1[1];
    //look at 2nd row of O5_wrt_1 = A_2/1 * A_3/2 * O_5/3
    double s3 = sin(q_shoulder_roll);
    double c3 = cos(q_shoulder_roll);
    double s4d5= sin(q_elbow)*DH_d5;
    double c4d5= cos(q_elbow)*DH_d5;
    //A*sin(q2)+B*cos(q2)= C
    //s4d5*c3+c4d5*s3*c3; s4d5*c3+c4d5*s3*c3
    double A= -s4d5*c3; //s4d5*c3 +s3*c3*c4d5;
    double B = c4d5+DH_d3;
    double C = w_wrt_1_y;
    //double q_shoulder_lift_solns[2];
    solve_Asin_plus_Bcos_eqC(A,B,C,q_shoulder_lift_solns);
    //convert to Fetch coords://q = q_fetch + DH_q_offsets[i];
    q_shoulder_lift_solns[0]-=DH_q_offsets[1];
    q_shoulder_lift_solns[1]-=DH_q_offsets[1];
    
    //test:
    /*
    Eigen::Matrix4d A21 = compute_A_of_DH(1, 1.0);
    Eigen::Matrix4d A32 = compute_A_of_DH(2, 1.0);
    ROS_INFO_STREAM("A32: "<<endl<<A32<<endl);
    //Eigen::Matrix4d A31 = A21*A32;
    Eigen::Vector4d O53;
    O53<<s4d5,-c4d5,0,1;
    ROS_INFO_STREAM("O53: "<<O53.transpose()<<endl);
    Eigen::Vector4d test_O51;
    Eigen::Vector4d test_O52;
    Eigen::Vector4d test2_O52;
    test_O52 = A32*O53;
    test2_O52<< s4d5*c3, s4d5*s3, c4d5+DH_d3, 1;
    test_O51= A21*A32*O53;
    ROS_INFO_STREAM("test_O52: "<<test_O52.transpose()<<endl);
    ROS_INFO_STREAM("test2_O52: "<<test2_O52.transpose()<<endl);

    ROS_INFO_STREAM("test_O51: "<<test_O51.transpose()<<endl);
    ROS_INFO("A,B,C = %f, %f, %f",A,B,C);
    ROS_INFO_STREAM("w_wrt_1: "<<w_wrt_1.transpose()<<endl);   
    //ERROR HERE: A should be O52[0], but this is not A= s4d5*c3+c4d5*s3*c3;
    //examine analytic A32*O_53;
    
    A = test_O52[0];
    ROS_INFO("redo: A= %f",A);
    solve_Asin_plus_Bcos_eqC(test_O52[0],B,C,q_shoulder_lift_solns);
    //convert to Fetch coords://q = q_fetch + DH_q_offsets[i];
    q_shoulder_lift_solns[0]-=DH_q_offsets[1];
    q_shoulder_lift_solns[1]-=DH_q_offsets[1];    
    ROS_INFO("q_shoulder_lift_solns: %f, %f",q_shoulder_lift_solns[0],q_shoulder_lift_solns[1]);
    */
    
    return true; 
}

//rtn elbow solns in ABB coords
bool Fetch_IK_solver::solve_for_elbow_theta(double r_goal, double q_elbow_solns[2]) {
     //phi_elbow;
    double acos_arg = (L_humerus_*L_humerus_ + L_forearm_*L_forearm_ - r_goal*r_goal)/(2.0*L_humerus_*L_forearm_);
    if (fabs(acos_arg>1.0)) {
        ROS_WARN("solve_for_elbow_theta logic err!  acos_arg = %f",acos_arg);
        return false;
    }
    double eta = acos(acos_arg);     
    //cout<<"eta = "<<eta<<endl;
    
    q_elbow_solns[0]=  M_PI -eta; //M_PI -phi_elbow -eta;    //q3(1) = pi - phi- eta; 
    q_elbow_solns[1]=  -q_elbow_solns[0];  //M_PI -phi_elbow +eta;  //q3(2) = pi - phi + eta; */
    /*debug:
    double testx = L_humerus_ + L_forearm_*cos(q_elbow_solns[0]);
    double testy = L_forearm_*sin(q_elbow_solns[0]);
    double testr = sqrt(testx*testx+testy*testy);
    cout<<"rgoal = "<<r_goal<<endl;
    cout<<"q_elbow[0] = "<<q_elbow_solns[0]<<" --> r = "<<testr<<endl;
     testx = L_humerus_ + L_forearm_*cos(q_elbow_solns[1]);
     testy = L_forearm_*sin(q_elbow_solns[1]);
     testr = sqrt(testx*testx+testy*testy);
    cout<<"q_elbow[1] = "<<q_elbow_solns[1]<<" --> r = "<<testr<<endl;
    */
    return true;
}

bool Fetch_IK_solver::fit_q_to_range(double q_min, double q_max, double &q) {
    while (q<q_min) {
        q+= 2.0*M_PI;
    }
    while (q>q_max) {
        q-= 2.0*M_PI;
    }    
    if (q<q_min)
        return false;
    else
        return true;
}

bool Fetch_IK_solver::fit_joints_to_range(Eigen::VectorXd &qvec) {
    bool fits=true;
    bool does_fit;
    double q;
    for (int i=0;i<6;i++) {
        q = qvec[i];
        does_fit = fit_q_to_range(q_lower_limits[i],q_upper_limits[i],q);
        qvec[i] = q;
        fits = fits&&does_fit;
    }
    if (fits)
        return true;
    else
        return false;
}

// find wrist solns; need to check joint limits of q6, wrist bend (symmetric)
// note: if q6 is near zero, then at a wrist singularity; 
// inf solutions of q5+D, q7-D
// use q1, q2, q3, q4 from q_in; copy these values to q_solns, and tack on the two solutions  q5, q6, q7
bool Fetch_IK_solver::solve_spherical_wrist(Eigen::VectorXd q_in,Eigen::Matrix3d R_des, std::vector<Eigen::VectorXd> &q_solns) {
    bool is_singular = false;
    Eigen::Matrix4d A01,A12,A23,A03,A34,A45, A04,A56,A05,A06;
    A01 = compute_A_of_DH(0, q_in[0]);
    A12 = compute_A_of_DH(1, q_in[1]);
    A23 = compute_A_of_DH(2, q_in[2]);
    A34 = compute_A_of_DH(3, q_in[3]);
    A04 = A01*A12*A23*A34;   
    Eigen::Vector3d n5,t5,b5; //axes of frame5
    Eigen::Vector3d n4,t4,b4; // axes of frame4
    Eigen::Vector3d n6,t6; // axes of frame6; b6 is antiparallel to b_des = b7
    Eigen::Vector3d n_des,b_des; // desired x-axis and z-axis of flange or gripper frame
    n4 = A04.col(0).head(3);
    t4 = A04.col(1).head(3);    
    b4 = A04.col(2).head(3);  
    b_des = R_des.col(2);
    n_des = R_des.col(0);
    b5 = b4.cross(b_des);
    ROS_INFO_STREAM("b_des = "<<b_des.transpose()<<endl);
    ROS_INFO_STREAM("b4_wrt0 = "<<b4.transpose()<<endl);
    ROS_INFO_STREAM("b5_wrt0 = "<<b5.transpose()<<endl);
    double q7,q5,q6;
    Eigen::VectorXd q_soln;
      if (b5.norm() <= 0.000001) {
                q5=0;
                is_singular = true;
                ROS_WARN("wrist singularity: b5 norm = %f ",b5.norm());
      }
      else {
            double cq5= b5.dot(-t4);
            double sq5= b5.dot(n4); 
            q5= atan2(sq5, cq5); 
        }
    // choose the positive forearm-rotation solution:
    if (q5>M_PI) {
        q5-= 2*M_PI;
    }    
    if (q5<0.0) {
        q5+= M_PI;
    }
    double q5b = q5 -M_PI;
    // THESE OPTIONS LOOK GOOD FOR q5
    //std::cout<<"forearm rotation options: "<<q5<<", "<<q5b<<std::endl;
    
    // use the + q5 soln to find q6, q7
    A45 = compute_A_of_DH(4, q5);
    A05 = A04*A45;
    n5 = A05.col(0).head(3);
    t5 = A05.col(1).head(3); 
    double cq6 = b_des.dot(t5);
    double sq6 = b_des.dot(n5);
    q6 = -atan2(sq6,cq6); //ad hoc; try to fix sign
    //std::cout<<"wrist bend = "<<q6<<std::endl;

    //solve for q7
    A56 = compute_A_of_DH(5, q6);
    A06 = A05*A56;
    n6 = A06.col(0).head(3);
    t6 = A06.col(1).head(3);   
        
    double cq7=n_des.dot(-n6);
    double sq7=n_des.dot(-t6);
    cout<<"n_des = "<<n_des.transpose()<<endl;
    cout<<"n6 = "<<n6.transpose()<<endl;
    cout<<"t6 = "<<t6.transpose()<<endl;
            
    q7 =atan2(sq7, cq7)+M_PI; //ad hoc attempt to fix soln
    if (q7>M_PI) q7-=M_PI*2.0;
    ROS_INFO("q5,q6,q7 = %f, %f, %f",q5,q6,q7);
    //ROS_INFO("q4,q5,q6 = %f, %f, %f",q4,q5,q6);
    q_soln = q_in;
    q_soln[4] = q5;
    q_soln[5] = q6;
    q_soln[6] = q7;
    q_solns.clear();
    //make 1st soln the positive wrist-bend soln:
    if ((q6>0)&&(q6<FETCH_WRIST_BEND_MAX)) {
       q_solns.push_back(q_soln);
        //2nd wrist soln: 
        q_soln[4] = q5b;
        q_soln[5] *= -1.0; // flip wrist opposite direction
        q_soln[6] = q7+M_PI; // fix the periodicity later; 
       // ROS_INFO("alt q4,q5,q6 = %f, %f, %f",q_soln[3],q_soln[4],q_soln[5]);
        q_solns.push_back(q_soln);        
    }
    
    if ((q6<0)&&(q6>FETCH_WRIST_BEND_MIN)) {
        q_soln[4] = q5b;
        q_soln[5] *= -1.0; // flip wrist opposite direction
        q_soln[6] = q7+M_PI; // fix the periodicity later;         
        q_solns.push_back(q_soln);
        q_soln[4] = q5;
        q_soln[5] = q6;
        q_soln[6] = q7;
        q_solns.push_back(q_soln);
    }    
    if (fabs(q6)>FETCH_WRIST_BEND_MAX) {
        ROS_WARN("wrist bend q6 soln = %f is out of range",q6);
    }
    return !is_singular; //return true if all is well
}


//solve for q given A*sin(q)+B*cos(q) = C
bool Fetch_IK_solver::solve_Asin_plus_Bcos_eqC(double A,double B,double C, double theta[]) {
    //say r = sqrt(A*A+B*B)
    //and A = r*sin(phi); B = r*cos(phi)
    // phi = atan2(A,B)
    // so r*sin(phi)*sin(q) + r*cos(phi)*cos(q)=C
    // C/r = cos(phi-q)
    // phi-q = acos(C/r)
    // q = phi - acos(C/r)
    double R = sqrt(A*A+B*B);
    double phi = atan2(A,B);
    double cos_arg = C/R;
    if (fabs(cos_arg>1.0)) {
        ROS_WARN("solve_Asin_plus_Bcos_eqC error: C/R>1");
        return false;
    }
    double q = acos(C/R);
    theta[0] = phi - q; //note: multipl solns to consider
    theta[1] = phi + q;
    
    //test:
    double testC = A*sin(theta[0])+B*cos(theta[0]);
    //ROS_INFO("theta = %f; test C = %f; specified C = %f",theta[0],testC,C);
    //testC = A*sin(theta[1])+B*cos(theta[1]);
    //ROS_INFO("theta = %f; test C = %f; specified C = %f",theta[1],testC,C);    
    return true;
    
}
/*
Eigen::Affine3d FwdSolver::fwd_kin_solve(Eigen::VectorXd const& q_vec) { // given vector of q angles, compute fwd kin
 Eigen::Affine3d fwd_soln;

 fwd_soln = Fetch_fwd_solver.fwd_kin_solve(q_vec);
 return fwd_soln;
} 

Eigen::MatrixXd FwdSolver::jacobian(const Eigen::VectorXd& q_vec) {
  Eigen::MatrixXd jacobian;
// ... do some work here FINISH ME!
  //jacobian = Fetch_fwd_solver.jacobian(q_vec);
  return jacobian;
}

IKSolver::IKSolver() {

}

int IKSolver::ik_solve(Eigen::Affine3d const& desired_hand_pose,  std::vector<Eigen::VectorXd> &q_ik_solns) {
       // int ik_solve(Eigen::Affine3d const& desired_hand_pose,vector<Eigen::VectorXd> &q_ik_solns);
  int nsolns = Fetch_IK_solver.ik_solve(desired_hand_pose,  q_ik_solns);
  return nsolns;
}
 * */


