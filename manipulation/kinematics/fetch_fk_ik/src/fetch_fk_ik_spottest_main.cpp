// wsn, Feb, 2019
// test function for fetch_fk_ik kinematics library

#include <fetch_fk_ik/fetch_kinematics.h>
//#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

Eigen::VectorXd g_q_vec;
using namespace std;
#define VECTOR_DIM 7 // 
//Eigen::VectorXd g_q_vec_arm_Xd;
//vector<int> g_arm_joint_indices; 
//vector<string> g_ur_jnt_names,g_ur_jnt_names_7dof;

double sgn(double x) {
    if (x>0.0) return 1.0;
    if (x<0.0) return -1.0;
    return 0.0; //should virtually never happen for floating-point numbers
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "fetch_kinematics_test_main");
    ros::NodeHandle nh;
    
    Fetch_fwd_solver fwd_solver;
    Fetch_IK_solver ik_solver;

    
    Eigen::VectorXd q_in;
    q_in.resize(NJNTS);
    //q_in<<0,0,0,0,0,0,0; //7 jnts for Kuka arm
    //q_in<<0,0.2,0,0.1, 0.5,1.5,0.3; //7 jnts for Kuka arm   
    //q_in<<0,0.5,0,0.5, 1,1,1;
    //q_in<<0,1,0,-1, 1,1,1;
    //q_goal<<0.0, 1.1, 0.0, -1.4, 0, 0.6, 1, 0.0; //box hover pose:
    //q_in<<0.0, 1.1, 0.0, -1.4, 0, 0.6, 1;
    //q_in<< -1, -1,-1,-1,0,0,0;
    q_in<<1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0;
    Eigen::Affine3d A_fwd_DH = fwd_solver.fwd_kin_solve(q_in); //fwd_kin_solve

        std::cout << "q_in: " << q_in.transpose() << std::endl;        

        std::cout << "A rot: " << std::endl;
        std::cout << A_fwd_DH.linear() << std::endl;
        std::cout << "A origin (to gripper frame): " << A_fwd_DH.translation().transpose() << std::endl;
        Eigen::Matrix3d R_flange = A_fwd_DH.linear();

        Eigen::Quaterniond quat(R_flange);
        std::cout<<"quat: "<<quat.x()<<", "<<quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl; 
        Eigen::Vector3d wrist_pt;
        wrist_pt = fwd_solver.compute_wrist_point(q_in);
        ROS_INFO_STREAM("wrist point: "<<wrist_pt.transpose()<<endl);
        
        std::vector<Eigen::VectorXd> q_solns;
         

        ROS_INFO("calling ik_solver....");
        //debug...
        
        //int nsolns = ik_solver.ik_solve(A_fwd_DH,q_solns);
        //    int ik_solve(Eigen::Affine3d const& desired_hand_pose, double q_shoulder_pan, std::vector<Eigen::VectorXd> &q_ik_solns);
        double q_shoulder_pan = q_in[0];
        int nsolns = ik_solver.ik_solve(A_fwd_DH,q_shoulder_pan,q_solns);
        ROS_INFO_STREAM("desired wrist point: "<<wrist_pt.transpose()<<endl);

        
        
        nsolns = q_solns.size();
        std::cout << "number of IK solutions: " << nsolns << std::endl;    
        
        //test fwd kin:
        std::cout << "q_in: " << q_in.transpose() << std::endl;        
        
        Eigen::Vector3d O_7,O_7_des;
        O_7_des = A_fwd_DH.translation();
        ROS_INFO_STREAM("desired hand position: " <<O_7_des <<endl);
        

        ROS_INFO("test solns (should have 4 solns in all: ");
        for (int i=0;i<nsolns;i++) {
            ROS_INFO_STREAM("q_soln: "<<q_solns[i].transpose()<<endl);
            A_fwd_DH = fwd_solver.fwd_kin_solve(q_solns[i]);
            O_7 = A_fwd_DH.translation();
            double hand_err = (O_7_des-O_7).norm();
            ROS_INFO_STREAM("fwd kin hand position err: " <<hand_err <<endl);
        }


        return 0;  //DEBUG
}
