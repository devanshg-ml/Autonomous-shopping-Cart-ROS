
//derived class

class RobotSpecificFK : public FwdSolver {
private:
    Fetch_IK_solver fetch_fwd_solver_;
    Eigen::Affine3d affine_fk_;
    Eigen::MatrixXd jacobian_;
public:
    //int dummy();
    //RobotSpecificFK(): FwdSolver() {};
    //~RobotSpecificFK() {};

    Eigen::Affine3d fwd_kin_solve(Eigen::VectorXd const& q_vec) {
        return (fetch_fwd_solver_.fwd_kin_solve(q_vec));
    };

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q_vec) {
        return (fetch_fwd_solver_.jacobian(q_vec));
    }
};

class RobotSpecificIK : public IKSolver {
private:
    Fetch_IK_solver fetch_ik_solver_;
    Eigen::Affine3d affine_fk_;
public:

    RobotSpecificIK() {
    }; //constructor
    //~RobotSpecificIK() {};

    int ik_solve(Eigen::Affine3d const& desired_hand_pose, std::vector<Eigen::VectorXd> &q_ik_solns) {
        return (fetch_ik_solver_.ik_solve_simple_reach(desired_hand_pose, q_ik_solns));
    }
    //irb140 does not need Jacobian refinement of IK, so provide a do-nothing dummy fnc
    //this is only needed if the robot requires refinement of IK solutions using Jacobian
    void ik_refine(std::vector<Eigen::Affine3d> cartesian_affine_samples, std::vector<Eigen::VectorXd> &optimal_path) {
        return;
    };

};

//these are global--but convenient to put them here:
//const int NJNTS = 7; defined in kinematics.h

//instantiate an object of derived class:
RobotSpecificFK robotSpecificFK;
RobotSpecificIK robotSpecificIK;
//have FwdSolver point to functions defined in derived class
FwdSolver * pFwdSolver = &robotSpecificFK;
IKSolver * pIKSolver = &robotSpecificIK;

