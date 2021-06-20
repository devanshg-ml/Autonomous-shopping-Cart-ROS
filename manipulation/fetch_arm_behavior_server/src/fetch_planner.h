//derived class

class RobotSpecificFK : public FwdSolver {
private:
    Fetch_fwd_solver fwd_kin_solver_;
    Eigen::Affine3d affine_fk_;
    Eigen::MatrixXd jacobian_;
public:
    //int dummy();
    //RobotSpecificFK(): FwdSolver() {};
    //~RobotSpecificFK() {};

    Eigen::Affine3d fwd_kin_solve(Eigen::VectorXd const& q_vec) {
        return (fwd_kin_solver_.fwd_kin_solve(q_vec));
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
};

//these are global--but convenient to put them here:
const int njnts = 6; xxx

//instantiate an object of derived class:
RobotSpecificFK robotSpecificFK;
RobotSpecificIK robotSpecificIK;
//have FwdSolver point to functions defined in derived class
FwdSolver * pFwdSolver = &robotSpecificFK;
IKSolver * pIKSolver = &robotSpecificIK;

