# fetch_fk_ik

forward and inverse kinematics library for Fetch robot arm.
Assumes base frame for arm kin is torso_lift_link, and end frame is generic_gripper_frame

FK is unambiguous.  
For IK, handles redundancy by solving for IK given shoulder-pan joint angle, e.g.:
specify desired transform from torso_lift_link to generic_gripper_frame as A_fwd_DH,
specify q_shoulder.
get back n solutions in q_solns.

int nsolns = ik_solver.ik_solve(A_fwd_DH,q_shoulder_pan,q_solns);

alt fnc: forces elbow-up soln

int Fetch_IK_solver::ik_solve_elbow_up_given_q1(Eigen::Affine3d const& desired_hand_pose, double q_shoulder_pan, std::vector<Eigen::VectorXd> &q_ik_solns)

## Example usage
See fetch_fk_ik_spottest_main.cpp for use of this library.
Can run spot checks with:
`roscore`
`rosrun fetch_fk_ik fetch_fk_ik_spottest_main`


    
