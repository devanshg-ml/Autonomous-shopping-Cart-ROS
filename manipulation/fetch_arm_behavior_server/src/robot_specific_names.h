//put robot-specific names here:
string g_urdf_base_frame_name("torso_lift_link");
string g_urdf_flange_frame_name("generic_gripper_frame"); //name from  URDF; need tf publication to generic gripper frame
//for fetch, generic_gripper_frame is what is assumed in fetch_fk_ik as "flange" (or final) frame
string g_joint_states_topic_name("/joint_states");
string g_traj_pub_topic_name("dummy"); //streaming not used for Fetch
string g_traj_as_name("/arm_controller/follow_joint_trajectory"); //action server input for arm
std::vector<std::string> g_jnt_names{"shoulder_pan_joint","shoulder_lift_joint","upperarm_roll_joint","elbow_flex_joint","forearm_roll_joint","wrist_flex_joint","wrist_roll_joint"};
bool g_use_trajectory_action_server = true; //do use action server; not traj streaming

