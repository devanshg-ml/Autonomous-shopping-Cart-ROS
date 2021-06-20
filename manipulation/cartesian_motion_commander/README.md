# cartesian_motion_commander
This library should be used by action clients of a cartMoveActionServer.
(see package arm_motion_interface).
Intent is that commands should be task oriented and robot agnostic.

This is a library that defines the class ArmMotionInterface, which contains functions
for sending codes and arguments to an arm-motion interface action server (and for
receiving result messages back from the action server).

This class populates goal messages corresponding to different generic (robot-agnostic)
functions, along with required arguments. A goal message includes at least a command (function) code.
This library communicates with an action server named "cartMoveActionServer".

## Example usage
Start up robot simulation in Gazebo with: 
roslaunch worlds Fetch_kit.launch

Initialize the robot pose (if not already done):
rosrun test_fetch_arm_ac fetch_torso_lift_preset
rosrun test_fetch_arm_ac fetch_head_tilt_preset
rosrun test_fetch_arm_ac fetch_arm_pre_pose

for testing, can manually pre-position the robot, e.g.:
try fetch at pose: 0.42, -1.1, 0, 0,0,-1.5707

freeze the robot here (only needed for Gazebo):
`rosrun gazebo_set_state freeze_robot_service`
`rosservice call freeze_robot_state true`

run static transforms publisher:
`roslaunch fetch_arm_behavior_server fetch_static_transforms.launch`

start the cartesian-move behavior server:
rosrun fetch_arm_behavior_server fetch_cart_move_as

TEST ACTION CLIENT:
`rosrun cartesian_motion_commander fetch_cartesian_interactive_ac`




## Running tests/demos
    
