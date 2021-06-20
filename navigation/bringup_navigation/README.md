# navigation_launch

package for launch file for navigation.
Navigation will start up: 
rosrun mobot_gazebo_state mobot_gazebo_state
rosrun lin_steering lin_steering_wrt_gazebo_state cmd_vel:=/base_controller/command
rosrun mobot_pub_des_state mobot_pub_des_state
rosrun gazebo_set_state freeze_robot_service

WATCH OUT:  this launch file assumes access to "magic" state from Gazebo.
This needs to be replaced with a localization means.
ALSO: this node uses a "freeze" service, which is only relevant for simulation.

## Example usage
`roslaunch navigation_launch navigation.launch`

## Running tests/demos
Normally, navigation will get its commands from the higher-level coordinator.
But here is an example using move_base_lib to interact with navigation subsystem:
`rosrun move_base_lib move_base_example_main` 
    
