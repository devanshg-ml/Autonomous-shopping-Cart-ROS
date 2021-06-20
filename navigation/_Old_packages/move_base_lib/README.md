# move_base_lib

package for library move_base.  
This class defines functions that help simplify interactions with mobot_pub_des_state.

## Example usage
Start up the simulation:
`roslaunch worlds Fetch_kit.launch`
Put robot in pre-pose:
`roslaunch manipulation_launch pre_pose.launch`
(alternatively, pre-pose is invoked as part of launching the manipulation sub-system, roslaunch manipulation_launch manipulation.launch).

Start up the navigation nodes:
`roslaunch navigation_launch navigation.launch`

## Running tests/demos
can run this subsystem test for navigation dev/debug/test:
`rosrun move_base_lib move_base_example_main` 
