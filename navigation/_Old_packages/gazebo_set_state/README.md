# gazebo_set_state
This node was written to work around Gazebo problems with sliding robot.
When the Fetch robot is navigated to a position, it tends to vibrate, causing it to
slide and rotate.  Thus, once a snapshot is taken and analyzed, it may no longer be 
relevant by the time the arm tries to move to a computed location.

This node subscribes to the Fetch robot's state from Gazebo, and and commands states
to place the robot in Gazebo.  When activated, it gets the current Gazebo state, then
it keeps resetting the robot to that state, thus counteracting the drift.

Service calls to this function should be embedded in control code to turn the "freeze" on and off.

# set_kit_service
this node sets the kit on top of the bottom of Fetch robot. So run this node after pre pose the arm and torso.

## Example usage
`rosrun gazebo_set_state set_kit_service`


## Example usage


Start Gazebo with:
`roslaunch worlds Fetch_kit.launch`

Start the state-freeze node with:
`rosrun gazebo_set_state freeze_robot_service`

Move the robot around (under program control, or manually).

Freeze its position with:
`rosservice call freeze_robot_state true`
or unfreeze it with:
`rosservice call freeze_robot_state false`

Uses message std_srvs/SetBool
