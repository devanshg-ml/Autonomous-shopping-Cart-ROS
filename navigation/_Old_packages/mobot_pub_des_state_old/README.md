# mobot_pub_des_state
This package illustrates a desired-state publisher that creates and publishes
sequences of states that are dynamically feasible and which lead a robot through
a sequence of subgoals, treated as a polyline.  The publisher exploits the
traj_builder library to construct dynamically-feasible trajectories.  It accepts
subgoals via the service "append_path_queue_service", which appends subgoals to
the current queue of subgoals.  The service "estop_service" invokes an e-stop
state, causing the robot to come to a halt with a dynamically-feasible trajectory.
The service "clear_estop_service" allows the robot to resume visiting subgoals.
The current path queue can be flushed via the service "flush_path_queue_service".
When there are no subgoals left in the queue, the robot halts at its lasts subgoal.
It will resume motion if/when new subgoals are added to the queue.

## Example usage
`roslaunch gazebo_ros empty_world.launch`
`roslaunch mobot_urdf mobot.launch`
`rosrun mobot_pub_des_state open_loop_controller`
`rosrun mobot_pub_des_state mobot_pub_des_state`
`rosrun mobot_pub_des_state pub_des_state_path_client`

Perform e-stop and e-stop reset with:
`rosservice call estop_service`
`rosservice call flush_path_queue_service`
`rosservice call clear_estop_service`

NEW CODE FOR FETCH ROBOT:
pubDesState is modified with a new service to reset the current pose.
Subsequent motion commands are trajectories with respect to the new start pose.
This likely does NOT work with closed-loop steering.  However, it does allow for corrections based on
AMCL base_link with respect to map, so one can clear accumulated errors and drive the robot using cmd_vel (via open_loop_controller in this package)  
To use the new code, publish a map, start up AMCL, and run:
`rosrun mobot_pub_des_state open_loop_controller`
`rosrun mobot_pub_des_state mobot_pub_des_state`
`rosrun mobot_pub_des_state pub_des_state_path_client_amcl_correction`

Then can command motions to key poses a client, or from command line with:
`rosservice call set_key_pose_index 2`  (pick a number of code to send for key-pose index)  

Robot will compute and execute a trajectory to move from the current pose to the desired x,y position.  Would need a separate command
to coerce the alignment as well.

 

    
