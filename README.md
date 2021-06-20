# Autonomous-shopping-Cart-ROS
This repository hosts TeamCase's Fetchit Challenge.

For Fetchit Challenge, refer: https://opensource.fetchrobotics.com/competition

This code is organized in three major areas: navigation, perception and manipulation. See READMEs in these directories regarding their use and how to run subsystem tests.

The coordinator package is responsible for integrating the above. This is aided by libraries in each subsystem that simplify the interfaces to them, including move_base_lib, move_part_lib and object_finder_lib.

A typical sequence would consist of: invoking navigation (e.g. to a pre-coded zone), invoking object finding (using object codes) and object manipulation (with part codes and location codes or specified poses).

Folders pre-fixed with "test" are designated for different testings.

Running the Code:
To start the challenge in Gazebo: roslaunch fetchit_challenge main.launch

Start the perception action server: roslaunch object_finder_launch object_finder.launch

Start the navigation nodes: roslaunch navigation_launch navigation.launch

Start the manipulation nodes: roslaunch manipulation_launch manipulation.launch

TEST: can do interactive Cartesian moves with: rosrun cartesian_motion_commander fetch_cartesian_interactive_ac

If desired, place kit on pedestal with: rosrun gazebo_set_state set_kit_service then "glue" the kit to the pedestal with: rosservice call sticky_finger/base_link true

Start the coordinator: roslaunch coordinator_launch coordinator.launch

Environment Configuration:
CWRU students:
Make sure you git pull the following repositories alongside Team Case's repository in your ROS_WS or Fetch_WS

Fetch ROS Repository: https://github.com/fetchrobotics-gbp/fetch_ros-release.git

Fetch Gazebo Repository: https://github.com/fetchrobotics/fetch_gazebo.git

Fetch Robot Controller Repository (0.6.0-0): https://github.com/fetchrobotics/robot_controllers-release.git

RGBD Launch Repository: https://github.com/ros-gbp/rgbd_launch-release.git

ROS Controllers Repository: https://github.com/ros-gbp/ros_controllers-release.git

Using Docker Provided by Fetch:
Please refer to: https://github.com/fetchrobotics/fetch_gazebo/issues/75

NOTE: Fetch uses catkin overlay, and have a repository named stable and active.

To make your own computer look like Fetch's enviornment: (adapted from Fetch Docker Enviornment)
Brand new clean environment.
Install ROS-melodic-desktop-full
Install python-catkin-tools python-rosinstall-generator python-rosinstall
Execute . /opt/ros/melodic/setup.sh
Execute rosinstall_generator fetchit_challenge --deps --deps-only --exclude RPP > stable.rosinstall
Execute rosinstall_generator fetchit_challenge --upstream > active.rosinstall
Execute mkdir -p $HOME/ros/stable $HOME/ros/active This will help create the default active and stable development folder. Always put stable code (meaning those code who are not changing often or special package not included in the common environement) in stable folder.
Execute wstool init $HOME/ros/stable/src stable.rosinstall \ && wstool init $HOME/ros/active/src active.rosinstall This will help do a default pull from fetch's repository and other needed repositories.
Goto your stable repository (ros/stable) and Execute: catkin config --init \ && catkin config --install --extend /opt/ros/melodic \ && catkin build This will be the first time compile.
Goto your active repository (ros/active) and Execute: catkin config --init \ && catkin config --extend $HOME/ros/stable/install \ && catkin build This will be the first time compile for your stable environement.
Your development environment should be all setup.
Note 1: To include your repository, add it to the active.rosinstall file.

Note 2: To update everything in the repository, instead of do git pull, you can just go to your src folder and execute rosinstall <your repo src folder path> <your rosinstall file path>

Note 3: You do need to add source ~/ros/devel/setup.bash to your ~/.bashrc file so that it auto sources when startup.

Note 4: Instead of catkin make, you do catkin build at where you usually do catkin make.
