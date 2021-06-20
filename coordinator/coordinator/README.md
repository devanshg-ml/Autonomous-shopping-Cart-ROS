# The Coordinator Code Package

This is the coordinator code package. It includes all the acutal code for coordinator.

## Most Up to Date Node

The following node will be actively used, as of `2019-04-30`.

**grabe_tote_v2:** `rosrun coordinator grab_tote_v2`  
This node is responsible for: initiating a real grabe tote call and test with a fake grab tote call

**visit_all:** `rosrun coordinator visit_all`  
This node is responsible for: command the robot around the arena once, with user interaction for starting a navigation

## Included Nodes

Nodes here are included, but they might not be active.

**arm_test:**`rosrun coordinator arm_test`  
This node is responsible for performing a simple arm grip motion test.

**grab_tote:** `rosrun coordinator grab_tote`  
This node is **OBSOLETE**. Its function is now replaced with **grabe_tote_v2**.

## Dependencies

Uses libraries: find_part_lib, move_base_lib, move_part_lib for perception, navigation and manipulation, respectively.