# move_part_lib
Library to simplify interactions with arm behavior server.
key functions:
movePart.get_part(partCode,source_pose);
movePart.place_grasped_part(int part_code, geometry_msgs::PoseStamped destination_pose);
bool stow_grasped_part(int part_code);


## Example usage
to be used in higher-level "coordinator", but for sub-system test, do blind manipulation test by:
start up robot positioned in front of gearbox table;  

`rosrun move_part_lib example_move_part_main`  

    
