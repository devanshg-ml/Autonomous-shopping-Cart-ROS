# object_finder_launch

package for launch file for object finder.
This starts up nodes:
rosrun object_finder object_finder_as
rosrun example_rviz_marker triad_display

## Example usage
`roslaunch object_finder_launch object_finder.launch`

## Running tests/demos
Normally, this action server gets goals from the higher-level coordinator.  
But for sub-system test, can run example illustrating use of object_finder_lib (to simplify interface to object finder):
`rosrun object_finder_lib example_main_object_finder_lib`    
