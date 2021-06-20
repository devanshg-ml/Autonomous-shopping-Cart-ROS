# object_finder
This package is a "shell" for development of an object-finder action service.
In the goal message, specify a code corresponding to a known object type.
These codes are defined in:
#include <part_codes/part_codes.h>

e.g.: part_codes::part_codes::GEARBOX_TOP

The response will contain a return code, e.g. for "object found", "object type not recognized",
or "object not found".  

If object is found, there may be one or more.  Poses of objects will be in result message
in a vector of object poses.


## Example usage
can emulate camera by starting a roscore, then fake publishing to camera topic with:
roscd pcd_images;  rosrun pcl_utils display_pcd_file
  (enter fname)

`rosrun object_finder object_finder_as`
`rosrun object_finder example_object_finder_action_client`

##update 5/1/19:
start a  bag file (rosbag play ...) and start up rviz.
Display items:  various pointcloud2 topics, e.g. /object_finder/box_filtered_pcd,  and Marker w/ topic triad_display  

`rosrun example_rviz_marker triad_display`  
`rosrun object_finder object_finder_as`  
`rosrun object_finder example_object_finder_action_client`  


