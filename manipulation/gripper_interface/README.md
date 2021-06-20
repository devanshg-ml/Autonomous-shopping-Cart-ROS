# GripperInterface class

# How to use: 
GripperInterface gripper_interface_object;

To close gripper: gripper_interface_object.graspObject();

To open gripper: gripper_interface_object.releaseObject();

More functions available with added functionality of specifying part type to determine grasp width
Note: Functions implementing a timeout need to be tested
