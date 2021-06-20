#include <gripper_interface/gripper_interface.h>

int main(int argc, char** argv) {
	ros::init(argc,argv, "gripper_interface_test");
	ros::NodeHandle nh;
	GripperInterface gripper_interface;
	int ans;

	while(ros::ok()) {
		std::cout<<"Enter 1 for open, 0 for close"<<std::endl;
		std::cin>>ans;
		switch(ans) {
			case 1: 
				gripper_interface.releaseObject();
				break;
			case 0:
				gripper_interface.graspObject();
				break;
			default:
				ROS_INFO("Try again");
		}
	}
	return 0;
}