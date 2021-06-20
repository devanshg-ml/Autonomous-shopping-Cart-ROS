#include <gripper_interface/gripper_interface.h>

/*important: this library considers gripper status "stalled" to be safe. 
Gripper returns "stalled" when the goal position cannot be reached without exceeding the "max_effort" force limit.  
It commands grasp width to be smaller than object width and waits till the gripper goes to "stalled" to mark grasp as successful. 
It also considers stalled status to be an indicator of whether the gripper is currently holding an object.
Another possibility is using the boolean "reached_goal" which is sent as a result. 
Why I have not used this: Reached goal returns true when the gripper position is within a tolerance of the goal position
This is not neccessarily an indication of successful grasps due to tolerance in measured dimensions. 
While this works in simulation, the actual hardware gripper might consider "stalled" status to be an error state. This could cause problems, need to check
*/ 

//4/26/19: disabled sticky-finger client for tests on actual robot

GripperInterface::GripperInterface() : ac_("gripper_controller/gripper_action", true) {
	ROS_INFO("Connecting to gripper action server");
	ac_.waitForServer();
	//open the gripper and be ready
	//serves a second purpose of getting gripper status through "result" continously to check gripper status
	//"result will only be sent after a first goal request "
	goal_.command.position = OPEN_POSITION_;
	goal_.command.max_effort = OPEN_EFFORT_;
	ac_.sendGoal(goal_);
	ROS_INFO("Connected!");
        //sticky_finger_client = nh_.serviceClient<std_srvs::SetBool>("/sticky_finger/r_gripper_finger_link");
        srv_stick.request.data = true;
        srv_release.request.data = false;
}



bool GripperInterface::graspObject() { //dummy func for testing
	//std::string object = "dummy_part";
        //sticky_finger_client.call(srv_stick);
        ROS_INFO("sticky-finger grasp response: %d",srv_stick.response.success);
	std::string object("dummy_part");
	double timeout = 0;
	return graspObject(object); 
}

/*
open loop grasp commander. doesnt check if grasp is successful or not
*/

bool GripperInterface::graspObject(std::string object) {
        //sticky_finger_client.call(srv_stick);
        //ROS_INFO("sticky-finger response: %d",srv_stick.response.success);
	result_ = *ac_.getResult();
	//if (isGrasping()) return false; //! TO FIX why it doesn't work
	ROS_ERROR("SENDING COMMEND NOW");
	goal_.command.position = 0;//part_width_map_[object];
	goal_.command.max_effort = MAX_EFFORT_;
	ac_.sendGoal(goal_);
	ROS_ERROR("SEND GOAL COMPLETE");
}

/*
computes grasp width from object name string. returns:
true: Grasp completed within timeout
false: (a) Already grasping another object (b) Grasp not successful by timeout (c) No object present to grasp - gripper reaches grasp width (<object width) without stalling 
*/

bool GripperInterface::graspObject(std::string object, double timeout) {
	result_ = *ac_.getResult();
	//if (isGrasping()) return false; //! TO FIX why it doesn't work
	ROS_ERROR("OPTION2");
	goal_.command.position = 0;//part_width_map_[object];
	goal_.command.max_effort = MAX_EFFORT_;
	ac_.sendGoal(goal_);
	return waitForGrasp(timeout);
}

/* waits for grasp. returns:
true: Object is found to be grasped within timeout
false: (b) Grasp not successful by timeout (c) No object present to grasp - gripper reaches grasp width (<object width) without stalling 
This function is still bounded by the time it takes to "getResult", which makes the timeout check inaccurate. 
Solution: Implement waitForResult() with a time limit << timeout
Possible problem: small time limit in waitForResult() might always fail  
*/

bool GripperInterface::waitForGrasp(double timeout) {
	double ts = ros::Time::now().toSec();
	while(!isGrasping()) {
		if((ros::Time::now().toSec() - ts)> timeout) {
			return false;
		}
	}
	return true;
}

/* requests new result and checks gripper status from it
true: gripper is stalled (refer above for potential holes in this logic)
false: (c) No object present to grasp - gripper reaches grasp width (<object width) without stalling 
*/

bool GripperInterface::isGrasping() {
	result_ = *ac_.getResult();
	return result_.stalled; 
}

/* open loop release function. All it does is sends a goal asking the gripper to open to a position = OPEN_POSITION_
*/

bool GripperInterface::releaseObject() { 
        //sticky_finger_client.call(srv_release);
        //ROS_INFO("sticky-finger release response: %d",srv_release.response.success);
	goal_.command.position = OPEN_POSITION_;
	goal_.command.max_effort = OPEN_EFFORT_;
	ac_.sendGoal(goal_);
}


bool GripperInterface::releaseObject(std::string, double timeout) {
	//Release doesn't really require object name, but keeping it for convenience 
	return releaseObject(timeout);
}

bool GripperInterface::releaseObject(double timeout) {
	/*Advantages of making release a blocking function: Prevent race condition between releasing object and other actions which could affect final position of released object.
	Like moving the arm away before the object is completely out of reach, this could be prevented by strictly ensuring that the first movement after releasing an object is perpendicular to the surface.
	For now, release is a blocking function that waits for the gripper to send a result of "reached goal = 1" on asking it to open
	How does a timeout affect release? Do we really want to wait until gripper fingers are back to open position? 
	*/
	//not implementing a check for whether the gripper is actually holding an object or not
	goal_.command.position = OPEN_POSITION_;
	goal_.command.max_effort = OPEN_EFFORT_;
	ac_.sendGoal(goal_);
	return waitForRelease(timeout);	
}

/* This function is probably overkill for a simple release. Not been tested.
Waits for the action server to return true to "reached_goal" for a goal of opening gripper fingers to a position of "OPEN_POSITION_"
*/

bool GripperInterface::waitForRelease(double timeout) {
	double ts = ros::Time::now().toSec();

	result_ = *ac_.getResult();
	while(!result_.reached_goal) {
		if((ros::Time::now().toSec() - ts)> timeout) {
			return false;
		}
		result_ = *ac_.getResult();
	}
	return result_.reached_goal;
}