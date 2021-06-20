//program to freeze robot state in Gazebo, 
//`rosservice call freeze_robot_state true`  
#include <ros/ros.h> //ALWAYS need to include this

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>

#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Quaternion.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <math.h>
#include <sstream>

using namespace std;


bool g_freeze_robot = false;
bool g_get_robot_pose = false;
bool g_trigger_new_request = false;

gazebo_msgs::ModelState g_fetch_model_state; //this is the pose of the robot in the world, according to Gazebo
//callbacks:
// 1) get model state
// 2) process incoming requests to freeze robot

//this callback gets fetch pose from Gazebo and latches it;
//set g_get_robot_pose to true to update the latched pose

void model_state_CB(const gazebo_msgs::ModelStates& model_states) {
    int n_models = model_states.name.size();
    int imodel;
    //ROS_INFO("there are %d models in the transmission",n_models);
    bool found_name = false;
    if (g_get_robot_pose) {
        for (imodel = 0; imodel < n_models; imodel++) {
            std::string model_name(model_states.name[imodel]);
            if (model_name.compare("fetch") == 0) {
                //ROS_INFO("found match: mobot is model %d",imodel);
                found_name = true;
                break;
            }
        }
        if (found_name) {
            g_fetch_model_state.pose = model_states.pose[imodel];
            g_fetch_model_state.model_name = model_states.name[imodel];
            g_get_robot_pose = false;
            ROS_INFO_STREAM("latched robot state: " << g_fetch_model_state << endl);
        } else {
            ROS_WARN("state of fetch model not found");
        }
    }
}

//callback fnc to process incoming requests to freeze or unfreeze the robot pose

bool callback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {

    g_freeze_robot = request.data;
    g_trigger_new_request = true;

    ROS_INFO("callback activated; g_freeze_robot= %d", g_freeze_robot);
    response.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "freeze_robot_node");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);
    geometry_msgs::Quaternion quat;
    ros::ServiceServer service = nh.advertiseService("freeze_robot_state", callback);
    // make sure service is available before attempting to proceed, else node will crash
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/set_model_state", true);
        ROS_INFO("waiting for set_model_state service");
        half_sec.sleep();
    }
    ROS_INFO("set_model_state service exists");

    ros::ServiceClient set_model_state_client =
            nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::SetModelState model_state_srv_msg;
    string robot_name("fetch");
    
    ros::Subscriber state_sub = nh.subscribe("gazebo/model_states",1,model_state_CB); 
    ROS_INFO("waiting for robot state from Gazebo: ");
    g_get_robot_pose = true;
    while (g_get_robot_pose) {
                ROS_INFO("waiting for robot pose from Gazebo...");
                ros::spinOnce();
                ros::Duration(0.5).sleep();
    }    


    while (ros::ok()) {
        if (g_trigger_new_request) {
            ROS_INFO("received new freeze/unfreeze request");
            g_trigger_new_request = false; //reset trigger
            //get the current robot model state:
            g_get_robot_pose = true;
            while (g_get_robot_pose) {
                ROS_INFO("waiting for robot pose from Gazebo...");
                ros::spinOnce();
                ros::Duration(0.5).sleep();
            }
            model_state_srv_msg.request.model_state = g_fetch_model_state;
        }
        if (g_freeze_robot) {

                set_model_state_client.call(model_state_srv_msg);
                //make sure service call was successful
                bool result = model_state_srv_msg.response.success;
                if (!result)
                    ROS_WARN("service call to set_model_state failed!");
                else {
                    //ROS_INFO("freezing robot");
                }
        }
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        

    }
}