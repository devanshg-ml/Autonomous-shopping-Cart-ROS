//Setting kit to a pose on top of the robot.

#include <ros/ros.h> //ALWAYS need to include this
//#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Quaternion.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>
#include <sstream>

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

using namespace std;


geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "reset_kit_state");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);
    geometry_msgs::Quaternion quat;

    ros::ServiceClient set_model_state_client = 
       nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    
    gazebo_msgs::SetModelState model_state_srv_msg;

    model_state_srv_msg.request.model_state.twist.linear.x= 0.0; 
    model_state_srv_msg.request.model_state.twist.linear.y= 0.0;
    model_state_srv_msg.request.model_state.twist.linear.z= 0.0;
    
    model_state_srv_msg.request.model_state.twist.angular.x= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.y= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.z= 0.0;
        
    model_state_srv_msg.request.model_state.reference_frame = "world";
    string block("kit");
    
    double x = 0.1;
    double y = 0;
    double z = 0.45;
    double yaw = 1.5707;
    
  
    model_state_srv_msg.request.model_state.model_name = "kit";


    ROS_INFO("set kit state: using x, y, z, yaw = %f, %f, %f, %f",x,y,z,yaw);
    quat = convertPlanarPsi2Quaternion(yaw);

    model_state_srv_msg.request.model_state.pose.position.x = x;
    model_state_srv_msg.request.model_state.pose.position.y = y;
    model_state_srv_msg.request.model_state.pose.position.z = z;
    
    model_state_srv_msg.request.model_state.pose.orientation.x = quat.x;
    model_state_srv_msg.request.model_state.pose.orientation.y = quat.y;
    model_state_srv_msg.request.model_state.pose.orientation.z = quat.z;
    model_state_srv_msg.request.model_state.pose.orientation.w = quat.w;
    
   
    set_model_state_client.call(model_state_srv_msg);
        //make sure service call was successful
        bool result = model_state_srv_msg.response.success;
        if (!result)
            ROS_WARN("service call to set_model_state failed!");
        else
            ROS_INFO("service call was successful");

}
