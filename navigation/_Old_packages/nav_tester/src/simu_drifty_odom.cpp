//fake robot motion
// create fake: base_link child of odom 
// and: odom child of map

//use this node to:
// *integrate pub_des_state speed/spin to compute (flawed) estimate of base_link w/rt odom
// *create a drifty transform to publish fake odom w/rt map (start w/ identical correspondence)

//plot out frames, notably base_link in map frame in rviz, to debug repeatable locations in spite of odom drift
// and imperfect cmd_vel steering

//subscribe to equiv of cmd_vel
//  x_base_link_wrt_odom += v*dt*cos(heading_wrt_odom)
//  y_base_link_wrt_odom += v*dt*sin(heading_wrt_odom)
//  heading_wrt_odom += omega*dt
// publish tf:  base_link w/rt odom


//separately, publish odom w/rt map; 
//init: publish transform parent=map, child = odom, initially identity (later, induce drift)
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
 #include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
#include <tf/transform_broadcaster.h>
#include <xform_utils/xform_utils.h>

geometry_msgs::Twist g_twist;
double g_speed = 0;
double g_omega = 0;



//pubDesState callback:
void desStateCallback(const nav_msgs::Odometry& des_state) {
    g_twist = des_state.twist.twist;
    g_speed = g_twist.linear.x;
    g_omega = g_twist.angular.z;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "odom_simu_node"); //node name
    ros::NodeHandle nh; 

    //XformUtils xform_utils;
    tf::TransformBroadcaster tf_br;

    tf::Transform tf_odom_wrt_map;
    tf::Quaternion q_odom_wrt_map;
    tf::Transform tf_base_link_wrt_odom;
    tf::Quaternion q_base_link_wrt_odom;

    ros::Subscriber des_state_subscriber = nh.subscribe("/desState",1,desStateCallback); 


    double x_base_link_wrt_odom=0;
    double y_base_link_wrt_odom=0;
    double heading_base_link_wrt_odom=0;

    double x_odom_wrt_map=0;
    double y_odom_wrt_map=0;
    double heading_odom_wrt_map=0;
    double odom_rot_drift_rate_factor = 0.02;
    double odom_drift_speed_factor = 0.02;

    double freq = 50.0;

    double dt = 1/freq;
    ros::Rate sleep_timer(50.0);
    //double new_time = ros::Time::now().toSec();

    while (ros::ok()) {
        ros::spinOnce();
        //integrate speed/spin cmds to emulate odometry:
    	 x_base_link_wrt_odom+=g_speed*cos(heading_base_link_wrt_odom)*dt;
   	 y_base_link_wrt_odom+=g_speed*sin(heading_base_link_wrt_odom)*dt;
    	 heading_base_link_wrt_odom+=g_omega*dt; 

         q_base_link_wrt_odom.setRPY(0, 0, heading_base_link_wrt_odom);
         tf_base_link_wrt_odom.setOrigin(tf::Vector3(x_base_link_wrt_odom,y_base_link_wrt_odom,0));
         tf_base_link_wrt_odom.setRotation(q_base_link_wrt_odom);
         tf_br.sendTransform(tf::StampedTransform(tf_base_link_wrt_odom, ros::Time::now(), "odom", "base_link"));


        //now compute and publish odom w/rt map
        heading_odom_wrt_map+=odom_rot_drift_rate_factor*fabs(g_omega)*dt;  //bias of robot tending to rotate CCW
    	 x_odom_wrt_map+=odom_drift_speed_factor*g_speed*cos(heading_odom_wrt_map)*dt;
   	 y_odom_wrt_map+=odom_drift_speed_factor*g_speed*sin(heading_odom_wrt_map)*dt;  
         //send these out as transform of odom w/rt map:
         q_odom_wrt_map.setRPY(0, 0, heading_odom_wrt_map);
         tf_odom_wrt_map.setOrigin(tf::Vector3(x_odom_wrt_map,y_odom_wrt_map,0));
         tf_odom_wrt_map.setRotation(q_odom_wrt_map);
         tf_br.sendTransform(tf::StampedTransform(tf_odom_wrt_map, ros::Time::now(), "map", "odom"));
      

        sleep_timer.sleep(); 
    }
    
    return 0;
} 


