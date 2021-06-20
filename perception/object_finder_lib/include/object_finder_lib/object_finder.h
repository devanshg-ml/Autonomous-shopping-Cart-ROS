#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_finder/objectFinderAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <part_codes/part_codes.h>

std::vector <geometry_msgs::PoseStamped> g_perceived_object_poses;
int g_found_object_code;

class FindPart {
public:
	FindPart();
//std::vector <geometry_msgs::PoseStamped> g_perceived_object_poses;
	bool find_part(int part_code, std::vector <geometry_msgs::PoseStamped> &part_poses);
        void display_triad(geometry_msgs::PoseStamped triad_pose);

private:
    ros::NodeHandle nh_; 
        actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac_;
        void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
            const object_finder::objectFinderResultConstPtr& result);

        object_finder::objectFinderGoal object_finder_goal_;
        bool found_object_code_; 
        std::vector <geometry_msgs::PoseStamped> perceived_object_poses_;
        ros::Publisher pose_publisher_; // = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
};



