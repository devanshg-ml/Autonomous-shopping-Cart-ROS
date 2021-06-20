// action server to respond to perception requests
// Wyatt Newman, 2/2019
#include<object_finder_as/object_finder.h>
#include "object_finder_helper_fncs.cpp"
//#include "wsn_gearbox_finder_fncs.cpp"
#include "gearbox_finder_fncs.cpp"

#include "wsn_tote_finder_fncs.cpp" 
//#include "gear_finder_fncs.cpp"
#include "gear_finder_fncs.cpp"

#include "bolt_finder_fncs.cpp" 

ObjectFinder::ObjectFinder() :
object_finder_as_(nh_, "object_finder_action_service", boost::bind(&ObjectFinder::executeCB, this, _1), false), pclCam_clr_ptr_(new PointCloud<pcl::PointXYZRGB>),
box_filtered_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>),
transformed_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>), crop_filtered_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>),
pass_filtered_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>) //(new pcl::PointCloud<pcl::PointXYZRGB>);
{
    ROS_INFO("in constructor of ObjectFinder...");
    // do any other desired initializations here...specific to your implementation


    object_finder_as_.start(); //start the server running
    tfListener_ = new tf::TransformListener; //create a transform listener
    found_surface_height_ = false;
    got_headcam_image_ = false;
    initializeSubscribers();
    initializePublishers();
    affine_cam_wrt_torso_ = compute_affine_cam_wrt_torso_lift_link();
    
    //! Frame ID set here. DO NOT CHANGE! Used for RVIZ display
    //PointCloud2 ros_cloud_, downsampled_cloud_, ros_box_filtered_cloud_, ros_crop_filtered_cloud_, ros_pass_filtered_cloud_; //here are ROS-compatible messages
    ros_cloud_.header.frame_id="head_camera_rgb_optical_frame";
    downsampled_cloud_.header.frame_id="head_camera_rgb_optical_frame";      
    ros_box_filtered_cloud_.header.frame_id="head_camera_rgb_optical_frame";
    ros_crop_filtered_cloud_.header.frame_id="head_camera_rgb_optical_frame";
    ros_pass_filtered_cloud_.header.frame_id="head_camera_rgb_optical_frame";

    ROS_INFO("waiting for image data");
    while (!got_headcam_image_) {
        ROS_INFO("waiting...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        got_headcam_image_ = true;
    }
}

void ObjectFinder::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    pointcloud_subscriber_ = nh_.subscribe("/head_camera/depth_registered/points", 1, &ObjectFinder::headcamCB, this);
    // add more subscribers here, as needed
}

void ObjectFinder::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    pubCloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("/object_finder/headcam_pointcloud", 1, true);
    pubDnSamp_ = nh_.advertise<sensor_msgs::PointCloud2> ("/object_finder/downsampled_pcd", 1, true);
    pubBoxFilt_ = nh_.advertise<sensor_msgs::PointCloud2> ("/object_finder/box_filtered_pcd", 1, true);
    pubCropFilt_ = nh_.advertise<sensor_msgs::PointCloud2> ("/object_finder/crop_filtered_pcd", 1, true);
    pubPassFilt_ = nh_.advertise<sensor_msgs::PointCloud2> ("/object_finder/pass_filtered_pcd", 1, true);
    
    //! Addition for publishsing imgae instead of using CV imshow
    pubBWImage_ = nh_.advertise<sensor_msgs::Image>("/object_finder/Black_and_White",1,true);
    pubSegmentedBlob_ =nh_.advertise<sensor_msgs::Image>("/object_finder/Blobbed_Image",1,true);
}

void ObjectFinder::headcamCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!got_headcam_image_) { // once only, to keep the data stable
        ROS_INFO("got new image");
        pcl::fromROSMsg(*cloud, *pclCam_clr_ptr_);
        ROS_INFO("image has  %d * %d points", pclCam_clr_ptr_->width, pclCam_clr_ptr_->height);
        got_headcam_image_ = true;
    }
}

// if asked to find an object:
//  *take a snapshot
//  *find table height
//  *transform points to table frame
//  *box filter these points above table
//  *convert to 2D and find blobs
//  *call corresponding find-object function to recognize specific parts
//  *for blobs of interest, convert coords back to robot torso-lift frame
//  *fill result message with vector of poses of object of interest

void ObjectFinder::executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal) {
    int object_id = goal->object_id;
    geometry_msgs::PoseStamped object_pose;
    vector<geometry_msgs::PoseStamped> object_poses;
    geometry_msgs::PoseStamped fake_object_pose;
    //double table_height;
    bool known_surface_ht = goal->known_surface_ht;
    
    //float surface_height;

    
    bool found_object = false;
    //get a fresh snapshot:
    got_headcam_image_ = false;

    while (!got_headcam_image_) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        ros::Duration(0.1).sleep();
        ROS_INFO("waiting for snapshot...");
    }

    //if here, have a new cloud in *pclCam_clr_ptr_; 
    pcl::toROSMsg(*pclCam_clr_ptr_, ros_cloud_); //convert from PCL cloud to ROS message this way

    //transform this cloud to base-frame coords:
    ROS_INFO("transforming point cloud");
    //transform the head-camera data into the torso frame
    // result will be in "transformed_cloud_ptr_"
    pcl::transformPointCloud(*pclCam_clr_ptr_, *transformed_cloud_ptr_, affine_cam_wrt_torso_);  //Fetch specific here...
    if (known_surface_ht) {
        table_height_ = goal->surface_ht;
    }
    else {
    //find table height from this snapshot:
     table_height_ = find_table_height(transformed_cloud_ptr_, TABLE_TOP_MIN, TABLE_TOP_MAX, 0.01);        
    }



    //box-filter the points:
    //specify opposite corners of a box to box-filter the transformed points
    //magic numbers are at top of this program
    //Eigen::Vector4f box_pt_min, box_pt_max;
    

    //convert point cloud to top-down 2D projection for OpenCV processing
    //convert_transformed_cloud_to_2D(transformed_cloud_ptr_, indices_);

    //find connected components; 
    //operates on global bw_img and puts region-labeled codes in global Mat "labelImage" 
    //also, expect centroids w/rt robot torso in g_x_centroids_wrt_robot,g_y_centroids_wrt_robot
    // and area (num pixels) in g_npts_blobs[label]
    //
    //blob_finder(g_x_centroids_wrt_robot, g_y_centroids_wrt_robot, g_avg_z_heights, g_npts_blobs,viable_labels_);

    result_.object_id = goal->object_id; //by default, set the "found" object_id to the "requested" object_id
    //note--finder might change this ID, if warranted



    switch (object_id) {
            //coordinator::ManipTaskResult::FAILED_PERCEPTION:
        case part_codes::part_codes::GEARBOX_TOP:
            //specialized functions to find objects of interest:
        {
            found_object = find_gearbox_tops(table_height_, g_x_centroids_wrt_robot, g_y_centroids_wrt_robot, g_avg_z_heights, g_npts_blobs, object_poses); //special case for gearbox_top; WRITE ME!
            if (found_object) {
                ROS_INFO("found gearbox_top objects");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_poses.clear();
                int nposes = object_poses.size();
                for (int ipose = 0; ipose < nposes; ipose++) {
                    result_.object_poses.push_back(object_poses[ipose]);
                }
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                object_finder_as_.setAborted(result_);
            }
        }
            break;

        case part_codes::part_codes::GEARBOX_BOTTOM:
            found_object = find_gearbox_bottoms(table_height_, g_x_centroids_wrt_robot, g_y_centroids_wrt_robot, g_avg_z_heights, g_npts_blobs,  object_poses); //special case for gearbox_top; WRITE ME!
            if (found_object) {
                ROS_INFO("found gearbox_bottom objects");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_poses.clear();
                int nposes = object_poses.size();
                for (int ipose = 0; ipose < nposes; ipose++) {
                    result_.object_poses.push_back(object_poses[ipose]);
                }
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                object_finder_as_.setAborted(result_);
            }

            break;

        case part_codes::part_codes::TOTE:
        {

            ROS_INFO("Starting find totes");
            found_object = find_totes(table_height_, g_x_centroids_wrt_robot, g_y_centroids_wrt_robot, g_avg_z_heights, g_npts_blobs,  object_poses);
            if (found_object) {

                ROS_INFO("found tote objects");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_poses.clear();
                int nposes = object_poses.size();
                for (int ipose = 0; ipose < nposes; ipose++) {
                    result_.object_poses.push_back(object_poses[ipose]);
                }
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                object_finder_as_.setAborted(result_);
            }
        }
            break;

        case part_codes::part_codes::BOLT:
            found_object = find_bolts(table_height_, g_x_centroids_wrt_robot, g_y_centroids_wrt_robot, g_avg_z_heights, g_npts_blobs,  object_poses); //special case for gearbox_top; WRITE ME!
            if (found_object) {
                ROS_INFO("found bolt objects");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_poses.clear();
                int nposes = object_poses.size();
                for (int ipose = 0; ipose < nposes; ipose++) {
                    result_.object_poses.push_back(object_poses[ipose]);
                }
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                object_finder_as_.setAborted(result_);
            }

            break;
        case part_codes::part_codes::SMALL_GEAR:
            found_object = find_small_gears(table_height_, g_x_centroids_wrt_robot, g_y_centroids_wrt_robot, g_avg_z_heights, g_npts_blobs,  object_poses); //special case for gearbox_top; WRITE ME!
            if (found_object) {
                ROS_INFO("found small gear objects");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_poses.clear();
                int nposes = object_poses.size();
                for (int ipose = 0; ipose < nposes; ipose++) {
                    result_.object_poses.push_back(object_poses[ipose]);
                }
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                object_finder_as_.setAborted(result_);
            }

            break;
        case part_codes::part_codes::LARGE_GEAR:
            found_object = find_large_gears(table_height_, g_x_centroids_wrt_robot, g_y_centroids_wrt_robot, g_avg_z_heights, g_npts_blobs, object_poses); //special case for gearbox_top; WRITE ME!
            if (found_object) {
                ROS_INFO("found large gear objects");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_poses.clear();
                int nposes = object_poses.size();
                for (int ipose = 0; ipose < nposes; ipose++) {
                    result_.object_poses.push_back(object_poses[ipose]);
                }
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                object_finder_as_.setAborted(result_);
            }

            break;

        case part_codes::part_codes::FAKE_PART: //return a hard-coded pose	
            ROS_INFO("returning fake part pose");
            result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
            result_.object_poses.clear();
            fake_object_pose.header.frame_id = "torso_lift_link";
            //clicked point on handle:  x: 0.643226742744, y: -0.120291396976, z: 0.225667536259	
            //clicked point on table:    x: 0.515702664852,   y: -0.101541608572,   z: 0.0994542837143	
            fake_object_pose.pose.position.x = 0.725; //had trouble w/  0.6...maybe arm hits head?	
            fake_object_pose.pose.position.y = 0.1;
            fake_object_pose.pose.position.z = 0.2; //0.064 is table height w/rt torso_lift_link, fingertips touch tote table	

            fake_object_pose.pose.orientation.x = 0;
            fake_object_pose.pose.orientation.y = 0;
            fake_object_pose.pose.orientation.z = 0.707;
            fake_object_pose.pose.orientation.w = 0.707;

            result_.object_poses.push_back(fake_object_pose);

            object_finder_as_.setSucceeded(result_);

            break;

        default:
            ROS_WARN("this object ID is not recognized");
            result_.found_object_code = object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED;
            object_finder_as_.setAborted(result_);
    }

}

