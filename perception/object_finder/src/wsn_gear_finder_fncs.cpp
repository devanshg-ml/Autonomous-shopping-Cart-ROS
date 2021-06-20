//source code for bolt-finder fncs


//Note: may want to use functions defined in object_helper_fncs.cpp, including:
// void ObjectFinder::find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector4f box_pt_min,
//        Eigen::Vector4f box_pt_max, vector<int> &indices)
// float ObjectFinder::find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz)

// void ObjectFinder::find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion) 

//  void ObjectFinder::blob_finder(vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot, ...)

//void ObjectFinder::convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> indices)


//FIX ME!!
const float MIN_X_GEAR = 0.5; //include points starting 0.4m in front of robot 
const float MAX_X_GEAR = 0.8; //include points out to 0.9m in front of robot
const float MIN_Y_GEAR = -1.0; //include points starting -0.5m to left of robot
const float MAX_Y_GEAR = 1.0; //include points up to 0.5m to right of robot
const float MIN_DZ_GEAR = 0.01; //0.01; //box filter from this height above the table top
const float MAX_DZ_GEAR = 0.05; //consider points up to this height above table top
const double TABLE_GRASP_CLEARANCE_GEAR= 0.01;

const double MAX_EVAL_UPPER_BOUND_GEAR = 500000.0; //TUNE ME!
const double MAX_EVAL_LOWER_BOUND_GEAR = 5000.0;
const double EVAL_RATIO_UPPER_BOUND_GEAR = 20.0;
const double EVAL_RATIO_LOWER_BOUND_GEAR = 3.0;
//        float min_blob_avg_ht= MIN_BLOB_AVG_HEIGHT, float min_blob_pixels=MIN_BLOB_PIXELS);

const double GEAR_MIN_BLOB_AVG_HEIGHT = 110;
const double GEAR_MIN_BLOB_PIXELS = 300;

bool ObjectFinder::find_small_gears(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses)  {


//bool ObjectFinder::find_bolts(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
//            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses) {
    
    geometry_msgs::PoseStamped object_pose; //* Used to populate the final message
    object_pose.header.frame_id = "torso_lift_link";
    object_pose.pose.position.z = table_height;

    //! These are the value will get passed back... Initialize them now...
    x_centroids_wrt_robot.clear();
    y_centroids_wrt_robot.clear();
    avg_z_heights.clear();
    npts_blobs.clear();
    object_poses.clear();
    viable_labels_.clear();

    //! Initialize parameter used for box filter
    Eigen::Vector4f box_pt_min, box_pt_max;
    box_pt_min << MIN_X_GEAR, MIN_Y_GEAR, table_height + MIN_DZ_GEAR, 0; //from MIN_DZ above table top
    box_pt_max << MAX_X_GEAR, MAX_Y_GEAR, table_height + MAX_DZ_GEAR, 0;
    
    //find which points in the transformed cloud are within the specified box
    //result is in "indices"
    find_indices_box_filtered(transformed_cloud_ptr_, box_pt_min, box_pt_max, indices_);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices_, *box_filtered_cloud_ptr_); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr_, ros_box_filtered_cloud_); //convert to ros message for publication and display
    int npts_cloud = indices_.size();

   //convert point cloud to top-down 2D projection for OpenCV processing
    convert_transformed_cloud_to_2D(transformed_cloud_ptr_, indices_);

    //! Blob Finder (Segmentation)
    //find connected components; 
    //operates on global bw_img and puts region-labeled codes in global Mat "labelImage" 
    //also, expect centroids w/rt robot torso in g_x_centroids_wrt_robot,g_y_centroids_wrt_robot
    // and area (num pixels) in g_npts_blobs[label]

    blob_finder(x_centroids_wrt_robot, y_centroids_wrt_robot, avg_z_heights, npts_blobs, viable_labels_);
    //
    
    //populate object_poses
    int nblobs = (int) viable_labels_.size();
    if (nblobs<1) return false;
    
    for (int iblob=0;iblob<nblobs;iblob++) {
        if (max_evals_[iblob]>MAX_EVAL_UPPER_BOUND_GEAR || max_evals_[iblob]<MAX_EVAL_LOWER_BOUND_GEAR) {
            ROS_WARN("max eval = %f is out of range for blob %d",max_evals_[iblob],iblob);
        }
        else  {  
               double eval_ratio =max_evals_[iblob]/mid_evals_[iblob];
               ROS_INFO("blob %d max_eval = %f is in range; eval ratio max/mid = %f",iblob,max_evals_[iblob],eval_ratio);
                if (eval_ratio>EVAL_RATIO_UPPER_BOUND_GEAR || eval_ratio<EVAL_RATIO_LOWER_BOUND_GEAR) {
                    ROS_WARN("eval ratio is out of range for blob %d",iblob);
                }
                else {
                    object_pose.pose.position.x = x_centroids_wrt_robot[iblob];
                    object_pose.pose.position.y = y_centroids_wrt_robot[iblob];
                    object_pose.pose.orientation = g_vec_of_quat[iblob];
                    object_poses.push_back(object_pose);                    
                }
        }

    }

    return true; 
}


//identical, really:
bool ObjectFinder::find_large_gears(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses) {

//bool ObjectFinder::find_bolts(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
//            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses) {
    
    geometry_msgs::PoseStamped object_pose; //* Used to populate the final message
    object_pose.header.frame_id = "torso_lift_link";
    object_pose.pose.position.z = table_height;

    //! These are the value will get passed back... Initialize them now...
    x_centroids_wrt_robot.clear();
    y_centroids_wrt_robot.clear();
    avg_z_heights.clear();
    npts_blobs.clear();
    object_poses.clear();
    viable_labels_.clear();

    //! Initialize parameter used for box filter
    Eigen::Vector4f box_pt_min, box_pt_max;
    box_pt_min << MIN_X_GEAR, MIN_Y_GEAR, table_height + MIN_DZ_GEAR, 0; //from MIN_DZ above table top
    box_pt_max << MAX_X_GEAR, MAX_Y_GEAR, table_height + MAX_DZ_GEAR, 0;
    
    //find which points in the transformed cloud are within the specified box
    //result is in "indices"
    find_indices_box_filtered(transformed_cloud_ptr_, box_pt_min, box_pt_max, indices_);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices_, *box_filtered_cloud_ptr_); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr_, ros_box_filtered_cloud_); //convert to ros message for publication and display
    int npts_cloud = indices_.size();

   //convert point cloud to top-down 2D projection for OpenCV processing
    convert_transformed_cloud_to_2D(transformed_cloud_ptr_, indices_);

    //! Blob Finder (Segmentation)
    //find connected components; 
    //operates on global bw_img and puts region-labeled codes in global Mat "labelImage" 
    //also, expect centroids w/rt robot torso in g_x_centroids_wrt_robot,g_y_centroids_wrt_robot
    // and area (num pixels) in g_npts_blobs[label]

    blob_finder(x_centroids_wrt_robot, y_centroids_wrt_robot, avg_z_heights, npts_blobs, viable_labels_,GEAR_MIN_BLOB_AVG_HEIGHT,GEAR_MIN_BLOB_PIXELS);
    //
    
    //populate object_poses
    int nblobs = (int) viable_labels_.size();
    if (nblobs<1) return false;
    
    for (int iblob=0;iblob<nblobs;iblob++) {
        if (max_evals_[iblob]>MAX_EVAL_UPPER_BOUND_GEAR || max_evals_[iblob]<MAX_EVAL_LOWER_BOUND_GEAR) {
            ROS_WARN("max eval = %f is out of range for blob %d",max_evals_[iblob],iblob);
        }
        else  {  
               double eval_ratio =max_evals_[iblob]/mid_evals_[iblob];
               ROS_INFO("blob %d max_eval = %f is in range; eval ratio max/mid = %f",iblob,max_evals_[iblob],eval_ratio);
                if (eval_ratio>EVAL_RATIO_UPPER_BOUND_GEAR || eval_ratio<EVAL_RATIO_LOWER_BOUND_GEAR) {
                    ROS_WARN("eval ratio is out of range for blob %d",iblob);
                }
                else {
                    object_pose.pose.position.x = x_centroids_wrt_robot[iblob];
                    object_pose.pose.position.y = y_centroids_wrt_robot[iblob];
                    object_pose.pose.orientation = g_vec_of_quat[iblob];
                    object_poses.push_back(object_pose);                    
                }
        }

    }

    return true; 
}