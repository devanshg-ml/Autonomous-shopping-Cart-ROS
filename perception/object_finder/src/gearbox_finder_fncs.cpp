//! This is the source code for finding the gearbox bottom and gearbox top

/*
*Note: may want to use functions defined in object_helper_fncs.cpp, including:
void ObjectFinder::find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector4f box_pt_min, Eigen::Vector4f box_pt_max, vector<int> &indices)
float ObjectFinder::find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz)
void ObjectFinder::find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion) 
void ObjectFinder::blob_finder(vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot, ...)
void ObjectFinder::convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> indices)
*/

//! Magic Numbers:
const float MIN_X_GEARBOX = -0.0; //include points starting 0.4m in front of robot
const float MAX_X_GEARBOX = 1.5; //include points out to 0.9m in front of robot
const float MIN_Y_GEARBOX = -1.5; //include points starting -0.5m to left of robot
const float MAX_Y_GEARBOX = 1.5; //include points up to 0.5m to right of robot
const float MIN_DZ_GEARBOX = 0.01; //box filter from this height above the table top
const float MAX_DZ_GEARBOX = 0.1; //consider points up to this height above table top
const double TABLE_GRASP_CLEARANCE_GEARBOX = 0.01; //determines how high away from the table the arm need to be

//! Magic Number for seperating gearbox:
//* Penalty Value
const float HEIGHT_PENALTY = 1.15;
const float POINTS_PENALTY = 1.2;
const float POSE_PENALTY = 1.05;
//const float DITCH_THREASHOLD = 0
//* Gearbox bottom facing up
const float GEARBOX_BOTTOM_UP_PTS = 860;
const float GEARBOX_BOTTOM_UP_Z = 130;
//* Gearbox bottom facing down
const float GEARBOX_BOTTOM_DOWN_Z = 150;
const float GEARBOX_BOTTOM_DOWN_PTS = 1170;
//* Gearbox bottom sideway
const float GEARBOX_BOTTOM_SIDE_Z = 135;
const float GEARBOX_BOTTOM_SIDE_PTS = 925;

//* Gearbox top facing up
const float GEARBOX_TOP_UP_PTS = 860;
const float GEARBOX_TOP_UP_Z = 130;
//* Gearbox top facing down
const float GEARBOX_TOP_DOWN_Z = 150;
const float GEARBOX_TOP_DOWN_PTS = 1170;
//* Gearbox bottom sideway
const float GEARBOX_TOP_SIDE_Z = 125;
const float GEARBOX_TOP_SIDE_PTS = 340;

//! Local Tool Kit:

void splitGearboxTopBottom(vector <int> &lookup_table, int part_id, vector<float> &avg_z_heights, vector<float> &npts_blobs){
    lookup_table.clear();
    if (part_id == part_codes::part_codes::GEARBOX_BOTTOM){
        ROS_WARN("[gearbox_finder_fnc=seperation] Seperating for BOTTOM gearbox part now...");
       //! Vectors for temporarily storing scores and corresponding poses 
        vector <float> scores; 
        vector <int> poses;  //Up = 1; Down = 0; Side = 2

        //! Loop through all posible blobs
        int total_blobs = avg_z_heights.size();
        for(int counters = 0; counters < total_blobs; counters ++){
            float temp_z = avg_z_heights[counters];
            float temp_pts = npts_blobs[counters];
            float scoring_up, scoring_down, scoring_side, final_score;

            //! Compare the poses with three possible poses, produce a final score and tell us what pose it is
            // We are using a nominal result, to make it dimensionless.
            scoring_up = (HEIGHT_PENALTY * abs((temp_z-GEARBOX_BOTTOM_UP_Z)/GEARBOX_BOTTOM_UP_Z))+(POINTS_PENALTY * abs((temp_pts-GEARBOX_BOTTOM_UP_PTS)/GEARBOX_BOTTOM_UP_PTS));
            ROS_WARN("Scoring_UP is: %f",scoring_up);
            scoring_down = (HEIGHT_PENALTY * abs((temp_z-GEARBOX_BOTTOM_DOWN_Z)/GEARBOX_BOTTOM_DOWN_Z))+(POINTS_PENALTY * abs((temp_pts-GEARBOX_BOTTOM_DOWN_PTS)/GEARBOX_BOTTOM_DOWN_PTS));
            ROS_WARN("Scoring_DOWN is: %f",scoring_down);
            scoring_side = (HEIGHT_PENALTY * abs((temp_z-GEARBOX_BOTTOM_SIDE_Z))/GEARBOX_BOTTOM_SIDE_Z)+(POINTS_PENALTY * abs((temp_pts-GEARBOX_BOTTOM_SIDE_PTS)/GEARBOX_BOTTOM_SIDE_PTS));
            ROS_WARN("Scoring_SIDE is: %f",scoring_side);
            final_score = min( min(scoring_up, scoring_down), scoring_side);

            //! Determine what pose the object is, and remember the poses
            if(final_score == scoring_down){
                poses.push_back(0);
            } else if(final_score == scoring_up){
                poses.push_back(1);
            } else if (final_score == scoring_side)
            {
                poses.push_back(2);
                final_score = final_score*POSE_PENALTY;
            }
            scores.push_back(final_score);
            lookup_table.push_back(counters);


        }
        //! SORTING!!!!
        for(int i = 0; i < total_blobs; i++){
            for (int j = 0; j < total_blobs-i-1; j++){
                if (scores[j] > scores[j+1]) {
                    float temp = scores[j];
                    scores[j] = scores[j+1];
                    scores[j+1] = temp;

                    int temp_2 = lookup_table[j];
                    lookup_table[j] = lookup_table[j+1];
                    lookup_table[j+1] = temp_2;

                    int temp_3 = poses[j];
                    poses[j] = poses[j+1];
                    poses[j+1] = temp_3;
                }
            }  
        }

        //! Printout for debug!
        for(int i = 0; i<total_blobs;i++){
            ROS_WARN("==================Scoring RESULT-- Find **BOTTOM** Part==================");
            ROS_WARN("[%d]-BEST CONFIDENT PICK IS: %d, with error of: %f, with recognized pose of %d",i,lookup_table[i], scores[i],poses[i]);    
        }

    } else if (part_id ==part_codes::part_codes::GEARBOX_TOP)
    {
        ROS_WARN("[gearbox_finder_fnc=seperation] Seperating for TOP gearbox part now...");

       //! Vectors for temporarily storing scores and corresponding poses 
        vector <float> scores; 
        vector <int> poses;  //Up = 1; Down = 0; Side = 2

        //! Loop through all posible blobs

        int total_blobs = avg_z_heights.size();
        for(int counters = 0; counters < total_blobs; counters ++){
            float temp_z = avg_z_heights[counters];
            float temp_pts = npts_blobs[counters];
            float scoring_up, scoring_down, scoring_side, final_score;

            //! Compare the poses with three possible poses, produce a final score and tell us what pose it is
            // We are using a nominal result, to make it dimensionless.
            scoring_up = (HEIGHT_PENALTY * abs((temp_z-GEARBOX_TOP_UP_Z)/GEARBOX_TOP_UP_Z))+(POINTS_PENALTY * abs((temp_pts-GEARBOX_TOP_UP_PTS)/GEARBOX_TOP_UP_PTS));
            ROS_WARN("Scoring_UP is: %f",scoring_up);
            scoring_down = (HEIGHT_PENALTY * abs((temp_z-GEARBOX_TOP_DOWN_Z)/GEARBOX_TOP_DOWN_Z))+(POINTS_PENALTY * abs((temp_pts-GEARBOX_TOP_DOWN_PTS)/GEARBOX_TOP_DOWN_PTS));
            ROS_WARN("Scoring_DOWN is: %f",scoring_down);
            scoring_side = (HEIGHT_PENALTY * abs((temp_z-GEARBOX_TOP_SIDE_Z))/GEARBOX_TOP_SIDE_Z)+(POINTS_PENALTY * abs((temp_pts-GEARBOX_TOP_SIDE_PTS)/GEARBOX_TOP_SIDE_PTS));
            ROS_WARN("Scoring_SIDE is: %f",scoring_side);
            final_score = min( min(scoring_up, scoring_down), scoring_side);
            //! Determine what pose the object is, and remember the poses
            if(final_score == scoring_down){
                poses.push_back(0);
            } else if(final_score == scoring_up){
                poses.push_back(1);
            } else if (final_score == scoring_side)
            {
                poses.push_back(2);
                final_score = final_score*POSE_PENALTY;
            }
            scores.push_back(final_score);
            lookup_table.push_back(counters);
        }
        //! SORTING!!!!
        for(int i = 0; i < total_blobs; i++){
            for (int j = 0; j < total_blobs-i-1; j++){
                if (scores[j] > scores[j+1]) {
                    float temp = scores[j];
                    scores[j] = scores[j+1];
                    scores[j+1] = temp;

                    int temp_2 = lookup_table[j];
                    lookup_table[j] = lookup_table[j+1];
                    lookup_table[j+1] = temp_2;

                    int temp_3 = poses[j];
                    poses[j] = poses[j+1];
                    poses[j+1] = temp_3;
                }
            }  
        }
        //! Printout for debug!
        for(int i = 0; i<total_blobs;i++){
            ROS_WARN("==================Scoring RESULT-- Find **TOP** Part==================");
            ROS_WARN("[%d]-CONFIDENT PICK IS: %d, with error of: %f, with recognized pose of %d",i,lookup_table[i], scores[i],poses[i]);    
        }   
    }else {

        ROS_ERROR("[gearbox_finder_fnc=seperation] COMPONENT ID NOT KNOWN!");
    }
}

//! Find Gearbox Top MAIN
bool ObjectFinder::find_gearbox_tops
                                    (
                                        float table_height,
                                        vector<float> &x_centroids_wrt_robot,
                                        vector<float> &y_centroids_wrt_robot,
                                        vector<float> &avg_z_heights,
                                        vector<float> &npts_blobs,
                                        vector<geometry_msgs::PoseStamped> &object_poses
                                    )
{
    ROS_ERROR("---------------LOOK FOR GEARBOZ TOP!-------------------------");
    
    //! Variable Initialization            
    geometry_msgs::PoseStamped object_pose; //* Used to populate the final message

    //! These are the value will get passed back... Initialize them now...
    x_centroids_wrt_robot.clear();
    y_centroids_wrt_robot.clear();
    avg_z_heights.clear();
    npts_blobs.clear();
    object_poses.clear();
    viable_labels_.clear();

    //! Initialize parameter used for box filter
    Eigen::Vector4f box_pt_min, box_pt_max;
    box_pt_min << MIN_X_GEARBOX, MIN_Y_GEARBOX, table_height + MIN_DZ_GEARBOX, 0; //from MIN_DZ above table top
    box_pt_max << MAX_X_GEARBOX, MAX_Y_GEARBOX, table_height + MAX_DZ_GEARBOX, 0;

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

    //! For debug the information
    int total_blob_count = x_centroids_wrt_robot.size();
    ROS_WARN("====================DEBUG [gearbox_finder_fnc=TOP]====================");
    ROS_WARN("Totoal Found Blobs: %d.",total_blob_count);
    for (int counter = 0; counter <total_blob_count; counter ++){
        ROS_WARN("label %d has %f points, avg height %f and centroid %f, %f, angle from newman: %f.", counter, npts_blobs[counter], avg_z_heights[counter],x_centroids_wrt_robot[counter], y_centroids_wrt_robot[counter],g_orientations[counter]);
    }
    ROS_WARN("====================END DEBUG [gearbox_finder_fnc=TOP]====================");

    //! Call seperation here to seperate gearbox bottom and gearbox top
    vector <int> valid_component_list;
    splitGearboxTopBottom(valid_component_list,part_codes::part_codes::GEARBOX_TOP,avg_z_heights,npts_blobs);

    //*** From here, in order to refer to object, we need to do: valid_component_list[i_object] as our original counter.
    int valid_components_count = valid_component_list.size(); // This gets how many valid gearbox component are there


    //! Update the object_pose for robot to grasp
    object_poses.clear();
    ROS_ERROR("!!!!!!!LOOK FOR GEARBOZ TOP!");

    //* Fault Handling
    if (valid_components_count < 1){
        ROS_ERROR("[gearbox_finder_fnc=TOP]OBJECT NOT FOUND!");
        return false; //background is object 0    
    } else
    {
        ROS_WARN("[gearbox_finder_fnc=TOP]CURRENT VALID COMPONENT COUNT IS: %d",valid_components_count);

    }
    
    
    //* Pre Publishing
    object_pose.header.frame_id = "torso_lift_link";
    for (int i_object = 0; i_object < valid_components_count; i_object++) {
        //? Position
        object_pose.pose.position.x = x_centroids_wrt_robot[valid_component_list[i_object]];
        object_pose.pose.position.y = y_centroids_wrt_robot[valid_component_list[i_object]];
        object_pose.pose.position.z = table_height + TABLE_GRASP_CLEARANCE_GEARBOX;
        //? Orientation
        object_pose.pose.orientation = g_vec_of_quat[valid_component_list[i_object]];
        //? Push Back
        object_poses.push_back(object_pose);
    }
    return true;
}

//! Find Gearbox Bottom MAIN
bool ObjectFinder::find_gearbox_bottoms
                                        (
                                            float table_height,
                                            vector<float> &x_centroids_wrt_robot,
                                            vector<float> &y_centroids_wrt_robot,
                                            vector<float> &avg_z_heights,
                                            vector<float> &npts_blobs,
                                            vector<geometry_msgs::PoseStamped> &object_poses
                                        )
 {
    ROS_ERROR("!!!!!!!LOOK FOR GEARBOX BOTTOM!");

    //! Variable Initialization            
    geometry_msgs::PoseStamped object_pose; //* Used to populate the final message

    //! These are the value will get passed back... Initialize them now...
    x_centroids_wrt_robot.clear();
    y_centroids_wrt_robot.clear();
    avg_z_heights.clear();
    npts_blobs.clear();
    object_poses.clear();
    viable_labels_.clear();
    g_orientations.clear();
    //! Initialize parameter used for box filter
    Eigen::Vector4f box_pt_min, box_pt_max;
    box_pt_min << MIN_X_GEARBOX, MIN_Y_GEARBOX, table_height + MIN_DZ_GEARBOX, 0; //from MIN_DZ above table top
    box_pt_max << MAX_X_GEARBOX, MAX_Y_GEARBOX, table_height + MAX_DZ_GEARBOX, 0;

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

    //! For debug the information
    int total_blob_count = x_centroids_wrt_robot.size();
    ROS_WARN("====================DEBUG [gearbox_finder_fnc=bottom]====================");
    ROS_WARN("Totoal Found Blobs: %d.",total_blob_count);
    for (int counter = 0; counter <total_blob_count; counter ++){
        ROS_WARN("label %d has %f points, avg height %f and centroid %f, %f, angle from newman: %f.", counter, npts_blobs[counter], avg_z_heights[counter],x_centroids_wrt_robot[counter], y_centroids_wrt_robot[counter],g_orientations[counter]);
    }
    ROS_WARN("====================END DEBUG [gearbox_finder_fnc=bottom]====================");

    //! Call seperation here to seperate gearbox bottom and gearbox top
    vector <int> valid_component_list;
    splitGearboxTopBottom(valid_component_list,part_codes::part_codes::GEARBOX_BOTTOM,avg_z_heights,npts_blobs);

    //*** From here, in order to refer to object, we need to do: valid_component_list[i_object] as our original counter.
    int valid_components_count = valid_component_list.size(); // This gets how many valid gearbox component are there


    //! Update the object_pose for robot to grasp
    object_poses.clear();

    //* Fault Handling
    if (valid_components_count < 1){
        ROS_ERROR("[gearbox_finder_fnc=bottom]OBJECT NOT FOUND!");
        return false; //background is object 0    
    } else
    {
        ROS_WARN("[gearbox_finder_fnc=bottom]CURRENT VALID COMPONENT COUNT IS: %d",valid_components_count);

    }
    
    
    //* Pre Publishing
    object_pose.header.frame_id = "torso_lift_link";
    for (int i_object = 0; i_object < valid_components_count; i_object++) {
        //? Position
        object_pose.pose.position.x = x_centroids_wrt_robot[valid_component_list[i_object]];
        object_pose.pose.position.y = y_centroids_wrt_robot[valid_component_list[i_object]];
        object_pose.pose.position.z = table_height + TABLE_GRASP_CLEARANCE_GEARBOX;
        //? Orientation
        object_pose.pose.orientation = g_vec_of_quat[valid_component_list[i_object]];
        //? Push Back
        object_poses.push_back(object_pose);
    }
    return true;
}