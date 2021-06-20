//  pcl and opencv functions that may be useful in object-finder
void ObjectFinder::find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector4f box_pt_min,
        Eigen::Vector4f box_pt_max, vector<int> &indices) {
    /**/
    //int npts = input_cloud_ptr->points.size();
    //Eigen::Vector3f pt;
    indices.clear();
    cout << "box min: " << box_pt_min.transpose() << endl;
    cout << "box max: " << box_pt_max.transpose() << endl;
    /*
    for (int i = 0; i < npts; ++i) {
        pt = input_cloud_ptr->points[i].getVector3fMap();

        //check if in the box:
        if ((pt[0] > box_pt_min[0])&&(pt[0] < box_pt_max[0])&&(pt[1] > box_pt_min[1])&&(pt[1] < box_pt_max[1])&&(pt[2] > box_pt_min[2])&&(pt[2] < box_pt_max[2])) {
            //passed box-crop test; include this point
            //ROS_INFO("point passes test");
            //cout<<"pt passed test: "<<pt.transpose()<<endl;
            indices.push_back(i);
        }
    }
     */
    pcl::CropBox<pcl::PointXYZRGB> cropFilter;
    cropFilter.setInputCloud(input_cloud_ptr);
    cropFilter.setMin(box_pt_min);
    cropFilter.setMax(box_pt_max);

    cropFilter.filter(indices);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices, *crop_filtered_cloud_ptr_);
    //really, want to extract indices and create this cloud:
    pcl::toROSMsg(*crop_filtered_cloud_ptr_, ros_crop_filtered_cloud_); //convert to ros message for publication and display        


    int n_extracted = indices.size();
    cout << " number of points within box = " << n_extracted << endl;
}


//This fnc was used to confirm the table height relative to torso frame;
// It is no longer needed, since the table height, once found, can be hard coded
// i.e., 7cm below the torso-frame origin

float ObjectFinder::find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz) {
    vector<int> indices;
    /*
     Eigen::Vector4f minPoint; 
      minPoint[0]=MIN_X;  // define minimum point x 
      minPoint[1]=MIN_Y;  // define minimum point y 
      minPoint[2]=TABLE_TOP_MIN;  // define minimum point z 
     Eigen::Vector4f maxPoint; 
      maxPoint[0]=MAX_X;  // define max point x 
      maxPoint[1]=MAX_Y;  // define max point y 
      maxPoint[2]=TABLE_TOP_MAX;  // define max point z 
    pcl::CropBox<pcl::PointXYZRGB> cropFilter; 
        cropFilter.setInputCloud (input_cloud_ptr); 
               cropFilter.setMin(minPoint); 
               cropFilter.setMax(maxPoint); 
               //cropFilter.setTranslation(boxTranslatation); 
               //cropFilter.setRotation(boxRotation); 
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop_filtered_cloud_ptr_(new PointCloud<pcl::PointXYZRGB>);           
    //crop_filtered_cloud_ptr_ = new PointCloud<pcl::PointXYZRGB>;
        cropFilter.filter (indices); 
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices, *crop_filtered_cloud_ptr_);
        //really, want to extract indices and create this cloud:
    pcl::toROSMsg(*crop_filtered_cloud_ptr_, ros_crop_filtered_cloud_); //convert to ros message for publication and display    
     */
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object    
    pass.setInputCloud(input_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    double z_table = 0.0;
    int npts_max = 0;
    int npts;
    pcl::PointXYZRGB point;
    int ans;
    for (float z = z_min; z < z_max; z += dz) {
        pass.setFilterLimits(z, z + dz);
        pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
        npts = indices.size();

        if (npts > 0) ROS_INFO("z=%f; npts = %d", z, npts);
        if (npts > npts_max) {
            npts_max = npts;
            z_table = z + 0.5 * dz;
        }
    }
    ROS_INFO("max pts %d at height z= %f", npts_max, z_table);

    pass.setFilterLimits(z_table - 0.5 * dz, z_table + 0.5 * dz);
    //pass.filter (*pass_filtered_cloud_ptr_);
    pass.filter(indices);

    //OOPS: want to select these from transformed point cloud?  (same frame as other displays)
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices, *pass_filtered_cloud_ptr_);
    //really, want to extract indices and create this cloud:
    pcl::toROSMsg(*pass_filtered_cloud_ptr_, ros_pass_filtered_cloud_); //convert to ros message for publication and display

    return z_table;
}

//hard-coded xform for Fetch w/ assumed torso lift and head angle:
//assuming head is tilted down 1.0rad, this is the head-camera frame
// with respect to the torso_lift_link frame, expressed as an Eigen::Affine

//should be  transform from  head_camera_rgb_optical_frame to torso_lift_link
//example Fetch rosbags have: rosrun tf tf_echo torso_lift_link head_camera_rgb_optical_frame

/* Translation: [0.238, 0.019, 0.656]
- Rotation: in Quaternion [-0.660, 0.673, -0.242, 0.230]
            in RPY (radian) [-2.461, -0.009, -1.593]
            in RPY (degree) [-140.999, -0.509, -91.274]
 */

Eigen::Affine3f ObjectFinder::compute_affine_cam_wrt_torso_lift_link(void) {
    //pose of cam w/rt floor
    Eigen::Affine3f affine_cam_wrt_torso;
    Eigen::Matrix3f R_cam;
    Eigen::Vector3f O_cam;
    O_cam << 0.238, 0.019, 0.656; //0.244, 0.02, 0.627;
    affine_cam_wrt_torso.translation() = O_cam;
    Eigen::Vector3f nvec, tvec, bvec;
    //magic numbers, as determined by rosrun tf tf_echo torso_lift_link head_camera_rgb_optical_frame
    // when running robot in Gazebo with headcam tilted down 1.0rad
    //from Fetch bag: 
    //Eigen::Quaternionf q;
    cout << "enter tilt angle (0.897=head_tilt_joint for fetch data): ";
    float angle = 1.0;
    ROS_INFO("using tilt angle %f", angle);
    //cin>>angle;
    Eigen::Quaternionf q(Eigen::AngleAxisf{angle, Eigen::Vector3f
        {0, 1, 0}});
    //    outputAsMatrix(Eigen::Quaterniond{Eigen::AngleAxisd{angle, Eigen::Vector3d{0, 1, 0}}});
    /*
    q.x() = -0.660; 
    q.y() = 0.673; 
    q.z() =-0.242; 
    q.w() = 0.230;  
     * */
    //R_cam = q.normalized().toRotationMatrix();

    nvec << 0, -1, 0;
    tvec << -sin(angle), 0, -cos(angle);
    bvec << cos(angle), 0, -sin(angle);
    R_cam.col(0) = nvec;
    R_cam.col(1) = tvec;
    R_cam.col(2) = bvec;

    affine_cam_wrt_torso.linear() = R_cam;
    return affine_cam_wrt_torso;
}

//given a binary image in bw_img, find and label connected regions  (blobs)
// labelImage will contain integer labels with 0= background, 1 = first blob found,
// ...up to nBlobs
// also creates a colored image such that each blob found gets assigned  a
// unique color, suitable for display and visual interpretation,  though this is
// NOT needed by the robot

void ObjectFinder::find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion) {
    //ROS_INFO("starting identification of plane from data: ");
    int npts = points_mat.cols(); // number of points = number of columns in matrix; check the size

    // first compute the centroid of the data:
    //Eigen::Vector3f centroid; // make this member var, centroid_
    centroid_ = Eigen::MatrixXf::Zero(3, 1); // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt

    //centroid = compute_centroid(points_mat);
    for (int ipt = 0; ipt < npts; ipt++) {
        centroid_ += points_mat.col(ipt); //add all the column vectors together
    }
    centroid_ /= npts; //divide by the number of points to get the centroid    
    cout << "centroid: " << centroid_.transpose() << endl;


    // subtract this centroid from all points in points_mat:
    Eigen::MatrixXf points_offset_mat = points_mat;
    for (int ipt = 0; ipt < npts; ipt++) {
        points_offset_mat.col(ipt) = points_offset_mat.col(ipt) - centroid_;
    }
    //compute the covariance matrix w/rt x,y,z:
    Eigen::Matrix3f CoVar;
    CoVar = points_offset_mat * (points_offset_mat.transpose()); //3xN matrix times Nx3 matrix is 3x3
    //cout<<"covariance: "<<endl;
    //cout<<CoVar<<endl;

    // here is a more complex object: a solver for eigenvalues/eigenvectors;
    // we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
    Eigen::EigenSolver<Eigen::Matrix3f> es3f(CoVar);

    Eigen::VectorXf evals; //we'll extract the eigenvalues to here
    //cout<<"size of evals: "<<es3d.eigenvalues().size()<<endl;
    //cout<<"rows,cols = "<<es3d.eigenvalues().rows()<<", "<<es3d.eigenvalues().cols()<<endl;
    //cout << "The eigenvalues of CoVar are:" << endl << es3d.eigenvalues().transpose() << endl;
    //cout<<"(these should be real numbers, and one of them should be zero)"<<endl;
    //cout << "The matrix of eigenvectors, V, is:" << endl;
    //cout<< es3d.eigenvectors() << endl << endl;
    //cout<< "(these should be real-valued vectors)"<<endl;
    // in general, the eigenvalues/eigenvectors can be complex numbers
    //however, since our matrix is self-adjoint (symmetric, positive semi-definite), we expect
    // real-valued evals/evecs;  we'll need to strip off the real parts of the solution

    evals = es3f.eigenvalues().real(); // grab just the real parts
    ROS_INFO_STREAM("real parts of evals: "<<evals.transpose()<<endl); 

    // our solution should correspond to an e-val of zero, which will be the minimum eval
    //  (all other evals for the covariance matrix will be >0)
    // however, the solution does not order the evals, so we'll have to find the one of interest ourselves

    double min_lambda = evals[0]; //initialize the hunt for min eval
    double max_lambda = evals[0]; // and for max eval
    int min_eval_index=0;
    int max_eval_index=0;
    int mid_eval_index=0;
    //Eigen::Vector3cf complex_vec; // here is a 3x1 vector of double-precision, complex numbers
    //Eigen::Vector3f evec0, evec1, evec2; //, major_axis; 
    //evec0 = es3f.eigenvectors().col(0).real();
    //evec1 = es3f.eigenvectors().col(1).real();
    //evec2 = es3f.eigenvectors().col(2).real();  


    //((pt-centroid)*evec)*2 = evec'*points_offset_mat'*points_offset_mat*evec = 
    // = evec'*CoVar*evec = evec'*lambda*evec = lambda
    // min lambda is ideally zero for evec= plane_normal, since points_offset_mat*plane_normal~= 0
    // max lambda is associated with direction of major axis

    //sort the evals:

    //complex_vec = es3f.eigenvectors().col(0); // here's the first e-vec, corresponding to first e-val
    //cout<<"complex_vec: "<<endl;
    //cout<<complex_vec<<endl;
    plane_normal_ = es3f.eigenvectors().col(0).real(); //complex_vec.real(); //strip off the real part
    major_axis_ = es3f.eigenvectors().col(0).real(); // starting assumptions

    //cout<<"real part: "<<est_plane_normal.transpose()<<endl;
    //est_plane_normal = es3d.eigenvectors().col(0).real(); // evecs in columns

    double lambda_test;
    int i_normal = 0;
    int i_major_axis = 0;
    //loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val
    for (int ivec = 1; ivec < 3; ivec++) {
        lambda_test = evals[ivec];
        if (lambda_test < min_lambda) {
            min_lambda = lambda_test;
            i_normal = ivec; //this index is closer to index of min eval
            min_eval_index = ivec;
            plane_normal_ = es3f.eigenvectors().col(i_normal).real();
        }
        if (lambda_test > max_lambda) {
            max_lambda = lambda_test;
            i_major_axis = ivec; //this index is closer to index of min eval
            major_axis_ = es3f.eigenvectors().col(i_major_axis).real();
            max_eval_index= ivec;
        }
    }
    
    max_lambda_ = max_lambda;
    min_lambda_ = min_lambda;
    
    
    mid_eval_index =0;
    if (max_eval_index==0 ||min_eval_index==0) { //can't be 0; try 1
        mid_eval_index = 1;
        if (max_eval_index==1 ||min_eval_index==1) //can't be 1, must be 2
            mid_eval_index = 2;
    }
    mid_lambda_ =  evals[mid_eval_index];
    
    ROS_INFO("max_lambda = %f; max/mid lambda ratio = %f",max_lambda,max_lambda/mid_lambda_);

    float x_component = major_axis_(0);
    float y_component = -major_axis_(1);
    orientation = atan2(y_component, x_component) - M_PI / 2;

    quaternion = xformUtils.convertPlanarPsi2Quaternion(orientation);
    // at this point, we have the minimum eval in "min_lambda", and the plane normal
    // (corresponding evec) in "est_plane_normal"/
    // these correspond to the ith entry of i_normal
    //cout<<"min eval is "<<min_lambda<<", corresponding to component "<<i_normal<<endl;
    //cout<<"corresponding evec (est plane normal): "<<plane_normal.transpose()<<endl;
    //cout<<"max eval is "<<max_lambda<<", corresponding to component "<<i_major_axis<<endl;
    //cout<<"corresponding evec (est major axis): "<<major_axis_.transpose()<<endl;  

    //what is the correct sign of the normal?  If the data is with respect to the camera frame,
    // then the camera optical axis is z axis, and thus any points reflected must be from a surface
    // with negative z component of surface normal
    if (plane_normal_(2) > 0) plane_normal_ = -plane_normal_; // negate, if necessary

    //cout<<"correct answer is: "<<normal_vec.transpose()<<endl;
    //cout<<"est plane distance from origin = "<<est_dist<<endl;
    //cout<<"correct answer is: "<<dist<<endl;
    //cout<<endl<<endl;    
    //ROS_INFO("major_axis: %f, %f, %f",major_axis_(0),major_axis_(1),major_axis_(2));
    //ROS_INFO("plane normal: %f, %f, %f",plane_normal(0),plane_normal(1),plane_normal(2));
}



//given a binary image in g_bw_img, find and label connected regions  (blobs)
// g_labelImage will contain integer labels with 0= background, 1 = first blob found,
// ...up to nBlobs
// also creates a colored image such that each blob found gets assigned  a
// unique color, suitable for display and visual interpretation,  though this is
// NOT needed by the robot
//OPERATES ON GLOBAL VARS g_bw_img and g_labelImage

void ObjectFinder::blob_finder(vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
        vector<float> &avg_z_heights,
        vector<float> &npts_blobs,
        vector<int> &viable_labels,
        float min_blob_avg_ht, float min_blob_pixels) {

    x_centroids_wrt_robot.clear();
    y_centroids_wrt_robot.clear();
    avg_z_heights.clear();
    npts_blobs.clear();
    viable_labels_.clear();
    max_evals_.clear();
    mid_evals_.clear();
    min_evals_.clear();

    //openCV function to do all the  hard work
    int nLabels = connectedComponents(g_bw_img, g_labelImage, 8); //4 vs 8 connected regions

    ROS_INFO("found %d blobs", nLabels);
    vector<float> temp_x_centroids, temp_y_centroids, temp_avg_z_heights, temp_npts_blobs;
    //g_x_centroids.resize(nLabels);
    //g_y_centroids.resize(nLabels);
    temp_y_centroids.resize(nLabels);
    temp_x_centroids.resize(nLabels);
    temp_avg_z_heights.resize(nLabels);
    temp_npts_blobs.resize(nLabels);
    for (int label = 0; label < nLabels; ++label) {
        temp_y_centroids[label] = 0.0;
        temp_x_centroids[label] = 0.0;
        temp_avg_z_heights[label] = 0.0;
        temp_npts_blobs[label] = 0.0;
    }
    std::vector<Vec3b> colors(nLabels);
    colors[0] = Vec3b(0, 0, 0); //make the background black
    //assign random color to each region label
    for (int label = 1; label < nLabels; ++label) {
        colors[label] = Vec3b((rand()&255), (rand()&255), (rand()&255));
    }

    //compute centroids
    for (int r = 0; r < g_dst.rows; ++r) {
        for (int c = 0; c < g_dst.cols; ++c) {
            int label = g_labelImage.at<int>(r, c);
            Vec3b &pixel = g_dst.at<Vec3b>(r, c);
            pixel = colors[label];
            temp_y_centroids[label] += c; //robot y-direction corresponds to columns--will negate later
            temp_x_centroids[label] += r; //robot x-direction corresponds to rows--will negate later
            temp_npts_blobs[label] += 1.0;
            double zval = (float) g_bw_img(r, c);
            temp_avg_z_heights[label] += zval; //check the  order of this
        }
    }

    //cout<<"checkpoint 1"<<endl;
    for (int label = 0; label < nLabels; ++label) {
        ROS_INFO("label %d has %f points", label, temp_npts_blobs[label]);
        temp_y_centroids[label] /= temp_npts_blobs[label];
        temp_x_centroids[label] /= temp_npts_blobs[label];
        temp_avg_z_heights[label] /= temp_npts_blobs[label];
        //ROS_INFO("label %d has centroid %f, %f:",label,temp_x_centroids[label],temp_y_centroids[label]);
    }
    cout << "filtering by height and area..." << endl;
    //filter to  keep only blobs that are high enough and large enough
    for (int label = 0; label < nLabels; ++label) {
        ROS_INFO("label %d has %d points and  avg height %f:", label, (int) temp_npts_blobs[label], temp_avg_z_heights[label]);
        if (temp_avg_z_heights[label] > min_blob_avg_ht) { //rejects the table surface
            if (temp_npts_blobs[label] > min_blob_pixels) {
                //ROS_INFO("label %d has %f points:",label,temp_npts_blobs[label]);
                x_centroids_wrt_robot.push_back(temp_x_centroids[label]);
                y_centroids_wrt_robot.push_back(temp_y_centroids[label]);
                avg_z_heights.push_back(temp_avg_z_heights[label]);
                npts_blobs.push_back(temp_npts_blobs[label]);
                viable_labels.push_back(label);
                ROS_INFO("label %d has %f points, avg height %f and centroid %f, %f:", label, temp_npts_blobs[label], temp_avg_z_heights[label],
                        temp_x_centroids[label], temp_y_centroids[label]);
                ROS_INFO("saving this blob");

            }
        }
    }


    //colorize the regions and display them:
    //also compute centroids;
    nLabels = viable_labels.size(); //new number of labels, after filtering
    ROS_INFO("found %d viable labels ", nLabels);
   

    g_orientations.clear();
    g_vec_of_quat.clear();
    Eigen::Vector3f e_pt;

    for (int label_index = 0; label_index < nLabels; label_index++) {
        int label = viable_labels[label_index];
        //cout << "label = " << label << endl;
        int npts_blob = npts_blobs[label_index];
        //cout << "npts_blob = " << npts_blob << endl;
        Eigen::MatrixXf blob(3, npts_blob);
        int col_num = 0;
        for (int r = 0; r < g_dst.rows; ++r) {
                for (int c = 0; c < g_dst.cols; ++c){
                        int label_num = g_labelImage.at<int>(r,c);
                        if(label_num == label){
                                e_pt<< (float) c, (float) r,0.0;
                                blob.col(col_num) = e_pt;
                                //ROS_INFO_STREAM("col "<<e_pt.transpose()<<" at col "<<col_num<<endl);
                                col_num++;
                        }
                }
        }


        float angle;
        geometry_msgs::Quaternion quat;
        find_orientation(blob, angle, quat);  
        max_evals_.push_back(max_lambda_);
        mid_evals_.push_back(mid_lambda_);
        min_evals_.push_back(min_lambda_); //all zeros?
        //add pi/2 to factor in rotated camera frame wrt robot
        //angle = 0.0; //FIX ME!!
        g_orientations.push_back(angle);
        //quat = xformUtils.convertPlanarPsi2Quaternion(angle);
        
        g_vec_of_quat.push_back(quat);
    }

    //convert to robot coords:
    //convert to dpixel_x, dpixel_y w/rt image centroid, scale by pixels/m, and add offset from PCL cropping, x,y
    ROS_INFO("converting pixel coords to robot coords...");
    cout << "size of x_centroids_wrt_robot: " << x_centroids_wrt_robot.size() << endl;
    for (int label = 0; label < nLabels; ++label) {
        x_centroids_wrt_robot[label] = ((g_dst.rows / 2) - x_centroids_wrt_robot[label]) / PIXELS_PER_METER + (MIN_X + MAX_X) / 2.0;
        y_centroids_wrt_robot[label] = ((g_dst.cols / 2) - y_centroids_wrt_robot[label]) / PIXELS_PER_METER + (MIN_Y + MAX_Y) / 2.0;
        ROS_INFO("label %d has %d points, avg height %f, centroid w/rt robot: %f, %f,  angle %f:", label, (int) npts_blobs[label], avg_z_heights[label], x_centroids_wrt_robot[label],
                y_centroids_wrt_robot[label], g_orientations[label]);
    }
    //xxx

    //!Here actually colorize the image for debug purposes:
    //colorize the regions; optionally display them:
    if(nLabels > 0) {
    	std::vector<Vec3b> colors(nLabels);
	    colors[0] = Vec3b(0, 0, 0);//background
	    //assign random color to each region label
	    for(int label = 1; label < nLabels; ++label){
	        colors[label] = Vec3b( (rand()&155+100), (rand()&155+100), (rand()&155+100) );
	    }
	    
	    //for display image, assign colors to regions
	    for(int r = 0; r < g_dst.rows; ++r){
	        for(int c = 0; c < g_dst.cols; ++c){
	            int label = g_labelImage.at<int>(r, c);
	            Vec3b &pixel = g_dst.at<Vec3b>(r, c);
	            pixel = colors[label];
	        }
		}
    }

    //! Display the image!
    //* Blob Image Show
    blobbed_image_.header = ros_cloud_.header;
    blobbed_image_.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    blobbed_image_.image = g_dst;
    //* 2D BW Image Show
    black_and_white_.header = ros_cloud_.header;
    black_and_white_.encoding = sensor_msgs::image_encodings::MONO8;
    black_and_white_.image = g_bw_img;
}


//operates on global OpenCV matrices g_labelImage and g_bw_img
//provide a transformed cloud pointer and indices of interest (filtered points above table)

void ObjectFinder::convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> indices) {
    //for each pointcloud point, compute which pixel it maps to and find its z value
    //convert point cloud to top-down 2D projection for OpenCV processing
    float x, y, z;
    int index, u, v;
    Eigen::Vector3f cloud_pt;
    g_bw_img = 0; //initialize image to all black
    g_labelImage = 0; // ditto for result that  will get populated  with region labels
    int npts_cloud = indices.size();
    for (int ipt = 0; ipt < npts_cloud; ipt++) {
        index = indices[ipt];
        cloud_pt = transformed_cloud_ptr->points[index].getVector3fMap();
        z = cloud_pt[2];
        y = cloud_pt[1];
        x = cloud_pt[0];
        //careful: some pointclouds include bad data; test for this
        if ((z == z)&&(x == x)&&(y == y)) { // test for Nan 
            //compute pixel values from metric values
            v = round((y - MIN_Y) * PIXELS_PER_METER); //second component is from y of PCloud
            u = round((x - MIN_X) * PIXELS_PER_METER); //first component is from x of PCloud
            //flip/invert these so image makes sense visually
            u = Nu - u; //robot x-coord w/rt torso is in direction of rows from bottom to  top of image, so negate; 
            v = Nv - v; //robot y-coord w/rt torso points left, so negate to convert to image  column number
            //make sure the computed indices fit within the matrix size:
            if ((u > 0)&&(u < Nu - 1)&&(v > 0)&&(v < Nv - 1)) {
                //set scaling such that 200 corresponds to 10cm height:
                int grayval = z * 1000;
                //cout<<"u,v,zval, grayval = "<<u<<", "<<v<<", "<<z<<", "<<grayval<<endl;

                if (grayval > 255) grayval = 255;

                g_bw_img(u, v) = (unsigned char) grayval; //assign scaled height as gray level; indices u,v are robot -x, -y, = row,col
            }
        }

    }
    
        //* 2D BW Image Show
    black_and_white_.header = ros_cloud_.header;
    black_and_white_.encoding = sensor_msgs::image_encodings::MONO8;
    black_and_white_.image = g_bw_img;
}

