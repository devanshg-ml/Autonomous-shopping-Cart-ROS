//source code for gear-finder fncs


//Note: may want to use functions defined in object_helper_fncs.cpp, including:
// void ObjectFinder::find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector4f box_pt_min,
//        Eigen::Vector4f box_pt_max, vector<int> &indices)
// float ObjectFinder::find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz)

// void ObjectFinder::find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion) 

//  void ObjectFinder::blob_finder(vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot, ...)

//void ObjectFinder::convert_transformed_cloud_to_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr, vector<int> indices)
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <math.h>
#include <stdio.h> 
using namespace std;
using namespace cv;
const float MIN_X_g = 0.4; //include points starting 0.4m in front of robot
const float MAX_X_g = 0.7; //include points out to 0.9m in front of robot
const float MIN_Y_g = -0.7; //include points starting -0.5m to left of robot
const float MAX_Y_g = 0.7; //include points up to 0.5m to right of robot
const float MIN_DZ_g = 0.02; //box filter from this height above the table top
const float MAX_DZ_g = 0.2; //consider points up to this height above table top
const float MIN_DZ_GEAR = 0.01;
const float MAX_DZ_GEAR = 0.1;

const float min_small_gear_pts = 95.0;
const float max_small_gear_pts = 250.0;
const float min_large_gear_pts = 500.0;
const float max_large_gear_pts = 900.0;
const float MIN_Height_LARGE_GEAR = 18;
const float MIN_Height_SMALL_GEAR = 15;


bool sortbysec(const pair<int,int> &a, 
              const pair<int,int> &b) 
{ 
    return (a.second < b.second); 
} 

bool ObjectFinder::find_small_gears(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses) {
    geometry_msgs::PoseStamped object_pose;
    object_poses.clear();
    Eigen::Vector4f box_pt_min, box_pt_max;
    vector<float> countours_area;
    vector<float> countours_orientation;
    box_pt_min << MIN_X_g, MIN_Y_g, table_height + 0.003 ,0; //1cm above table top
    box_pt_max << MAX_X_g, MAX_Y_g, table_height+ 0.04,0;
    ROS_INFO("Finding blobs");
    find_indices_box_filtered(transformed_cloud_ptr_, box_pt_min, box_pt_max, indices_);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices_, *box_filtered_cloud_ptr_); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr_, ros_box_filtered_cloud_); //convert to ros message for publication and display
    float npts_cloud = indices_.size();



    //convert point cloud to top-down 2D projection for OpenCV processing
    convert_transformed_cloud_to_2D(transformed_cloud_ptr_, indices_);

    blob_finder(x_centroids_wrt_robot, y_centroids_wrt_robot, avg_z_heights, npts_blobs,viable_labels_);

    // Doing PCA analysis to find the orientation fo each blob


    imwrite("gearimage.png",g_dst);


    XformUtils transform;
    int nlabels = viable_labels_.size();
    if ( nlabels>0)
    {
        
    for (int label = 0; label < nlabels; ++label) {
       
        if (avg_z_heights[label] <= MIN_Height_SMALL_GEAR) 
        
            if (npts_blobs[label] >=  min_small_gear_pts && npts_blobs[label] <=  max_small_gear_pts ) 
            {
                cout<<"inside thje blolb.....\n";
                ROS_INFO("label %d has %d points and  avg height %f:", label, (int) npts_blobs[label], avg_z_heights[label]);

                object_pose.pose.position.x = x_centroids_wrt_robot[label];
                object_pose.pose.position.y = y_centroids_wrt_robot[label];
                object_pose.pose.position.z = 0.005;
                object_pose.pose.orientation = g_vec_of_quat[label];
                object_poses.push_back(object_pose);

            }
            else continue;
        
        
    }
}
else return false;

return true;
}


//placeholder for new code:
bool ObjectFinder::find_large_gears(float table_height, vector<float> &x_centroids_wrt_robot, vector<float> &y_centroids_wrt_robot,
            vector<float> &avg_z_heights, vector<float> &npts_blobs,  vector<geometry_msgs::PoseStamped> &object_poses) {

   
    vector<float> e_x_centroids_wrt_robot, e_y_centroids_wrt_robot;
    vector<float> e_avg_z_heights;
    vector<float> e_npts_blobs;

    geometry_msgs::PoseStamped object_pose;
    object_poses.clear();
    Eigen::Vector4f box_pt_min, box_pt_max;

    box_pt_min << MIN_X_g, MIN_Y_g, table_height + MIN_DZ_GEAR ,0; //1cm above table top
    box_pt_max << MAX_X_g, MAX_Y_g, table_height+ MAX_DZ_GEAR,0;
    ROS_INFO("Finding blobs");
    find_indices_box_filtered(transformed_cloud_ptr_, box_pt_min, box_pt_max, indices_);
    pcl::copyPointCloud(*pclCam_clr_ptr_, indices_, *box_filtered_cloud_ptr_); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr_, ros_box_filtered_cloud_); //convert to ros message for publication and display
    float npts_cloud = indices_.size();



    //convert point cloud to top-down 2D projection for OpenCV processing
    convert_transformed_cloud_to_2D(transformed_cloud_ptr_, indices_);

    blob_finder(x_centroids_wrt_robot, y_centroids_wrt_robot, avg_z_heights, npts_blobs,viable_labels_);

    imwrite("gearimage.png",g_dst);
    Mat src = imread("gearimage.png", CV_LOAD_IMAGE_UNCHANGED);


    int nlabels = viable_labels_.size();

//########################################################################################################
//########################################################################################################
//########################################################################################################
//########################################################################################################
//########################################################################################################
//########################################################################################################

  // tried dilation for multiple broken blobs unsuccessful......
  // the dilation worked but im unable to use blob_finder with this thing.
  /*  
    
    int merge_cnt = 0;
    int morph_size = 3;
    
    if ( nlabels>0)
        {
            
          for (int label = 0; label < nlabels; ++label) {
           // ROS_INFO("label %d has %d points and  avg height %f:", label, (int) npts_blobs[label], avg_z_heights[label]);
            if (avg_z_heights[label] > MIN_Height_LARGE_GEAR) 
            { 
                if (npts_blobs[label] <  min_large_gear_pts)
                {
                 merge_cnt++; 
                }  
            }
          }
        }
    else return false;


    cv::Mat erode_bw_img(128,128,CV_8U,cv::Scalar(0)),erode_dst;
    if(merge_cnt >1)
    {
    
      Mat element = getStructuringElement( MORPH_ELLIPSE, cv::Size( 2*morph_size + 1 , 2*morph_size +1), cv::Point( morph_size, morph_size ) );
      dilate( src, erode_dst, element );
      imwrite("gearimage_erode.png",erode_dst);
      g_dst = erode_dst;
      cvtColor(erode_bw_img, erode_dst, COLOR_GRAY2BGR);
      g_bw_img = erode_bw_img;
      blob_finder(e_x_centroids_wrt_robot, e_y_centroids_wrt_robot, e_avg_z_heights, e_npts_blobs, viable_labels_);
    }

  nlabels = viable_labels_.size();

   if ( nlabels>0)
      {
          
          for (int label = 0; label < nlabels; ++label) {
          //ROS_INFO("label %d has %d points and  avg height %f:", label, (int) npts_blobs[label], avg_z_heights[label]);
          if (e_avg_z_heights[label] > MIN_Height_LARGE_GEAR) 
          { 
              if (e_npts_blobs[label] >=  min_large_gear_pts && e_npts_blobs[label] <=  max_large_gear_pts ) {

                  object_pose.pose.position.x = e_x_centroids_wrt_robot[label];
                  object_pose.pose.position.y = e_y_centroids_wrt_robot[label];
                  object_pose.pose.position.z = 0.005;
                  object_pose.pose.orientation = g_vec_of_quat[label];
                  gear_ori =  g_orientations[label];
                  object_poses.push_back(object_pose);

                  //return true;
              }
              //else continue;
          }
          
      }
   }

    src = imread("gearimage_erode.png", CV_LOAD_IMAGE_UNCHANGED);*/

//########################################################################################################
//########################################################################################################
//########################################################################################################
//########################################################################################################
//########################################################################################################
//########################################################################################################
float gear_ori;

double pi = 3.14159; 

    if ( nlabels>0)
          {
              
           for (int label = 0; label < nlabels; ++label) 
           {
              if (avg_z_heights[label] > MIN_Height_LARGE_GEAR) 
              { 
                  if (npts_blobs[label] >=  min_large_gear_pts && npts_blobs[label] <=  max_large_gear_pts ) 
                  {
                      ROS_INFO("label %d has %d points and  avg height %f:", label, (int) npts_blobs[label], avg_z_heights[label]);
                      object_pose.pose.position.x = x_centroids_wrt_robot[label];
                      object_pose.pose.position.y = y_centroids_wrt_robot[label];
                      object_pose.pose.position.z = 0.005;
                      object_pose.pose.orientation = g_vec_of_quat[label];
                      gear_ori =  g_orientations[label];
                     // object_poses.push_back(object_pose);
                  }
              }
              
          }
       }


    ROS_INFO("Rotating the Image by %f", 90.0 - (float) (gear_ori * (180/pi)));

   // Mat r_bw_img;
   
    Mat grayImg;




    Mat_<uchar> r_bw_img(Nu, Nv);
    Mat_<int> r_labelImage(Nu, Nv);
    // Mat r_labelImage;
    Mat dst(r_bw_img.size(), CV_8UC3);

    r_bw_img = 0;
    r_labelImage = 0;
    Point2f pc(src.cols/2., src.rows/2.);

    Mat r = getRotationMatrix2D(pc, 90.0 - (double) (gear_ori * (180/pi)), 1.0);

    warpAffine(src, dst, r, src.size());

    imwrite("rotated_im.png", dst);
    //dst = imread("rotated_im.png");
  

    ROS_INFO("Done with Rotation of the image.");

    cvtColor(dst, r_bw_img, CV_BGR2GRAY);
    imwrite("bw_rotated_im.png", r_bw_img);
    

    int nLabels = connectedComponents(r_bw_img, r_labelImage, 8);
   // cout<<r_bw_img;

    ROS_INFO("Found New connected componenets. %d",nLabels);

    

    vector<float>  temp_x_centroids, temp_y_centroids,temp_avg_z_heights, temp_npts_blobs;
    temp_avg_z_heights.resize(nLabels);
    temp_npts_blobs.resize(nLabels);
    temp_y_centroids.resize(nLabels);
    temp_x_centroids.resize(nLabels);

      for (int label = 0; label < nLabels; ++label) {
        temp_y_centroids[label] = 0.0;
        temp_x_centroids[label] = 0.0;
        temp_avg_z_heights[label] = 0.0;
        temp_npts_blobs[label] = 0.0;
    }

    for (int r = 0; r < dst.rows; ++r) {
        for (int c = 0; c < dst.cols; ++c) {
            int label = r_labelImage.at<int>(r, c);
             temp_y_centroids[label] += c; //robot y-direction corresponds to columns--will negate later
            temp_x_centroids[label] += r; 
            temp_npts_blobs[label] += 1.0;
            //cout<<temp_npts_blobs[label] <<endl;
            double zval = (float) r_bw_img(r, c);
            temp_avg_z_heights[label] += zval; //check the  order of this
        }
    }

    int gear_label;

    for (int label = 0; label < nLabels; ++label) {
      if (temp_avg_z_heights[label] > MIN_Height_LARGE_GEAR) { //rejects the table surface
         if (temp_npts_blobs[label] >=  min_large_gear_pts && temp_npts_blobs[label] <=  max_large_gear_pts )  {
            gear_label = label;
            break;
         }
      }
    }

    vector<pair<int, int>> blob_pixels;

    for (int label = 0; label < nLabels; ++label) {
      //cout<<"number of points:"<<temp_npts_blobs[label]<<endl;
        if(temp_npts_blobs[label] == 0) continue;
         temp_y_centroids[label] /= temp_npts_blobs[label];
        temp_x_centroids[label] /= temp_npts_blobs[label];
        temp_avg_z_heights[label] /= temp_npts_blobs[label];
        ROS_INFO("label %d has centroid %f, %f, number of points are %f:",label,temp_x_centroids[label],temp_y_centroids[label],temp_npts_blobs[label]);
    }

    vector<int> numPointsLeftOfCentroid;
    vector<int> numPointsRightOfCentroid;

    numPointsLeftOfCentroid.resize(nLabels);
    numPointsRightOfCentroid.resize(nLabels);

    for (int r = 0; r < dst.rows; ++r) {
      for (int c = 0; c < dst.cols; ++c) {
          int label = r_labelImage.at<int>(r, c);
          if (label == gear_label) { //rejects the table surface
              blob_pixels.push_back(make_pair(r, c));
           }
      }
    }

   sort(blob_pixels.begin(), blob_pixels.end());
   int min_x = blob_pixels[0].first;
   int max_x =  blob_pixels[blob_pixels.size()-1].first;

   sort(blob_pixels.begin(), blob_pixels.end(), sortbysec);

   int min_y = blob_pixels[0].second;
   int max_y =  blob_pixels[blob_pixels.size()-1].second;
    
    float mid_x = float(min_x+max_x)/float(2.0);
    float mid_y = float(min_y+max_y)/float(2.0);

    cout<<"Mid point is: ("<<mid_x<<","<<mid_y<<")\n";
  

   for (int r = 0; r < dst.rows; ++r) {
            for (int c = 0; c < dst.cols; ++c) {
                int label = r_labelImage.at<int>(r, c);
                if (temp_avg_z_heights[label] > MIN_Height_LARGE_GEAR) { //rejects the table surface
                    if (temp_npts_blobs[label] >=  min_large_gear_pts && temp_npts_blobs[label] <=  max_large_gear_pts )  {
                        //ROS_INFO("Point in blob: (%i, %i), centroid: (%f, %f)", c, r, temp_x_centroids[label], temp_y_centroids[label]);
                        if(c < mid_y){
                          numPointsLeftOfCentroid[label]++;
                        }
                        else if(c > mid_y){
                          numPointsRightOfCentroid[label]++;
                        }
                     }
                 }
            }
    }
    for (int label = 0; label < nLabels; ++label){
      ROS_INFO("Num points Left: %i , Num Points Right: %i", numPointsLeftOfCentroid[label], numPointsRightOfCentroid[label]);
      if(numPointsLeftOfCentroid[label] > numPointsRightOfCentroid[label]){
        gear_ori = gear_ori + pi;
      }
    }
    
    ROS_INFO("Points in blob_pixels: %lu" , blob_pixels.size());
    ROS_INFO("New Orientation: %f" , gear_ori);

     XformUtils transform;
    object_pose.pose.orientation =  transform.convertPlanarPsi2Quaternion(gear_ori);
    object_poses.push_back(object_pose);
       

}

