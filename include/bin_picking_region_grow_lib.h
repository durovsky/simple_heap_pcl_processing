/*********************************************************************************************//**
* @file bin_picking_region_grow_lib.h
* 
* class PCL_recognizer handles complete pointcloud recognition
*
* Copyright (c)
* Frantisek Durovsky 
* Department of Robotics
* Technical University Kosice 
* February 2015
*   
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/
#ifndef BIN_PICKING_REGION_GROW_LIB
#define BIN_PICKING_REGION_GROW_LIB

//Standard ROS Headers
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

//PCL basic headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//PCL filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

//PCL Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>

//PCL other headers
#include <pcl/surface/convex_hull.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include "pcl_ros/transforms.h"

//Bin picking region grow service header
#include <bin_picking_region_grow/bin_picking_region_grow_service.h>

//////////////////////////////////////////////////////////////////////////////////////////

class PCL_recognizer
{
public:

   PCL_recognizer(ros::NodeHandle nh);
   ~PCL_recognizer();
   void cloud_Callback(const sensor_msgs::PointCloud2::ConstPtr &data);
   bool object_pose_Callback(bin_picking_region_grow::bin_picking_region_grow_service::Request &req,
                             bin_picking_region_grow::bin_picking_region_grow_service::Response &res);

   void visualize_basic_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
   void visualize_cloud_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals);
   void visualize_clustered_cloud();

private:

   //ROS messaging
   ros::Publisher cloud_pub;
   ros::Publisher marker_pub;

   //Additional variables
   sensor_msgs::PointCloud2 topic_cloud;	//Pointcloud variable
   bool first_point_cloud_recieved;		    //Pointcloud message flag
   float nx,ny,nz;                          //Normal vector
   
   //Variables for launch file arguments
   std::string world_frame;
   std::string camera_frame;

   double voxel_leaf_size;
   double x_filter_min,
          x_filter_max,
          y_filter_min,
          y_filter_max,
          z_filter_min,
          z_filter_max;

   double plane_max_iter,
          plane_dist_thresh,
          smoothness_threshold,
          curvature_threshold;

   int min_cluster_size,
       max_cluster_size;

};

#endif //BIN_PICKING_REGION_GROW_LIB
















