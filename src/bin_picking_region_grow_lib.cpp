/*********************************************************************************************//**
* @file simple_heap_lib.cpp
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
#ifndef BIN_PICKING_REGION_GROW_LIB_CPP
#define BIN_PICKING_REGION_GROW_LIB_CPP

///////////////////////////////////////////////////////////////////////////////////////////////

#include <bin_picking_region_grow_lib.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PCL_recognizer::PCL_recognizer(ros::NodeHandle *nh) :
  first_point_cloud_recieved(false)
  {
     nh->getParam("world_frame", world_frame);
     nh->getParam("camera_frame", camera_frame);
     nh->getParam("voxel_leaf_size", voxel_leaf_size);
     nh->getParam("x_filter_min", x_filter_min);
     nh->getParam("x_filter_max", x_filter_max);
     nh->getParam("y_filter_min", y_filter_min);
     nh->getParam("y_filter_max", y_filter_max);
     nh->getParam("z_filter_min", z_filter_min);
     nh->getParam("z_filter_max", z_filter_max);
     nh->getParam("plane_max_iterations", plane_max_iter);
     nh->getParam("plane_distance_threshold", plane_dist_thresh);
     nh->getParam("min_cluster_size", min_cluster_size);
     nh->getParam("max_cluster_size", max_cluster_size);
     nh->getParam("smoothness_threshold", smoothness_threshold);
     nh->getParam("curvature_threshold", curvature_threshold);
       
     cloud_pub = nh->advertise<sensor_msgs::PointCloud2>("object_cluster",1);
     marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker",1);

  }

PCL_recognizer::~PCL_recognizer()
  {
     // delete
  }
  
void
PCL_recognizer::cloud_Callback(const sensor_msgs::PointCloud2::ConstPtr &data)
{
    if(first_point_cloud_recieved == false)
    {
        first_point_cloud_recieved = true;
        std::cout << std::endl << "First pointcloud recieved" << std::endl;
    }
    topic_cloud = *data;
}

void
PCL_recognizer::visualize_basic_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(cloud_in);
    while (!viewer.wasStopped ())
    {
       ros::Duration(0.1).sleep();
    }
}

void
PCL_recognizer::visualize_cloud_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, "cloud");
    // Display one normal out of 20, as a line of length 3cm.
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_in, normals, 20, 0.03, "normals");
    while (!viewer->wasStopped())
    {
       viewer->spinOnce(100);
       boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    //Clean up
    delete &viewer;
}



bool
PCL_recognizer::object_pose_Callback(bin_picking_region_grow::bin_picking_region_grow_service::Request &req,
                                     bin_picking_region_grow::bin_picking_region_grow_service::Response &res)
{
  if(req.coordinates_demand == true) 
  {
     //Spin tf transform listener
     tf::TransformListener listener;
     ros::Duration(0.5).sleep();
     //========================================================
     //Transform pointcloud from camera frame to world frame
     //========================================================
     topic_cloud.header.frame_id = camera_frame;
     topic_cloud.header.stamp = ros::Time(0);
   
     //Transform
     tf::StampedTransform stransform;
     try
     {
        listener.waitForTransform(world_frame,camera_frame,ros::Time(0), ros::Duration(5) );
        listener.lookupTransform(world_frame, camera_frame, ros::Time(0), stransform);
     }
     catch (tf::TransformException ex)
     {
        ROS_ERROR("%s", ex.what());
     }
     
     sensor_msgs::PointCloud2 transformed_cloud;
     pcl_ros::transformPointCloud(world_frame, topic_cloud, transformed_cloud, listener);

     //Convert to pcl
     PointCloudT::Ptr cloud_in (new PointCloudT);
     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
     pcl::fromROSMsg(transformed_cloud, *cloud_in);

     //======================================================
     //Crop box filter
     //======================================================
     PointCloudT::Ptr cloud_crop_boxed (new PointCloudT);
     pcl::CropBox<PointT> crop;
     crop.setInputCloud(cloud_in);
     Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
     Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
     crop.setMin(min_point);
     crop.setMax(max_point);
     crop.filter(*cloud_crop_boxed);
     std::cout << "Cropped cloud size: " << cloud_crop_boxed->size() << std::endl;
     
     //=======================================================
     //Voxel grid
     //=======================================================
     PointCloudT::Ptr cloud_voxel_filtered (new PointCloudT);
     pcl::VoxelGrid<PointT> voxel_filter;
     voxel_filter.setInputCloud(cloud_crop_boxed);
     voxel_filter.setLeafSize(float(voxel_leaf_size), float(voxel_leaf_size), float(voxel_leaf_size));
     voxel_filter.filter(*cloud_voxel_filtered);
     std::cout << "Downsampled cloud size: " << cloud_voxel_filtered->size() << std::endl;
     
     //========================================================
     //Remove NANs
     //========================================================
     std::vector<int> mapping;
     pcl::removeNaNFromPointCloud(*cloud_voxel_filtered, *cloud_voxel_filtered, mapping);
     
     //==================================================
     //Convert pointcloud PCL->ROS and visualize
     //==================================================
     sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
     pcl::toROSMsg(*cloud_voxel_filtered, *pc2_cloud);
     pc2_cloud->header.frame_id = world_frame;
     pc2_cloud->header.stamp = ros::Time::now();
     cloud_pub.publish(pc2_cloud);
   
     //========================================================
     //Normal estimation
     //========================================================
     pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
     pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
     normal_estimator.setSearchMethod (tree);
     normal_estimator.setInputCloud (cloud_voxel_filtered);
     normal_estimator.setKSearch (50);
     normal_estimator.compute (*normals);
     
     //=========================================================
     //Region grow segmentation
     //=========================================================
     pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
     reg.setMinClusterSize (min_cluster_size);
     reg.setMaxClusterSize (max_cluster_size);
     reg.setSearchMethod (tree);
     reg.setNumberOfNeighbours (50);
     reg.setInputCloud (cloud_voxel_filtered);
     reg.setInputNormals (normals);
     reg.setSmoothnessThreshold (smoothness_threshold / 180.0 * M_PI);
     reg.setCurvatureThreshold (curvature_threshold);

     std::vector <pcl::PointIndices> clusters;
     reg.extract (clusters);
     std::cout << "Nuber of clusters detected: " << clusters.size() << std::endl;
     std::cout << "--------------------------------------------------" << std::endl;
     
//     //========================================
//     //Visualization of colored cloud
//     //========================================
//     pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//     pcl::visualization::CloudViewer viewer ("Cluster viewer");
//     viewer.showCloud(colored_cloud);
//     while (!viewer.wasStopped ())
//     {
//         ros::Duration(1).sleep();
//     }

     //==========================================
     //Process detected clusters
     //==========================================
     if(clusters.size() > 0)
     {
        int currentClusterNum = 1;
        geometry_msgs::Pose single_object_pose;                //Current object pose
        std::vector<geometry_msgs::Pose> object_poses;         //Vector of object poses

        geometry_msgs::Vector3 single_object_normal;           //Current object normal
        std::vector<geometry_msgs::Vector3> object_normals;    //Vector of object normals

        int max_z_coordinate_object_index = 0;
        float max_z_coordinate = 0;
	
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
        {
          std::cout << "Processing cluster n." << currentClusterNum << "..." << std::endl;
          PointCloudT::Ptr cluster(new PointCloudT);
          for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
              cluster->points.push_back(cloud_voxel_filtered->points[*point]);
          cluster->width = cluster->points.size();
          cluster->height = 1;
          cluster->is_dense = true;

          //Variables for storing results
          PointCloudT::Ptr cluster_filtered(new PointCloudT);
          PointCloudT::Ptr cluster_plane_raw(new PointCloudT);

          //-------------------------------------
          //Cluster plane segmentation
          //-------------------------------------
          pcl::SACSegmentation<PointT> seg;
          pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
          pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

          seg.setOptimizeCoefficients(true);
          seg.setModelType(pcl::SACMODEL_PLANE);
          seg.setMethodType(pcl::SAC_RANSAC);
          seg.setMaxIterations(plane_max_iter);
          seg.setDistanceThreshold(plane_dist_thresh);

          //Segment the largest planar komponent from cluster
          seg.setInputCloud (cluster);
          seg.segment(*inliers, *coefficients);

          //Extract the planar inliers from the input cloud
          pcl::ExtractIndices<PointT> extract;
          extract.setInputCloud(cluster);
          extract.setIndices(inliers);
          extract.setNegative(false);

          //Get the points associated with the planar surface
          extract.filter(*cluster_plane_raw);
          std::cout << "Planar cloud size: " << cluster_plane_raw->points.size() << std::endl;

          //Remove the planar inliers, extract the rest
          extract.setNegative(true);
          extract.filter(*cluster_filtered);
     
          //--------------------------------------
          //Statistical outlier removal
          //--------------------------------------
          PointCloudT::Ptr cluster_plane(new PointCloudT);
          pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
          sor.setInputCloud (cluster_plane_raw);
          sor.setMeanK (50);
          sor.setStddevMulThresh (1.0);
          sor.filter (*cluster_plane);

          //--------------------------------------
          //Convex Hull + normal estimation
          //--------------------------------------
          PointCloudT::Ptr convexHull (new PointCloudT);
          pcl::ConvexHull<pcl::PointXYZ> hull;
          std::vector<pcl::Vertices> polygons_alpha;
          hull.setComputeAreaVolume(true);
          hull.setInputCloud(cluster_plane);
          hull.reconstruct(*convexHull,polygons_alpha);
	  
          std::vector <int> vertex_index;
          float curvature;

          for(int k = 0; k < convexHull->width; k++)
             vertex_index.push_back(k);
          
          pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
          ne.computePointNormal(*convexHull, vertex_index, nx,ny,nz, curvature);

          Eigen::Vector4f object_centroid;
          pcl::compute3DCentroid(*cluster_plane, object_centroid);

          //Detect and save the index of highest "z" centroid coordinate
          if(object_centroid[2] > max_z_coordinate)
          {
             max_z_coordinate = object_centroid[2];
             max_z_coordinate_object_index = currentClusterNum - 1;
          }

          float rx, ry, rz;
          rx = -((M_PI/2)-acos(ny));
          ry = ((M_PI/2)-acos(nx));
          rz = acos(nz);

          tf::Quaternion object_orientation;
          object_orientation.setRPY(rx,ry,0);

          //Save current object pose
          single_object_pose.position.x = object_centroid[0];
          single_object_pose.position.y = object_centroid[1];
          single_object_pose.position.z = object_centroid[2];

          single_object_pose.orientation.x = object_orientation.getX();
          single_object_pose.orientation.y = object_orientation.getY();
          single_object_pose.orientation.z = object_orientation.getZ();
          single_object_pose.orientation.w = object_orientation.getW();

          object_poses.push_back(single_object_pose);

          //Save current object normals
          single_object_normal.x = nx;
          single_object_normal.y = ny;
          single_object_normal.z = nz;

          object_normals.push_back(single_object_normal);

	  //----------------------------------------------
          //Terminal outputs
          //----------------------------------------------
          std::cout << "Normal vector: [ " << nx << ", "
                                           << ny << ", "
                                           << nz << "]"
                                           << std::endl;

          std::cout << "Cluster RPY: [ " << rx*180/M_PI << ", "
                                         << ry*180/M_PI << ", "
                                         << rz*180/M_PI << "]"
                                         << std::endl;

          std::cout << "Centroid: [ " << object_centroid[0] << ", "
                                      << object_centroid[1] << ", "
                                      << object_centroid[2] << "]"
                                      << std::endl;

          std::cout << "--------------------------------------------------" << std::endl;

          //------------------------------
          //Visualization marker
          //------------------------------
          uint32_t shape = visualization_msgs::Marker::SPHERE;
          visualization_msgs::Marker marker;

          marker.header.frame_id = "base_link";
          marker.header.stamp = ros::Time(0);
          marker.ns = "basic_shapes";
          marker.id = currentClusterNum+1;
          marker.type = shape;
          marker.action = visualization_msgs::Marker::ADD;

          marker.pose.position.x = object_centroid[0];
          marker.pose.position.y = object_centroid[1];
          marker.pose.position.z = object_centroid[2];

	  marker.scale.x = 0.02;
          marker.scale.y = 0.02;
          marker.scale.z = 0.02;

          marker.color.r = 0.0f;
          marker.color.g = 0.0f;
          marker.color.b = 1.0f;
          marker.color.a = 1.0;

          marker.lifetime = ros::Duration(40);
          marker_pub.publish(marker);

          currentClusterNum++;

        }
       
       //-----------------------------------------------------------------------
       //Responding pose and normal of object with highest "z" coordinate
       //-----------------------------------------------------------------------
       single_object_pose = object_poses[max_z_coordinate_object_index];
       single_object_normal = object_normals[max_z_coordinate_object_index];

       std::cout << "Number of objects detected: " << object_poses.size() << std::endl;
       res.x_pose = single_object_pose.position.x;
       res.y_pose = single_object_pose.position.y;
       res.z_pose = single_object_pose.position.z;

       res.x_orientation = single_object_pose.orientation.x;
       res.y_orientation = single_object_pose.orientation.y;
       res.z_orientation = single_object_pose.orientation.z;
       res.w_orientation = single_object_pose.orientation.w;

       res.nx = single_object_normal.x;
       res.ny = single_object_normal.y;
       res.nz = single_object_normal.z;

       res.remaining_objects = currentClusterNum-1;
     }
     else
     {
       //-----------------------------------------------------------------------
       //Responding unreachable pose if no cluster detected
       //-----------------------------------------------------------------------

       std::cout << "No cluster detected " << std::endl;
       res.x_pose = 100;
       res.y_pose = 100;
       res.z_pose = 100;

       res.x_orientation = 0;
       res.y_orientation = 0;
       res.z_orientation = 0;
       res.w_orientation = 1;

       res.nx = 0;
       res.ny = 0;
       res.nz = 0;

       res.remaining_objects = 0;
     }      
  }
   


  return(true);
}


#endif //BIN_PICKING_REGION_GROW_LIB_CPP






