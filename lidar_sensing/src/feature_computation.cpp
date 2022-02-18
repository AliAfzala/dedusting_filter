#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include"lidar_sensing/dedusting.h"
#include <pcl/octree/octree_search.h>
#include <iostream>
#include <vector>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <eigen3/Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include<eigen3/Eigen/Eigenvalues>
#include<cmath>
#include<iterator>
#include<algorithm>
#include<chrono>
#include <ros/console.h>

  // Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

  // Topics
static const std::string IMAGE_TOPIC = "velodyne_points";
static const std::string PUBLISH_TOPIC = "feature_topics";

using namespace std::chrono;
using namespace Eigen;

  // ROS Publisher
  ros::Publisher pub;

  void cloud_cb(const sensor_msgs::PointCloud2 cloud_msg)
  {
    // Container for original & filtered data
    auto start = high_resolution_clock::now();
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZICropped (new pcl::PointCloud<pcl::PointXYZI>);
    ROS_INFO_STREAM("Hello from feature Node: " << ros::this_node::getName());
    //pcl::PointCloud<pcl::PointXYZI> *cloudXYZI = new pcl::PointCloud<pcl::PointXYZI>;
    //pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloudXYZIPtr(cloudXYZI);
    //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    //pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(cloud_msg, *cloud);
    pcl::fromPCLPointCloud2 (*cloud,*cloudXYZI) ; 

    //   Croping the point cloud 
    double minX = -10; double minY = 0; double minZ = -3;
    double maxX = 20 ; double maxY = 20 ; double maxZ = 3 ;
    pcl::CropBox<pcl::PointXYZI> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloudXYZI);
    boxFilter.filter(*cloudXYZICropped);


    // Voxelization using octree lib

  float resolution = 0.20f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
  octree.setInputCloud (cloudXYZICropped);
  octree.addPointsFromInputCloud ();
  pcl::octree::OctreePointCloud < pcl::PointXYZI>::AlignedPointTVector searchPoint;
  octree.getOccupiedVoxelCenters( searchPoint);

  // Computing features for each voxel
  std::vector<int> pointIdxVec;
  std::vector<int> voxel_intensity;
  lidar_sensing::dedusting features;


  for (int j = 0; j < searchPoint.size(); ++j)
  {
    if ( (octree.voxelSearch (searchPoint[j], pointIdxVec)) && (pointIdxVec.size() > 3))
      {
        
        int sz = pointIdxVec.size();
        //std::cout<< " number of points inside voxel is " << sz << std::endl ; 
        Matrix<double,Dynamic,3> voxel_points;
        voxel_points.conservativeResize(sz,3);

        
        for (std::size_t i = 0; i < pointIdxVec.size (); ++i)
            {
            //voxel_points.conservativeResize(i+1,3);
            //voxel_intensity.conservativeResize(i+1,1); 
            //std::cout<<i<< std::endl;
            voxel_points (i,0) = (*cloudXYZICropped)[pointIdxVec[i]].x ; 
            //std::cout<< " Hello World!" << std::endl;
            voxel_points(i,1) =( (*cloudXYZICropped)[pointIdxVec[i]].y ) ;
            voxel_points(i,2)= ( (*cloudXYZICropped)[pointIdxVec[i]].z);
            //std::cout<< " Hello World!" << std::endl;
            //voxel_intensity (i,0) = (*cloud)[pointIdxVec[i]].intensity;
            voxel_intensity.push_back((*cloudXYZI)[pointIdxVec[i]].intensity);
           
            } 


        EigenSolver< Matrix<double,Dynamic,3> > es(voxel_points.transpose()*voxel_points);      
        MatrixXd D = es.pseudoEigenvalueMatrix();
        MatrixXd V = es.pseudoEigenvectors();  

        double voxel_roughness = std::abs(D(2,2)); 
        double voxel_slope = std::sin( V(2,2)/ V.col(2).norm() );
        double sum = std::accumulate(voxel_intensity.begin(), voxel_intensity.end(), 0.0);
        double voxel_intensity_mean = sum / voxel_intensity.size();
        double sq_sum = std::inner_product(voxel_intensity.begin(), voxel_intensity.end(), voxel_intensity.begin(), 0.0);
        double voxel_intensity_std = std::sqrt(sq_sum / voxel_intensity.size() - voxel_intensity_mean *voxel_intensity_mean);
        
        features.voxel_mean_intensity.push_back(voxel_intensity_mean);
        features.voxel_std_intensity.push_back(voxel_intensity_std);
        features.voxel_slope.push_back(voxel_slope);
        features.voxel_roughness.push_back(voxel_roughness);

        pointIdxVec.clear();

        
        }
      
      
  }
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  ROS_INFO_STREAM( "Time for computing features : " << duration.count());


    // Perform the actual filtering:Voxel Grid
    /*
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (cloud_filtered);
    */

   // Perform the actual filter: ROR

  /*
   pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
   outrem.setInputCloud(cloudPtr);
   outrem.setRadiusSearch(0.8);
   outrem.setMinNeighborsInRadius (2);
   outrem.setKeepOrganized(true);
    // apply filter
   outrem.filter (cloud_filtered);


    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);


  */

    // Publish the data
    pub.publish (features);
  }

  int main (int argc, char** argv)
  {
    //lidar_sensing::dedusting a;
    //a.voxel_mean_intensity.push_back(1);
    // Initialize the ROS Node "roscpp_pcl_example"
    ros::init (argc, argv, "feature_node");
    ros::NodeHandle nh;

    // Print "Hello" message with node name to the terminal and ROS log file
    ROS_INFO_STREAM("Hello from feature Node: " << ros::this_node::getName());

    // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
    ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 50, cloud_cb);

    // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
    pub = nh.advertise<lidar_sensing::dedusting>(PUBLISH_TOPIC, 50);

    // Spin
    ros::spin();

    // Success
    return 0;
  }