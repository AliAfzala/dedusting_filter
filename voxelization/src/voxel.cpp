#include <ros/ros.h>

  // Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <iostream>
#include <vector>
  // Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>




int main (int argc, char** argv)
  {
    // Initialize the ROS Node "roscpp_pcl_example"
    ros::init (argc, argv, "voxel_node");
    ros::NodeHandle nh;

    // Print "Hello" message with node name to the terminal and ROS log file
    ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());


    // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("voxel_topic", 10);
    // Code for creating voxels

    //pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    //ROS_INFO_STREAM("Points inside the voxel: " << " x=" << (*cloud)[i].x << "y=" << (*cloud)[i].y) ;
  }

  /*float resolution = 500.0f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();

  pcl::PointXYZ searchPoint;

  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  std::vector<int> pointIdxVec;

  if (octree.voxelSearch (searchPoint, pointIdxVec))
  {
    std::cout << "Neighbors within voxel search at (" << searchPoint.x 
     << " " << searchPoint.y 
     << " " << searchPoint.z << ")" 
     << std::endl;
              
    for (std::size_t i = 0; i < pointIdxVec.size (); ++i)
    {
   std::cout << "    " << (*cloud)[pointIdxVec[i]].x 
       << " " << (*cloud)[pointIdxVec[i]].y 
       << " " << (*cloud)[pointIdxVec[i]].z << std::endl;
      ROS_INFO_STREAM("Points inside the voxel: " << " x=" << (*cloud)[pointIdxVec[i]].x << "y=" << (*cloud)[pointIdxVec[i]].y) ;
    }
  }
  */
    
    
    
    ros::Rate loop_rate(4);
    while (nh.ok())
    {
    //pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pcl::PCLPointCloud2 pcl_pc ; 
    pcl::toPCLPointCloud2 (*cloud,pcl_pc) ; 
    sensor_msgs::PointCloud2 pt2;
    pcl_conversions::fromPCL (pcl_pc,pt2);
    pub.publish (pt2);
    
    ros::spinOnce;
    loop_rate.sleep ();
    }
    // Success
    return 0;
  }