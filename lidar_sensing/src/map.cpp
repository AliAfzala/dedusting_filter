#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/PCLPointCloud2.h>


ros::Publisher pub;


void convert2PCL (const sensor_msgs::PointCloud2ConstPtr& cloud_msg ) {

// Container for original & filtered data
//pcl::PointCloud<pcl::PCLPointCloud2 >* msg (new pcl::PointCloud<pcl::PCLPointCloud2 >);
pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
//pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//pcl::PCLPointCloud2 cloud_filtered;





// Convert to PCL data type
pcl_conversions::toPCL(*cloud_msg, *cloud);

 // Perform the actual filtering
  /*pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

*/
  //sensor_msgs::PointCloud2 output;
  //pcl_conversions::fromPCL(cloud_filtered, output);

pub.publish(*cloud);
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"pcl_node");
    ros::NodeHandle n ;
    ros::Publisher pub = n.advertise < pcl::PointCloud<pcl::PCLPointCloud2 >> ("pcl_topic",1);
    //ros::Rate loop_rate(0.5)
    ros::Subscriber sub = n.subscribe ("velodyne_points",1,convert2PCL);
    

    ros::spin();
    return 0;

}








