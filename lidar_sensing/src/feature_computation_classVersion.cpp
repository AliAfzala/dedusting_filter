#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "lidar_sensing/dedusting.h"
#include <pcl/octree/octree_search.h>
#include <vector>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <eigen3/Eigen/Dense>
#include <pcl/filters/crop_box.h>
#include <eigen3/Eigen/Eigenvalues>
#include <cmath>
#include <chrono>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
  // Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

  // Topics

using namespace std::chrono;
using namespace Eigen;

#include <std_msgs/Float64.h>

class FeatureComputation
{
public:
    std::vector<int> pointIdxVec;
    std::vector<int> voxel_intensity;
    lidar_sensing::dedusting features;
   // lidar_sensing::dedusting *features1 = new lidar_sensing::dedusting;
    geometry_msgs::Point point ;
    double sum ;
    double voxel_intensity_mean ;
    double sq_sum; 
    double voxel_intensity_std; 
    double distance ;
    int sz;
    const std::string IMAGE_TOPIC = "velodyne_points";
    const std::string PUBLISH_TOPIC = "feature_topics";
    FeatureComputation(ros::NodeHandle *nh) 
    {
        // Initialize temperature and ROS publisher
        sub = nh->subscribe(IMAGE_TOPIC,1,&FeatureComputation::readLidarData,this);

        // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
        pub = nh->advertise<lidar_sensing::dedusting>(PUBLISH_TOPIC, 1);
        minX = 0.0 ; minY = -10.0 ;minZ = -3.0 ; maxX = 15.0 ; maxY = 10.0 ; maxZ = 3.0 ;
        resolution = 0.30f;

    }
    void readLidarData(const sensor_msgs::PointCloud2 cloud_msg )
    {
        auto start = high_resolution_clock::now();
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZICropped (new pcl::PointCloud<pcl::PointXYZI>);
        pcl_conversions::toPCL(cloud_msg, *cloud);
        pcl::fromPCLPointCloud2 (*cloud,*cloudXYZI) ;
        cropFilter(cloudXYZI,cloudXYZICropped);
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
        octree.setInputCloud (cloudXYZICropped);
        octree.addPointsFromInputCloud ();
        pcl::octree::OctreePointCloud < pcl::PointXYZI>::AlignedPointTVector searchPoint;
        octree.getOccupiedVoxelCenters( searchPoint);
        features.voxel_number = searchPoint.size();
        for (int j = 0; j < features.voxel_number; ++j)
        {
            if ( (octree.voxelSearch (searchPoint[j], pointIdxVec)) && (pointIdxVec.size() > 3))
            {
                sz = pointIdxVec.size();
                Matrix<double,Dynamic,3> voxel_points;
                voxel_points.conservativeResize(sz,3);
                for (std::size_t i = 0; i < pointIdxVec.size (); ++i)
                {
                    voxel_points (i,0) = (*cloudXYZICropped)[pointIdxVec[i]].x ;
                    voxel_points(i,1) =( (*cloudXYZICropped)[pointIdxVec[i]].y ) ;
                    voxel_points(i,2)= ( (*cloudXYZICropped)[pointIdxVec[i]].z);
                    voxel_intensity.push_back((*cloudXYZI)[pointIdxVec[i]].intensity);
                }

                JacobiSVD<MatrixXd> svd(voxel_points.transpose()*voxel_points);
                MatrixXd D =svd.singularValues();
                sum = std::accumulate(voxel_intensity.begin(), voxel_intensity.end(), 0.0);
                voxel_intensity_mean = sum / voxel_intensity.size();
                sq_sum = std::inner_product(voxel_intensity.begin(), voxel_intensity.end(), voxel_intensity.begin(), 0.0);
                voxel_intensity_std = std::sqrt(sq_sum / voxel_intensity.size() - voxel_intensity_mean *voxel_intensity_mean);
                distance = sqrt(std::pow(searchPoint[j].x,2) + std::pow(searchPoint[j].y,2)) ;
                point.x = searchPoint[j].x;
                point.y = searchPoint[j].y;
                point.z = searchPoint[j].z;
                features.voxel_mean_intensity.push_back(voxel_intensity_mean);
                features.voxel_std_intensity.push_back(voxel_intensity_std);
                features.voxel_eigen3OverEigen1.push_back(D(2)/D(0));
                features.voxel_eigen2OverEigen1.push_back(D(1)/D(0));
                features.voxel_eigen1OverSumEigen.push_back(D(0)/(D(0)+D(1)+D(2)));
                features.voxel_numberOverDis.push_back(sz/distance);
                features.point.push_back(point);
                pointIdxVec.clear();
                voxel_intensity.clear();
                
            }
            
        }
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        ROS_INFO_STREAM( "Time for computing features : " << duration.count());
        //feat = features;
        pub.publish (features);
        reset();
    

    }

    void cropFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI ,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZICropped)
    {
        //double minX = 0; double minY = -10; double minZ = -3;
        //double maxX = 15 ; double maxY = 10 ; double maxZ = 3 ;
        pcl::CropBox<pcl::PointXYZI> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
        boxFilter.setInputCloud(cloudXYZI);
        boxFilter.filter(*cloudXYZICropped);
    }
   
   /*
    void publishFeatures()
    {
        pub.publish(features);
    }
    */


    void reset()
    {
        features.point.clear();
        features.voxel_eigen1OverSumEigen.clear();
        features.voxel_eigen2OverEigen1.clear();
        features.voxel_eigen3OverEigen1.clear();
        features.voxel_mean_intensity.clear();
        features.voxel_std_intensity.clear();
        features.voxel_numberOverDis.clear();
    }
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    double minX , minY ,minZ , maxX , maxY , maxZ  ;
    float resolution;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_node_classVersion");
    ros::NodeHandle nh;
    // Create an instance of Temperature sensor
    FeatureComputation fc = FeatureComputation(&nh);
    // Create a ROS timer for reading data
    // the callback function for the Timer must
    // be bound with std::bind or boost::bind
    
    /*
    ros::Timer timerReadTemperature =
        nh.createTimer(ros::Duration(1.0 / 10.0),
                       std::bind(&FeatureComputation::readLidarData, featureComputation));

                       */
    // Create a ROS timer for publishing temperature
    /*
    ros::Timer timerPublishTemperature =
        nh.createTimer(ros::Duration(1.0 / 20.0),
                   std::bind(&FeatureComputation::publishFeatures, fc));
                   */
    // We can now use spin, or do whatever 
    // we want in this node
    ros::spin();
    return 0;
}