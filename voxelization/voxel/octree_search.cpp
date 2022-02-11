#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <iostream>
#include <vector>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include"eigen3/Eigen/Dense"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>

int main (int argc, char** argv)
  {

  //pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile <pcl::PointXYZI> ("dust-5m-4m-Frame77.pcd", *cloud1); 


  double minX = -10; double minY = 0; double minZ = -3;
  double maxX = 20 ; double maxY = 20 ; double maxZ = 3 ;
  pcl::CropBox<pcl::PointXYZI> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
  boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
  boxFilter.setInputCloud(cloud1);
  boxFilter.filter(*cloud);
    // For conversion of pcl::pointcloud2 to pcl::pointXYZI refer to following website: 
    //https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt

    /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }
  */

  float resolution = 0.25f;
  //pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ> octree (resolution);
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
  //octree.defineBoundingBox (-10.0 , 0.0 ,  -3.0 ,  20.0 ,  20.0 ,  3.0);
  //pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ> octree (resolution);

  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();

  //octree.setInputCloud (cloud);
  //octree.addPointsFromInputCloud ();
  pcl::octree::OctreePointCloud < pcl::PointXYZI>::AlignedPointTVector searchPoint;
  octree.getOccupiedVoxelCenters( searchPoint);
  
  /*std::cout<< voxel_center_list[0] << std::endl;
  std::cout<< voxel_center_list[1] << std::endl;
  std::cout << voxel_center_list.size()<< std::endl;
  */

  /*pcl::PointXYZ searchPoint;

  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  std::vector<int> pointIdxVec;
  */
 
 std::vector<int> pointIdxVec;
 //int sz = searchPoint.size();
  int ii = 1;
  //for (std::size_t j = 0; j < searchPoint.size (); ++j)
  for (int j = 0; j < 1; ++j)
  {
    if (octree.voxelSearch (searchPoint[j], pointIdxVec))
      {
        std::cout << "Neighbors within voxel search at (" << searchPoint[j].x 
        << " " << searchPoint[j].y 
        << " " << searchPoint[j].z << ")" 
        << std::endl;
        std::cout<< " iteration over voxel number:"<< ii<<std::endl;
              
          for (std::size_t i = 0; i < pointIdxVec.size (); ++i)
            {
            std::cout << "    " << (*cloud)[pointIdxVec[i]].x 
            << " " << (*cloud)[pointIdxVec[i]].y 
            << " " << (*cloud)[pointIdxVec[i]].z << std::endl;

            }
        }
      ii++;
  }
  std::cout<< " the size of voxel center vector is  " << searchPoint.size()<< std::endl;
  std::cout<< " the voxel resolution is " << octree.getResolution() << std::endl;
    // Success
    return 0;
  }

 // Eigen::Matrix3f covariance_matrix;