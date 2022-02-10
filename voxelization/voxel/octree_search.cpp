#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <iostream>
#include <vector>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include"eigen3/Eigen/Dense"

int main (int argc, char** argv)
  {

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
  }

  float resolution = 800.0f;
  //pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ> octree (resolution);
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree1 (resolution);
  //pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ> octree (resolution);

   octree1.setInputCloud (cloud);
  octree1.addPointsFromInputCloud ();

  //octree.setInputCloud (cloud);
  //octree.addPointsFromInputCloud ();
  pcl::octree::OctreePointCloud < pcl::PointXYZ>::AlignedPointTVector searchPoint;
  octree1.getOccupiedVoxelCenters( searchPoint);
  
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
 std::cout<<searchPoint.size();
 std::vector<int> pointIdxVec;
 //int sz = searchPoint.size();
  int ii = 1;
  for (std::size_t j = 0; j < searchPoint.size (); ++j)
  {
    if (octree1.voxelSearch (searchPoint[j], pointIdxVec))
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
  std::cout<< searchPoint.size() <<std::endl;
    // Success
    return 0;
  }

  Eigen::Matrix3f covariance_matrix;