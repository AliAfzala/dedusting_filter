#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <iostream>
#include <vector>
#include <pcl/octree/octree_pointcloud_pointvector.h>

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

  float resolution = 500.0f;
  pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ> octree (resolution);
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

    }
  }
  pcl::octree::OctreePointCloudPointVector<	

    // Success
    return 0;
  }