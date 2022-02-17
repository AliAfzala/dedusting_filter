#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

using namespace Eigen;

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
 std::vector<int> voxel_intensity;
 //int sz = searchPoint.size();
  int ii = 1;
  //for (std::size_t j = 0; j < searchPoint.size (); ++j)
  for (int j = 0; j < 100; ++j)
  {
    if ( (octree.voxelSearch (searchPoint[j], pointIdxVec)) && (pointIdxVec.size() > 3))
      {
        
        int sz = searchPoint.size();
        Matrix<double,Dynamic,3> voxel_points;
        //voxel_points.resize(sz,3);  
        //VectorXd voxel_intensity ; 
        //Matrix<float,Dynamic,1> voxel_intensity;
        //voxel_intensity.resize(sz,1);

        
        for (std::size_t i = 0; i < pointIdxVec.size (); ++i)
            {
            voxel_points.conservativeResize(i+1,3);
            //voxel_intensity.conservativeResize(i+1,1); 
            //std::cout<<i<< std::endl;
            voxel_points (i,0) = (*cloud)[pointIdxVec[i]].x ; 
            //std::cout<< " Hello World!" << std::endl;
            voxel_points(i,1) =( (*cloud)[pointIdxVec[i]].y ) ;
            voxel_points(i,2)= ( (*cloud)[pointIdxVec[i]].z);
            //std::cout<< " Hello World!" << std::endl;
            //voxel_intensity (i,0) = (*cloud)[pointIdxVec[i]].intensity;
            voxel_intensity.push_back((*cloud)[pointIdxVec[i]].intensity);
           
            } 
            /*
            std::cout << "start" << std::endl;
            std::cout << voxel_points << std::endl;
            std::cout << "end" << std::endl;
            */


        
        EigenSolver< Matrix<double,Dynamic,3> > es(voxel_points.transpose()*voxel_points);      
        MatrixXd D = es.pseudoEigenvalueMatrix();
        MatrixXd V = es.pseudoEigenvectors();  

        double voxel_roughness = std::abs(D(2,2)); 
        double voxel_slope = std::sin( V(2,2)/ V.col(2).norm() );
        std::cout << " Voxel number " << ii << std::endl; 
        std::cout << " Voxel roughness is:  " << voxel_roughness << std::endl ;  
        std::cout << " Voxel slope is:  "<< voxel_slope << std::endl ;
        //std::cout << " EigenValueMatrix starts"<< std::endl;
        //std::cout << D << std::endl;
        //std::cout << V << std::endl;

        
         /* for (std::size_t i = 0; i < pointIdxVec.size (); ++i)
            {
            std::cout << "    " << (*cloud)[pointIdxVec[i]].x 
            << " " << (*cloud)[pointIdxVec[i]].y 
            << " " << (*cloud)[pointIdxVec[i]].z << std::endl;

            } */
          double sum = std::accumulate(voxel_intensity.begin(), voxel_intensity.end(), 0.0);
          double voxel_intensity_mean = sum / voxel_intensity.size();
          double sq_sum = std::inner_product(voxel_intensity.begin(), voxel_intensity.end(), voxel_intensity.begin(), 0.0);
          double voxel_intensity_std = std::sqrt(sq_sum / voxel_intensity.size() - voxel_intensity_mean *voxel_intensity_mean);
          std::cout << " Voxel mean intensity is:  " << voxel_intensity_mean << std::endl ;
          std::cout << " Voxel std intensity is:  " << voxel_intensity_std << std::endl ;
          pointIdxVec.clear();
        }
      ii++;
      
      
  }
  //std::cout<< " the size of voxel center vector is  " << searchPoint.size()<< std::endl;
  //std::cout<< " the size of voxel having point more than 3 " <<  ii << std::endl;
  //std::cout<< " the size of voxel having point more than 3 " << octree.getResolution() << std::endl;
    // Success
    return 0;
  }

 // Eigen::Matrix3f covariance_matrix;