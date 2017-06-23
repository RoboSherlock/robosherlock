#ifndef OCCUPANCY_MAP_PCL_HPP
#define OCCUPANCY_MAP_PCL_HPP

#include <rs/segmentation/array_utils.hpp>
#include <rs/segmentation/Geometry.hpp>
#include <rs/utils/output.h>

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

#include <vector>

template<typename PointT>
class OccupancyMapPCL{
private:
  typename pcl::octree::OctreePointCloudSearch<PointT> octree;
  float resolution;

  std::vector<Eigen::Vector4f> bounding_planes;
public:
  std::vector<int> kNearestPointId;
  std::vector<float> nearestDistMap;

  OccupancyMapPCL() : resolution(0.0) {}
  ~OccupancyMapPCL() {}

  bool setInputCloud(pcl::PointCloud<PointT>::Ptr &cloud){
    if(cloud->points.size() < 3){
      outError("Insufficient points in cloud!");
      return false;
    }
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    return true;
  }

  void setBoundingPlanes(std::vector<Eigen::Vector4f>& planes){

  }

  void setResolution(){
    
  }

  void getNearestOccupiedDistance(){

  }


};

#endif
