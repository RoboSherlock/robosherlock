#ifndef DISTANCE_MAP_HPP
#define DISTANCE_MAP_HPP

#include <rs/segmentation/array_utils.hpp>
#include <rs/segmentation/Geometry.hpp>
#include <rs/utils/output.h>

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

#include <vector>

template<typename PointT>
class DistanceMap{
private:
  typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr octree;
  float resolution;

  std::vector<Eigen::Vector4f> bounding_planes;
public:
  std::vector<int> kNearestPointId;
  std::vector<float> nearestDistMap;

  DistanceMap() : resolution(0.0) {}
  DistanceMap(float res) : resolution(res) {}
  ~DistanceMap() {}

  bool setInputCloud(pcl::PointCloud<PointT>::Ptr &cloud){
    if(cloud->points.size() < 3){
      outError("Insufficient points in cloud!");
      return false;
    }

    octree = new pcl::octree::OctreePointCloudSearch<PointT>(resolution);
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();

    return true;
  }

  bool setBoundingPlanes(std::vector<Eigen::Vector4f>& planes, std::vector<float>& offset = std::vector<float>(0)){
      if(offset.size() == 0){
        bounding_planes = planes;
      }
      else if(planes.size() != offset.size()){
        outError("Size of input planes does not equal to size of offsets!");
        return false;
      }
      else{
        bounding_planes.resize(planes.size());

        for(size_t it = 0; it < planes.size(); it++){
          Eigen::Vector3f point, normal;
          planeToPointNormal(planes[it], point, normal);
          point += normal * offsets[it];
          pointNormalToPlane(point, normal, bounding_planes[it]);
        }
      }
  }

  void setResolution(float res){
    resolution = res;
  }

  bool getNearestOccupiedDistance(PointT& point, int& result_index, float& sqr_dist){
    if(!octree){
      outError("Octree was not initialized, maybe you forget to input cloud!");
      return false;
    }

    float nearestPointDist;
    int nearestPointId;
    approxNearestSearch(point, nearestPointId, nearestPointDist);

    float boundingPlaneDist = getMinDistToBoundingPlane(point.getVector3fMap());
    boundingPlaneDist = std::max(-boundingPlaneDist, 0.0);

    return std::max(nearestPointDist, boundingPlaneDist);
  }

private:
  float getMinDistToBoundingPlane(Eigen::Vector3f& point){
    float dist = std::numeric<float>::max();

    for(size_t it = 0;  it < bounding_planes.size(); it++)
      dist = std::min(dist, pointToPlaneSignedNorm(point, bounding_planes[it]));
    return dist;
  }

};

#endif
