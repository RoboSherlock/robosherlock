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
  DistanceMap() : resolution(0.0) {}
  DistanceMap(float res) : resolution(res) {}
  ~DistanceMap() {}

  bool setInputCloud(typename pcl::PointCloud<PointT>::Ptr &cloud){
    if(cloud->points.size() < 3){
      outError("Insufficient points in cloud!");
      return false;
    }

    octree = typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr ( new pcl::octree::OctreePointCloudSearch<PointT>(resolution) );
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();

    return true;
  }

  bool setBoundingPlanes(const std::vector<Eigen::Vector4f>& planes, const std::vector<float>& offset = std::vector<float>(0)){
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
          point += normal * offset[it];
          pointNormalToPlane(point, normal, bounding_planes[it]);
        }
      }
      return true;
  }

  void setResolution(float res){
    resolution = res;
  }

  bool getNearestOccupiedDistance(PointT& point, int& result_index, float& sqr_dist){
    if(!octree){
      outError("Octree was not initialized, maybe you forget to input cloud!");
      return false;
    }

    octree->approxNearestSearch(point, result_index, sqr_dist);
    sqr_dist = std::sqrt(sqr_dist);

    Eigen::Vector3f pointVec = point.getVector3fMap();
    float boundingPlaneDist = getMinDistToBoundingPlane(pointVec);
    boundingPlaneDist = std::max(-boundingPlaneDist, 0.0f);

    if(boundingPlaneDist > sqr_dist){
      sqr_dist = boundingPlaneDist;
      result_index = -1;
    }

    return true;
  }

private:
  float getMinDistToBoundingPlane(Eigen::Vector3f& point){
    float dist = std::numeric_limits<float>::max();

    for(size_t it = 0;  it < bounding_planes.size(); it++)
      dist = std::min(dist, pointToPlaneSignedNorm(point, bounding_planes[it]));
    return dist;
  }

};

#endif
