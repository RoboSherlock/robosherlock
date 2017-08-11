/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __DISTANCE_MAP_HPP__
#define __DISTANCE_MAP_HPP__

#include <rs/segmentation/array_utils.hpp>
#include <rs/segmentation/Geometry.hpp>
#include <rs/utils/output.h>

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

#include <vector>

template<typename PointT>
class DistanceMap
{
private:
  typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr octree;
  float resolution;

  std::vector<Eigen::Vector4f> bounding_planes;
public:
  typedef boost::shared_ptr< DistanceMap<PointT> > Ptr;

  DistanceMap() : resolution(0.0) {}
  DistanceMap(float res) : resolution(res) {}

  ~DistanceMap() {}

  bool setInputCloud(typename pcl::PointCloud<PointT>::Ptr &cloud)
  {
    if(cloud->points.size() < 3)
    {
      outError("Insufficient points in cloud!");
      return false;
    }

    octree = typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr ( new pcl::octree::OctreePointCloudSearch<PointT>(resolution) );
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();

    return true;
  }

  bool setBoundingPlanes(const std::vector<Eigen::Vector4f> &planes, const std::vector<float> &offset = std::vector<float>(0))
  {
    if(offset.size() == 0)
    {
      bounding_planes = planes;
    }
    else if(planes.size() != offset.size())
    {
      outError("Size of input planes does not equal to size of offsets!");
      return false;
    }
    else
    {
      bounding_planes.resize(planes.size());

      for(size_t it = 0; it < planes.size(); it++)
      {
        Eigen::Vector3f point, normal;
        planeToPointNormal<float>(planes[it], point, normal);
        point += normal * offset[it];
        pointNormalToPlane(point, normal, bounding_planes[it]);
      }
    }

    return true;
  }

  void setResolution(float res)
  {
    resolution = res;
  }

  bool getNearestOccupiedDistance(PointT &point, int &result_index, float &sqr_dist)
  {
    if(!octree)
    {
      outError("Octree was not initialized, maybe you forget to input cloud!");
      return false;
    }

    octree->approxNearestSearch(point, result_index, sqr_dist);
    sqr_dist = std::sqrt(sqr_dist);

    //NOTE: it seems that it does not need to consider bounding planes in practical pipeline (because all objects are above bounding plane)
    /*Eigen::Vector3f pointVec = point.getVector3fMap();
    float boundingPlaneDist = getMinDistToBoundingPlane(pointVec);
    boundingPlaneDist = std::max(-boundingPlaneDist, 0.0f);

    if(boundingPlaneDist > sqr_dist){
      sqr_dist = boundingPlaneDist;
      result_index = -1;
    }*/

    return true;
  }

private:
  float getMinDistToBoundingPlane(Eigen::Vector3f &point)
  {
    float dist = std::numeric_limits<float>::max();

    for(size_t it = 0;  it < bounding_planes.size(); it++)
    {
      dist = std::min(dist, pointToPlaneSignedNorm(point, bounding_planes[it]));
    }
    return dist;
  }
};

#endif // __DISTANCE_MAP_HPP__
