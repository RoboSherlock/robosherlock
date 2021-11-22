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

#include <robosherlock/utils/array_utils.hpp>
#include <robosherlock/symmetrysegmentation/Geometry.hpp>
#include <robosherlock/utils/output.h>

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

#include <vector>

/** \brief Data structure to compute nearest occlusion distance from a specified point to
 *  other points. If the point is outside bounding planes, the distance is
 *  closest to one of bounding planes, in case this distance is larger than closest distance to other point.
 */
template<typename PointT>
class DistanceMap
{
private:

  /** \brief search tree. */
  typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr octree;
  float resolution;

  /** \brief Bounding planes. */
  std::vector<Eigen::Vector4f> bounding_planes;
public:
  typedef boost::shared_ptr< DistanceMap<PointT> > Ptr;

  /** \brief Empty constructor. */
  DistanceMap() : resolution(0.0) {}

  /** \brief Constructor to set resolution for OcTree. */
  DistanceMap(float res) : resolution(res) {}

  /** \brief destructor. */
  ~DistanceMap() {}

  /** \brief Set input cloud for search tree
   *  \param[in] cloud  Point cloud
   *  \return false if cloud size if lower than 3*/
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

  /** \brief Set bounding planes that cover the whole specified cloud
   *  \param[in] planes    a vector of plane coefficients
   *  \param[in] offsets   a vector of factor to translate planes along their normals
   *  \return false if size of offsets and planes are not equal*/
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

  /** \brief Set resolution for search tree
   *  \param[in] resolution
   */
  void setResolution(float res)
  {
    resolution = res;
  }

  /** \brief Get nearest occlusion distance from specified point
   *  \param[in]  point         specified point
   *  \param[out] result_index  index of the nearest point, equal to -1 if it is a plane
   *  \param[out] sqr_dist      nearest occlusion distance
   *  \return false if input cloud is not set yet
   */
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

  /** \brief Get nearest occlusion distance from specified point to bounding planes
   *  \param[in]  point         specified point
   *  \return nearest occlusion distance to planes
   */
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
