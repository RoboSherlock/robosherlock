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

#ifndef __DOWNSAMPLE_MAP_HPP__
#define __DOWNSAMPLE_MAP_HPP__

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>

#include <rs/utils/output.h>

enum DownsampleMethod
{
  AVERAGE,
  NEAREST_NEIGHBOR
};

/** \brief Data structure inherited from VoxelGrid to downsample the cloud and generate
 *  a mapping from downsampled cloud to original cloud and vice versa.
 *  It also provides a vector of indices to nearest point in original cloud from each downsampled point.
 */
template<typename PointT>
class DownsampleMap : public pcl::VoxelGrid<PointT>
{
private:
  DownsampleMethod downsampleMethod;

  typename pcl::PointCloud<PointT>::Ptr dsCloud;

  /** \brief Store mapping from downsample indices to original indices. */
  std::vector< std::vector<int> > downsampleMap;

  /** \brief Store mapping from original indices to downsample indices. */
  std::vector<int> reversedMap;

  /** \brief Store nearest index of each point for NEAREST NEIGHBOR method. */
  std::vector<int> nearestIndices;

  /** \brief Clear all mappings and downsampled cloud. */
  inline void reset()
  {
    dsCloud.reset(new pcl::PointCloud<PointT>);
    downsampleMap.clear();
    nearestIndices.clear();
  }

  /** \brief Compute mapping from downsample indices to original indices.
   *  \return false if filter function is not run first*/
  inline bool initDownsampleMap()
  {
    if(downsampleMap.empty())
    {
      if(dsCloud->points.empty())
      {
        outError("No downsampled cloud, please run filter first!");
        return false;
      }

      downsampleMap.resize(dsCloud->points.size());
      for(size_t it = 0; it < this->indices_->size(); it++)
      {
        int pointId = this->indices_->at(it);
        int dsPointId;
        try
        {
          dsPointId = this->getCentroidIndex(this->input_->points[pointId]);
        }
        catch(...)
        {
          Eigen::Vector3f coordinate = this->input_->points[pointId].getVector3fMap();
          dsPointId = this->getCentroidIndexAt(this->getGridCoordinates(coordinate[0], coordinate[1], coordinate[2]));
          if(dsPointId == -1)
          {
            continue;
          }
        }

        downsampleMap[dsPointId].push_back(pointId);
      }
    }

    return true;
  }


  /** \brief Compute mapping from original indices to downsample indices. */
  inline void initReversedMap()
  {
    if(reversedMap.empty())
    {
      initDownsampleMap();

      reversedMap.resize(this->indices_->size());
      for(size_t dsPointId = 0; dsPointId < downsampleMap.size(); dsPointId++)
      {
        for(size_t pointIdIt = 0; pointIdIt < downsampleMap[dsPointId].size(); pointIdIt++)
        {
          int pointId = downsampleMap[dsPointId][pointIdIt];
          reversedMap[pointId] = dsPointId;
        }
      }
    }
  }

  /** \brief Compute a vector of indices to nearest point in original cloud from each downsampled point. */
  inline void initNearestPointIndices()
  {
    if(nearestIndices.empty())
    {
      initDownsampleMap();

      pcl::search::KdTree<PointT> search;
      std::vector<int> indices(1);
      std::vector<float> dists(1);

      for(size_t it = 0; it < dsCloud->points.size(); it++)
      {
        search.setInputCloud(this->input_, boost::make_shared< std::vector<int> >(downsampleMap[it]));
        search.nearestKSearch(dsCloud->points[it], 1, indices, dists);
        nearestIndices.push_back(indices[0]);
      }
    }
  }

  /** \brief Apply downsample method to downsample cloud.
   *  \param[out] output  downsampled cloud
   */
  virtual void applyFilter(pcl::PointCloud<PointT>& output)
  {
    if(dsCloud->points.empty())
    {
      pcl::VoxelGrid<PointT>::applyFilter(*dsCloud);
    }

    if(downsampleMethod == AVERAGE)
    {
      output = *dsCloud;
    }
    else if(downsampleMethod == NEAREST_NEIGHBOR)
    {
      initNearestPointIndices();
      pcl::copyPointCloud(*this->input_, nearestIndices, output);
    }
  }

public:

  /** \brief Default Constructor. */
  DownsampleMap() : downsampleMethod(AVERAGE), dsCloud(new pcl::PointCloud<PointT>), downsampleMap(), nearestIndices()
  {
    this->setSaveLeafLayout(true);
  }

  /** \brief Destructor. */
  ~DownsampleMap() {}

  /** \brief Set input cloud.
   *  \param[in]  cloud   original cloud
   */
  virtual void setInputCloud(typename pcl::PointCloud<PointT>::Ptr &cloud)
  {
    pcl::VoxelGrid<PointT>::setInputCloud(cloud);
    this->reset();
  }

  /** \brief Set input indices.
   *  \param[in]  indices   pointer of considered indices of input cloud
   */
  virtual void setIndices(pcl::PointIndices::Ptr &indices)
  {
    pcl::VoxelGrid<PointT>::setIndices(indices);
    this->reset();
  }

  /** \brief Set downsample method.
   *  \param[in]  dsMethod   downsample method
   */
  inline void setDownsampleMethod(DownsampleMethod dsMethod)
  {
    downsampleMethod = dsMethod;
  }

  /** \brief Set resolution of downsample.
   *  \param[in]  leafSize   downsample resolution
   */
  inline void setLeafSize(const float leafSize)
  {
    pcl::VoxelGrid<PointT>::setLeafSize(leafSize, leafSize, leafSize);
    this->reset();
  }

  /** \brief Get downsample map from downsample indices to original indices.
   *  \param[in]  dsMap   a vector of vector of int
   */
  inline void getDownsampleMap(std::vector< std::vector<int> > &dsMap)
  {
    initDownsampleMap();
    dsMap = downsampleMap;
  }

  /** \brief Get downsample map from original indices to downsampled indices.
   *  \param[in]  reversed_map   a vector of int
   */
  inline void getReversedMap(std::vector<int> &reversed_map)
  {
    initReversedMap();
    reversed_map = reversedMap;
  }

  /** \brief Get vector of indices to nearest point in original cloud from each downsampled point.
   *  \param[in]  reversed_map   a vector of int
   */
  inline void getNearestNeighborMap(std::vector< int > &dsNearestMap)
  {
    initNearestPointIndices();
    dsNearestMap = nearestIndices;
  }
};

/** \brief Function to compute downsample normal cloud by AVERAGE or NEAREST_NEIGHBOR method.
 *  \param[in]   in_normals      original normal cloud
 *  \param[in]   dsMap           mapping from downsampled indices to original indices
 *  \param[in]   dsNearestMap    vector of indices to nearest point in original cloud from each downsampled point.
 *  \param[in]   method          downsample method
 *  \param[out]  out_normals     downsampled normal cloud
 *  \return false if dsNearestMap size is not equal to dsMap size
 */
//NOTE: this normal downsample method can use AVERAGE or NEAREST_NEIGHBOR method
inline bool computeDownsampleNormals(pcl::PointCloud<pcl::Normal>::Ptr &in_normals,
                                     std::vector< std::vector<int> > &dsMap,
                                     std::vector<int> &dsNearestMap,
                                     DownsampleMethod method,
                                     pcl::PointCloud<pcl::Normal>::Ptr &out_normals)
{
  if(dsNearestMap.size() != dsMap.size())
  {
    outError("Inconsistent downsample map and nearest neighbors map!");
    return false;
  }
  out_normals.reset(new pcl::PointCloud<pcl::Normal>(dsMap.size(), 1));
  out_normals->points.resize(dsMap.size());

  if(method == AVERAGE)
  {
    for(size_t dsPointId = 0; dsPointId < dsMap.size(); dsPointId++)
    {
      int denom = dsMap[dsPointId].size();
      float dsNormX = 0.0f, dsNormY = 0.0f, dsNormZ = 0.0f;

      for(size_t pointIdIt = 0; pointIdIt < dsMap[dsPointId].size(); pointIdIt++)
      {
        int pointId = dsMap[dsPointId][pointIdIt];
        if(pcl_isfinite(in_normals->points[pointId].normal_x) &&
           pcl_isfinite(in_normals->points[pointId].normal_y) &&
           pcl_isfinite(in_normals->points[pointId].normal_z))
        {
          dsNormX += in_normals->points[pointId].normal_x;
          dsNormY += in_normals->points[pointId].normal_y;
          dsNormZ += in_normals->points[pointId].normal_z;
        }
        else
        {
          denom--;
        }
      }

      if(denom != 0)
      {
        dsNormX /= denom;
        dsNormY /= denom;
        dsNormZ /= denom;
        out_normals->points[dsPointId] = pcl::Normal(dsNormX, dsNormY, dsNormZ);
      }
      else
      {
        outWarn("Normal cloud is corrupted! Too many NaN normals");
      }
    }
  }
  else if(method == NEAREST_NEIGHBOR)
  {
    for(size_t dsPointId = 0; dsPointId < dsNearestMap.size(); dsPointId++)
    {
      int pointId = dsNearestMap[dsPointId];
      out_normals->points[dsPointId] = in_normals->points[pointId];
    }
  }

  return true;
}


/** \brief Function to upsample from downsampled cloud to original cloud.
 *  \param[in]  dsIndices   downsampled indices
 *  \param[in]  dsMap       mapping from downsampled indices to original indices
 *  \param[out] usIndices   upsampled indices
 *  \return false if downsampled map is empty
 */
inline bool upsample_cloud(const std::vector<int> &dsIndices,
                           const std::vector< std::vector<int> > &dsMap,
                           std::vector<int> &usIndices)
{
  if(dsMap.empty())
  {
    outError("Downsample map is null!");
    return false;
  }

  for(size_t it = 0; it < dsIndices.size(); it++)
  {
    usIndices.insert(usIndices.end(), dsMap[dsIndices[it]].begin(), dsMap[dsIndices[it]].end());
  }

  return true;
}

#endif // __DOWNSAMPLE_MAP_HPP__
