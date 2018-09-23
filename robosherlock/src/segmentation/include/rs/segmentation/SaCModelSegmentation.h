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

#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


namespace rs
{
namespace rs_pcl
{
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class SacModelSegmentation
{
public:

  class Config
  {
  public:
    bool useNormals;
    bool downsample;

    int sacModel;
    int minPoints;

    float minOpeningAngle;
    float maxOpeningAngle;
    float radiusLimit1;
    float radiusLimit2;
    float distanceWeight;
    float distanceThreshold;
    float downsampleLeafSize;
  };


  std::vector< pcl::PointIndices> modelsIndices;
  Config config;

  SacModelSegmentation()
  {

  }

  void configure(const Config &config);

  /**
   * \brief this method detect the shapes of multiple point clouds
   * \param inputCluster cluster of point clouds
   * \param cloud_normals the normals of the inputCloud
   */
  bool detect(PointCloudT::Ptr inputCloud,
              pcl::PointCloud< pcl::Normal>::Ptr cloud_normals);

private:

  pcl::SACSegmentationFromNormals< PointT, pcl::Normal> segNormal;
  pcl::SACSegmentation< PointT> seg;
  // Datasets
  pcl::ModelCoefficients::Ptr coefficients;
  pcl::ExtractIndices< PointT> extractIndices;
  pcl::ExtractIndices< pcl::Normal> extractIndicesFromNormals;

  /**
   * \brief calculate the normals
   * \param cloud point cloud
   * \param cloud_normals resulting normals
   */
  bool estimateNormals(const PointCloudT::Ptr &cloud,
                       pcl::PointCloud< pcl::Normal>::Ptr &cloud_normals);

};
}
}

