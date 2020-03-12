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

#include <vector>
#include <mutex>

#include <omp.h>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

//RS
#include <robosherlock/utils/output.h>
#include <robosherlock/types/all_types.h>

//graph
#include <robosherlock/graph/Graph.h>
#include <robosherlock/graph/GraphAlgorithms.hpp>

#include <robosherlock/utils/array_utils.hpp>

class OverSegmenter
{
private:
  std::vector< pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> > rg;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  std::vector<int> cloudIds; // Ids used to input to region growing

  std::vector<pcl::PointIndices> linear_segments;

  float minNormalThreshold;
  float maxNormalThreshold;
  float curvatureThreshold;

  float overlapThreshold;

  int minClusterSize;
  int maxClusterSize;

  int neighborNumber;

  int numSegmentation;

  bool isSetup;
public:
  OverSegmenter ();

  ~OverSegmenter();

  void initialize(float minNormalThreshold,
                  float maxNormalThreshold,
                  float curvatureThreshold,
                  float overlapThreshold,
                  int minClusterSize,
                  int maxClusterSize,
                  int neighborNumber,
                  int numSegmentation);

  void setInputClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr normals);

  void removeSegments(std::vector<pcl::PointIndices> &segments);

  void removeSegments(std::vector< std::vector<int> > &segments);

  void resetCloudIds();

  bool segment();

  bool getSegments(std::vector<pcl::PointIndices> &segments);

  bool getColoredCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &coloredCloud, int index);

protected:

  void refineSegments(std::vector< std::vector<pcl::PointIndices> > &segmentations);
};
