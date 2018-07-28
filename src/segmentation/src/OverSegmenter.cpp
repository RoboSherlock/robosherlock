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

 #include <rs/segmentation/OverSegmenter.h>

 OverSegmenter::OverSegmenter()
 {
   this->cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
   this->normals.reset(new pcl::PointCloud<pcl::Normal>());

   this->isSetup = false;
 }

 OverSegmenter::~OverSegmenter() {}

 void OverSegmenter::initialize(float minNormalThreshold,
                                float maxNormalThreshold,
                                float curvatureThreshold,
                                float overlapThreshold,
                                int minClusterSize,
                                int maxClusterSize,
                                int neighborNumber,
                                int numSegmentation)
{
  this->minNormalThreshold = minNormalThreshold;
  this->maxNormalThreshold = maxNormalThreshold;
  this->curvatureThreshold = curvatureThreshold;
  this->overlapThreshold = overlapThreshold;

  this->minClusterSize = minClusterSize;
  this->maxClusterSize = maxClusterSize;

  this->neighborNumber = neighborNumber;
  this->numSegmentation = numSegmentation;

  this->isSetup = true;

  rg.resize(numSegmentation);
}

void OverSegmenter::setInputClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                                   pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  this->cloud = cloud;
  this->normals = normals;
}

bool OverSegmenter::removePlanes(std::vector<rs::Plane> &planes)
{
  if(this->cloud->empty() || this->normals->empty())
  {
    outError("Cloud or Normal cloud is empty!");
    return false;
  }

  if(planes.empty())
  {
    outInfo("There are no plane! This will not remove anything!");
  }

  //get plane indices if it has
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  for(size_t planeId = 0; planeId < planes.size(); planeId++)
  {
    std::vector<int> currIds = planes[planeId].inliers();
    inliers->indices.insert(inliers->indices.end(), currIds.begin(), currIds.end());
  }

  pcl::ExtractIndices<pcl::PointXYZRGBA> extractCloud;
  pcl::ExtractIndices<pcl::Normal> extractNormal;

  extractCloud.setInputCloud(this->cloud);
  extractNormal.setInputCloud(this->normals);

  extractCloud.setIndices(inliers);
  extractNormal.setIndices(inliers);

  extractCloud.setNegative(true);
  extractNormal.setNegative(true);

  extractCloud.filter(*this->cloud);
  extractNormal.filter(*this->normals);

  outInfo("Object cloud size: " << this->cloud->size());
  outInfo("Object normal size: " << this->normals->size());

  return true;
}

void OverSegmenter::removeSegments(std::vector<pcl::PointIndices> &rotational_segments,
                                   std::vector<pcl::PointIndices> &bilateral_segments)
{
  std::vector<int> cloudObjectIds(this->cloud->size());
  std::vector<int> segmentIds;
  for(size_t pointId = 0; pointId < this->cloud->size(); pointId++)
  {
    cloudObjectIds[pointId] = pointId;
  }

  for(size_t segmentId = 0; segmentId < rotational_segments.size(); segmentId++)
  {
    segmentIds.insert(segmentIds.end(), rotational_segments[segmentId].indices.begin(), rotational_segments[segmentId].indices.end());
  }

  for(size_t segmentId = 0; segmentId < bilateral_segments.size(); segmentId++)
  {
    segmentIds.insert(segmentIds.end(), bilateral_segments[segmentId].indices.begin(), bilateral_segments[segmentId].indices.end());
  }

  if(!segmentIds.empty())
  {
    cloudIds = Difference(cloudObjectIds, segmentIds);
  }
  else
  {
    outInfo("There are no previous segments! Using whole ids of object clouds");
    cloudIds = cloudObjectIds;
  }
}

bool OverSegmenter::segment()
{
  if(cloudIds.empty())
  {
    outError("CloudIds for oversegmenting is empty! Did you run removeSegments yet?");
    return false;
  }

  if(!this->isSetup)
  {
    outError("Parameters are not set! Abort segmenting.");
    return false;
  }

  // populate normal Threshold
  std::vector<float> normalThresholds;
  if(numSegmentation == 1)
  {
    normalThresholds.push_back(minNormalThreshold);
  }
  else
  {
    for(size_t i = 0 ; i < numSegmentation; i++)
    {
      normalThresholds.push_back(i * (maxNormalThreshold - minNormalThreshold) / (numSegmentation - 1) + minNormalThreshold);
    }
  }

  //container for segmentation Results
  std::vector< std::vector<pcl::PointIndices> > segmentations(numSegmentation);

  //main execution
  #pragma omp parallel for
  for(size_t i = 0; i < numSegmentation; i++)
  {
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>());

    rg[i].setMinClusterSize(minClusterSize);
    rg[i].setMaxClusterSize(maxClusterSize);
    rg[i].setNumberOfNeighbours(neighborNumber);
    rg[i].setSmoothnessThreshold(normalThresholds[i]);
    rg[i].setCurvatureThreshold(curvatureThreshold);
    rg[i].setSearchMethod(tree);
    rg[i].setInputCloud(cloud);
    rg[i].setInputNormals(normals);
    rg[i].setIndices(boost::make_shared< std::vector<int> >(this->cloudIds));
    rg[i].extract(segmentations[i]);
  }

  this->refineSegments(segmentations);

  return true;
}

void OverSegmenter::refineSegments(std::vector< std::vector<pcl::PointIndices> > &segmentations)
{
  linear_segments.clear();
  // Merge similar segments from multiple segmentations and concat them to linear array
  int numSegments = matrixToLinear(segmentations, segmentations.size(), 0);
  outInfo("Total segments = " << numSegments);

  Graph segmentGraph(numSegments);
  std::mutex graph_mutex;

  for(size_t srcSegmentationIt = 0; srcSegmentationIt < numSegmentation - 1; srcSegmentationIt++)
  {
    std::vector<pcl::PointIndices> src_segmentation = segmentations[srcSegmentationIt];
    for(size_t tgtSegmentationIt = srcSegmentationIt + 1; tgtSegmentationIt < numSegmentation; tgtSegmentationIt++)
    {
      std::vector<pcl::PointIndices> tgt_segmentation = segmentations[tgtSegmentationIt];

      #pragma omp parallel for
      for(size_t srcSegmentIt = 0; srcSegmentIt < src_segmentation.size(); srcSegmentIt++)
      {
        for(size_t tgtSegmentIt = 0; tgtSegmentIt < tgt_segmentation.size(); tgtSegmentIt++)
        {
          int intersectSize = Intersection(src_segmentation[srcSegmentIt].indices, tgt_segmentation[tgtSegmentIt].indices).size();
          int unionSize = Union(src_segmentation[srcSegmentIt].indices, tgt_segmentation[tgtSegmentIt].indices).size();
          float ratio = (float) intersectSize / unionSize;

          if(ratio > overlapThreshold)
          {
            int linearSrcSegmentSub = matrixToLinear(segmentations, srcSegmentationIt, srcSegmentIt);
            int linearTgtSegmentSub = matrixToLinear(segmentations, tgtSegmentationIt, tgtSegmentIt);

            graph_mutex.lock();
            segmentGraph.addEdge(linearSrcSegmentSub, linearTgtSegmentSub);
            graph_mutex.unlock();
          }
        }
      }
    }
  }

  std::vector< std::vector<int> > mergedSegmentIds = extractConnectedComponents(segmentGraph);

  outInfo("Total segments after merging " << mergedSegmentIds.size() << " segments");
  linear_segments.resize(mergedSegmentIds.size());
  for(size_t ccIt = 0; ccIt < mergedSegmentIds.size(); ccIt++)
  {
    int maxSize = -1;
    int selectSegmentationIt = -1;
    int selectSegmentIt = -1;
    for(size_t linSegmentIdIt = 0; linSegmentIdIt < mergedSegmentIds[ccIt].size(); linSegmentIdIt++)
    {
      int segmentationIt, segmentIt;
      int linear_id = mergedSegmentIds[ccIt][linSegmentIdIt];
      linearToMatrix(segmentations, linear_id, segmentationIt, segmentIt);

      int currSegmentSize = segmentations[segmentationIt][segmentIt].indices.size();
      if(currSegmentSize > maxSize)
      {
        maxSize = currSegmentSize;
        selectSegmentationIt = segmentationIt;
        selectSegmentIt = segmentIt;
      }
    }

    linear_segments[ccIt] = segmentations[selectSegmentationIt][selectSegmentIt];
  }
}

bool OverSegmenter::getSegments(std::vector<pcl::PointIndices> &segments)
{
  if(linear_segments.empty())
  {
    outWarn("Result oversegments are not available! Did you run segment yet?");
    return false;
  }

  segments = linear_segments;

  return true;
}

bool OverSegmenter::getColoredCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &coloredCloud, int index)
{
  if(linear_segments.empty())
  {
    outWarn("Result oversegments are not available! Did you run segment yet?");
    return false;
  }

  if(index < 0 || index >= rg.size())
  {
    outWarn("Index of oversegment is invalid!");
    return false;
  }

  coloredCloud = rg[index].getColoredCloud();

  return true;
}
