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

#include <uima/api.hpp>
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

//RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/types/all_types.h>

//graph
#include <rs/graph/GraphBase.hpp>
#include <rs/graph/Graph.hpp>
#include <rs/graph/GraphAlgorithms.hpp>

#include <rs/segmentation/array_utils.hpp>

using namespace uima;

class OverSegmentationAnnotator : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  std::vector< pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> > rg;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  std::vector< std::vector<int> > dsMap;

  bool object_pass_through;

  float minNormalThreshold;
  float maxNormalThreshold;
  float curvatureThreshold;

  float overlapThreshold;

  int minClusterSize;
  int maxClusterSize;

  int neighborNumber;

  int numSegmentation;

  int choose;

  double pointSize;

public:
  OverSegmentationAnnotator () : DrawingAnnotator(__func__), pointSize(1.0) {}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");


    if(ctx.isParameterDefined("object_pass_through"))
    {
      ctx.extractValue("object_pass_through", object_pass_through);
    }
    if(ctx.isParameterDefined("minNormalThreshold"))
    {
      ctx.extractValue("minNormalThreshold", minNormalThreshold);
    }
    if(ctx.isParameterDefined("maxNormalThreshold"))
    {
      ctx.extractValue("maxNormalThreshold", maxNormalThreshold);
    }
    if(ctx.isParameterDefined("overlapThreshold"))
    {
      ctx.extractValue("overlapThreshold", overlapThreshold);
    }
    if(ctx.isParameterDefined("curvatureThreshold"))
    {
      ctx.extractValue("curvatureThreshold", curvatureThreshold);
    }
    if(ctx.isParameterDefined("minClusterSize"))
    {
      ctx.extractValue("minClusterSize", minClusterSize);
    }
    if(ctx.isParameterDefined("maxClusterSize"))
    {
      ctx.extractValue("maxClusterSize", maxClusterSize);
    }
    if(ctx.isParameterDefined("neighborNumber"))
    {
      ctx.extractValue("neighborNumber", neighborNumber);
    }
    if(ctx.isParameterDefined("numSegmentation"))
    {
      ctx.extractValue("numSegmentation", numSegmentation);
    }

    rg.resize(numSegmentation);

    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    choose = 0;
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;

    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    //check routine to use scene cloud or object cloud
    bool isObjectPassed = false;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr (new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_CLOUD_OBJECTS, *cloud_ptr);
    if(cloud_ptr->size() == 0)
    {
      outInfo("Input Object cloud address is empty! Using scene cloud");
      cas.get(VIEW_CLOUD, *cloud_ptr);
      cas.get(VIEW_NORMALS, *normals_ptr);
    }
    else
    {
      //get normal cloud
      cas.get(VIEW_NORMALS_OBJECTS, *normals_ptr);
      isObjectPassed = true;
    }

    //if cloud is passed, no need to segment planes from cloud
    if(isObjectPassed)
    {
      cloud = cloud_ptr;
      normals = normals_ptr;
    }
    else
    {
      //get plane indices if it has
      std::vector<int> object_indices;
      std::vector<int> plane_indices;
      std::vector<int> cloudIds(cloud_ptr->size());
      for(size_t pointId = 0; pointId < cloud_ptr->size(); pointId++)
      {
        cloudIds[pointId] = pointId;
      }

      std::vector<rs::Plane> planes;
      scene.annotations.filter(planes);
      for(size_t planeId = 0; planeId < planes.size(); planeId++)
      {
        std::vector<int> currIds = planes[planeId].inliers();
        plane_indices.insert(plane_indices.end(), currIds.begin(), currIds.end());
      }

      if(plane_indices.size() != 0)
      {
        object_indices = Difference(cloudIds, plane_indices);
      }
      else
      {
        object_indices = cloudIds;
      }

      //filter object cloud
      pcl::copyPointCloud(*cloud_ptr, object_indices, *cloud);
      pcl::copyPointCloud(*normals_ptr, object_indices, *normals);
    }

    outInfo("Object Cloud size: " << cloud->size());
    outInfo("Object Normals size: " << normals->size());

    //for combining the two segmentation method purpose
    std::vector<int> cloudObjectIds(cloud->size());
    for(size_t pointId = 0; pointId < cloud->size(); pointId++)
    {
      cloudObjectIds[pointId] = pointId;
    }
    std::vector<int> remainedIds;
    std::vector<int> segmentIds;
    std::vector<pcl::PointIndices> rotational_segments;
    std::vector<pcl::PointIndices> bilateral_segments;
    cas.get(VIEW_ROTATIONAL_SEGMENTATION_IDS, rotational_segments);
    cas.get(VIEW_BILATERAL_SEGMENTATION_IDS, bilateral_segments);
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
      remainedIds = Difference(cloudObjectIds, segmentIds);
    }
    else
    {
      remainedIds = cloudObjectIds;
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
    std::vector<pcl::PointIndices> linear_segments;

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
      rg[i].setIndices(boost::make_shared< std::vector<int> >(remainedIds));
      rg[i].extract(segmentations[i]);
    }

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

    //for visualization purpose
    outInfo("Choosing segment cloud num: " << choose);
    colored_cloud = rg[choose].getColoredCloud();

    //publish clusters to CAS
    cas.set(VIEW_SEGMENT_IDS, linear_segments);

    //publish cloud that only has objects
    cas.set(VIEW_CLOUD_OBJECTS, *cloud);
    cas.set(VIEW_NORMALS_OBJECTS, *normals);

    return UIMA_ERR_NONE;
  }

  bool callbackKey(const int key, const Source source)
  {
    choose = (int) key - 48;
    if(choose > numSegmentation - 1)
    {
      choose = numSegmentation - 1;
    }
    return true;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    const std::string normalsname = this->name + "_normals";

    if(firstRun)
    {
      visualizer.addPointCloud(colored_cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(colored_cloud, normals, 50, 0.02f, normalsname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, normalsname);
    }
    else
    {
      visualizer.updatePointCloud(colored_cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removePointCloud(normalsname);
      visualizer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(colored_cloud, normals, 50, 0.02f, normalsname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, normalsname);
    }
  }

  void drawImageWithLock(cv::Mat& disp) {}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(OverSegmentationAnnotator)
