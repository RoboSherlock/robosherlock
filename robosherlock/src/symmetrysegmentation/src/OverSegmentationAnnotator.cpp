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

//RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/types/all_types.h>

#include <rs/symmetrysegmentation/OverSegmenter.h>

//graph
#include <rs/graph/Graph.h>
#include <rs/graph/GraphAlgorithms.hpp>

#include <rs/utils/array_utils.hpp>

using namespace uima;

class OverSegmentationAnnotator : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  std::vector<pcl::PointIndices> linear_segments;

  float minNormalThreshold;
  float maxNormalThreshold;
  float curvatureThreshold;

  float overlapThreshold;

  int minClusterSize;
  int maxClusterSize;

  int neighborNumber;

  int numSegmentation;

  OverSegmenter segmenter;

  int choose;
  double pointSize;

public:
  OverSegmentationAnnotator () : DrawingAnnotator(__func__), pointSize(1.0) {}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

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

    segmenter.initialize(minNormalThreshold,
                          maxNormalThreshold,
                          curvatureThreshold,
                          overlapThreshold,
                          minClusterSize,
                          maxClusterSize,
                          neighborNumber,
                          numSegmentation);

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
    bool useObjectCloud = false;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr (new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_CLOUD_OBJECTS, *cloud_ptr);
    if(cloud_ptr->size() == 0)
    {
      outInfo("Input Object cloud address is empty! Using non-NAN cloud from NormalEstimator");
      rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
      cas.getFS(VIEW_CLOUD_NON_NAN, rcp);
      rs::conversion::from(rcp.cloud(), *cloud_ptr);
      rs::conversion::from(rcp.normals(), *normals_ptr);
    }
    else
    {
      //get normal cloud
      cas.get(VIEW_NORMALS_OBJECTS, *normals_ptr);
      useObjectCloud = true;
    }

    outInfo("Cloud size: " << cloud_ptr->size());
    outInfo("Normals size: " << normals_ptr->size());

    segmenter.setInputClouds(cloud_ptr, normals_ptr);

    if(!useObjectCloud)
    {
      std::vector<rs::Plane> planes;
      scene.annotations.filter(planes);

      segmenter.removePlanes(planes);
    }

    std::vector<pcl::PointIndices> rotational_segments;
    std::vector<pcl::PointIndices> bilateral_segments;
    cas.get(VIEW_ROTATIONAL_SEGMENTATION_IDS, rotational_segments);
    cas.get(VIEW_BILATERAL_SEGMENTATION_IDS, bilateral_segments);

    std::vector<pcl::PointIndices> segments;
    segments.insert(segments.end(), rotational_segments.begin(), rotational_segments.end());
    segments.insert(segments.end(), bilateral_segments.begin(), bilateral_segments.end());

    segmenter.removeSegments(segments);

    segmenter.segment();

    //for visualization purpose
    outInfo("Choosing segment cloud num: " << choose);
    segmenter.getColoredCloud(colored_cloud, choose);

    //publish clusters to CAS
    segmenter.getSegments(linear_segments);
    cas.set(VIEW_SEGMENT_IDS, linear_segments);

    //publish cloud that only has objects
    cas.set(VIEW_CLOUD_OBJECTS, *cloud_ptr);
    cas.set(VIEW_NORMALS_OBJECTS, *normals_ptr);

    return UIMA_ERR_NONE;
  }

  bool callbackKey(const int key, const Source source)
  {
    choose = (int) key - 48;
    if(choose > numSegmentation - 1)
    {
      choose = numSegmentation - 1;
    }
    else if(choose < 0)
    {
      choose = 0;
    }
    return true;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    //const std::string normalsname = this->name + "_normals";

    if(firstRun)
    {
      visualizer.addPointCloud(colored_cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      //visualizer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(colored_cloud, normals, 50, 0.02f, normalsname);
      //visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, normalsname);
    }
    else
    {
      visualizer.updatePointCloud(colored_cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      //visualizer.removePointCloud(normalsname);
      //visualizer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(colored_cloud, normals, 50, 0.02f, normalsname);
      //visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, normalsname);
    }
  }

  void drawImageWithLock(cv::Mat& disp) {}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(OverSegmentationAnnotator)
