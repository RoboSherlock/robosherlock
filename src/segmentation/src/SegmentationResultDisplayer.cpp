/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
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

// UIMA
#include <uima/api.hpp>

// RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/segmentation/array_utils.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <stdlib.h>
#include <time.h>

using namespace uima;

class SegmentationResultDisplayer : public DrawingAnnotator
{

private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  int numSegments;

  double pointSize;

public:

  SegmentationResultDisplayer(): DrawingAnnotator(__func__), pointSize(1.0) {}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("Initialize");

    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    srand (time(NULL));

    numSegments = 0;

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("Destroy");

    return UIMA_ERR_NONE;
  }

private:

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("Process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);

    //get plane indices if it has
    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);
    if(!planes.empty())
    {
      std::vector<int> object_indices;
      std::vector<int> plane_indices;
      std::vector<int> cloudIds(cloud_ptr->size());
      for(size_t pointId = 0; pointId < cloud_ptr->size(); pointId++)
      {
        cloudIds[pointId] = pointId;
      }

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
    }
    else
    {
      cloud = cloud_ptr;
    }

    //get segments
    std::vector<pcl::PointIndices> rotational_segments;
    std::vector<pcl::PointIndices> bilateral_segments;
    cas.get(VIEW_ROTATIONAL_SEGMENTATION_IDS, rotational_segments);
    cas.get(VIEW_BILATERAL_SEGMENTATION_IDS, bilateral_segments);
    numSegments = rotational_segments.size() + bilateral_segments.size();

    for(size_t pointId = 0; pointId < cloud->size(); pointId++)
    {
      cloud->points[pointId].r = 30;
      cloud->points[pointId].g = 30;
      cloud->points[pointId].b = 30;
    }

    for(size_t segmentId = 0; segmentId < rotational_segments.size(); segmentId++)
    {
      int r = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));
      int g = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));
      int b = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));
      for(size_t pointIdIt = 0; pointIdIt < rotational_segments[segmentId].indices.size(); pointIdIt++)
      {
        int pointId = rotational_segments[segmentId].indices[pointIdIt];
        cloud->points[pointId].r = r;
        cloud->points[pointId].g = g;
        cloud->points[pointId].b = b;
      }
    }

    for(size_t segmentId = 0; segmentId < bilateral_segments.size(); segmentId++)
    {
      int r = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));
      int g = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));
      int b = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));
      for(size_t pointIdIt = 0; pointIdIt < bilateral_segments[segmentId].indices.size(); pointIdIt++)
      {
        int pointId = bilateral_segments[segmentId].indices[pointIdIt];
        cloud->points[pointId].r = r;
        cloud->points[pointId].g = g;
        cloud->points[pointId].b = b;
      }
    }

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.addText("Total Segment " + std::to_string(numSegments), 15, 125, 24, 1.0, 1.0, 1.0);
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeAllShapes();
      visualizer.addText("Total Segment " + std::to_string(numSegments), 15, 125, 24, 1.0, 1.0, 1.0);
    }
  }

};

MAKE_AE(SegmentationResultDisplayer)
