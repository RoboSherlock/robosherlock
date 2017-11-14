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
#include <math.h>

using namespace uima;

class SegmentationResultDisplayer : public DrawingAnnotator
{

private:
  struct Cluster
  {
    pcl::PointIndices indices;
    cv::Rect roi, roiHires;
    cv::Mat mask, maskHires;
  };

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  cv::Mat rgb_;

  std::vector<int> object_indices;
  pcl::PointIndices mapping_to_original;

  int numSegments;
  bool processCorrespondence;

  double pointSize;

public:

  SegmentationResultDisplayer(): DrawingAnnotator(__func__), pointSize(1.0) {}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("Initialize");

    ctx.extractValue("processCorrespondence", processCorrespondence);

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

    rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
    cas.get(VIEW_CLOUD_NON_NAN, rcp);
    rs::conversion::from(rcp.cloud(), *cloud_ptr);
    rs::conversion::from(rcp.indices.get(), mapping_to_original);


    cas.get(VIEW_COLOR_IMAGE, rgb_);

    //get plane indices if it has
    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);
    if(!planes.empty())
    {
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

    processSegment(rotational_segments, tcas, scene);
    processSegment(bilateral_segments, tcas, scene);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    const std::string text = "segment_results";

    if(firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.addText("Total Segment " + std::to_string(numSegments), 15, 125, 24, 1.0, 1.0, 1.0, text);
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeShape(text);
      visualizer.addText("Total Segment " + std::to_string(numSegments), 15, 125, 24, 1.0, 1.0, 1.0, text);
    }
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    if(processCorrespondence)
    {
      disp=rgb_.clone();
    }
  }

  cv::Point indexToCoordinates(int index, cv::Mat& rgb)
  {
	   int row = index / rgb.size().width + 1;
	   int col = index % rgb.size().width + 1;

     return cv::Point(col, row);
  }

  void processSegment(std::vector<pcl::PointIndices>& segments, uima::CAS& tcas, rs::Scene& scene)
  {
    for(size_t segmentId = 0; segmentId < segments.size(); segmentId++)
    {
      int r = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));
      int g = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));
      int b = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));

      pcl::PointIndices original_indices;

      for(size_t pointIdIt = 0; pointIdIt < segments[segmentId].indices.size(); pointIdIt++)
      {
        int pointId = segments[segmentId].indices[pointIdIt];
        cloud->points[pointId].r = r;
        cloud->points[pointId].g = g;
        cloud->points[pointId].b = b;

        if (processCorrespondence)
        {
          int original_index = mapping_to_original.indices[object_indices[pointId]];
          original_indices.indices.push_back(original_index);
          cv::Point current = indexToCoordinates(original_index, rgb_);

          rgb_.at<cv::Vec3b>(current)[0] = r;
          rgb_.at<cv::Vec3b>(current)[1] = g;
          rgb_.at<cv::Vec3b>(current)[2] = b;
        }
      }

      //publish Clusters to CAS
      if(processCorrespondence)
      {
        rs::Cluster uimaCluster = rs::create<rs::Cluster>(tcas);
        rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
        rs::PointIndices uimaIndices = rs::conversion::to(tcas, original_indices);

        Cluster currentCluster;
        currentCluster.indices = original_indices;
        createImageRoi(currentCluster, rgb_);

        rcp.indices.set(uimaIndices);
        rs::ImageROI imageRoi = rs::create<rs::ImageROI>(tcas);
        imageRoi.mask(rs::conversion::to(tcas, currentCluster.mask));
        imageRoi.mask_hires(rs::conversion::to(tcas, currentCluster.maskHires));
        imageRoi.roi(rs::conversion::to(tcas, currentCluster.roi));
        imageRoi.roi_hires(rs::conversion::to(tcas, currentCluster.roiHires));

        uimaCluster.points.set(rcp);
        uimaCluster.rois.set(imageRoi);
        uimaCluster.source.set("SymmetryClustering");
        scene.identifiables.append(uimaCluster);
      }
    }
  }

  //reuse function from PointCloudClusterExtractor
  void createImageRoi(Cluster &cluster, cv::Mat rgb) const
  {
    size_t width = rgb.size().width;
    size_t height = rgb.size().height;

    int min_x = width;
    int max_x = -1;
    int min_y = height;
    int max_y = -1;

    cv::Mat mask_full = cv::Mat::zeros(height, width, CV_8U);

    // get min / max extents (rectangular bounding box in image (pixel) coordinates)
    for(size_t i = 0; i < cluster.indices.indices.size(); ++i)
    {
      const int idx = cluster.indices.indices[i];
      const int x = idx % width;
      const int y = idx / width;

      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);

      mask_full.at<uint8_t>(y, x) = 255;
    }

    cluster.roi = cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    cluster.roiHires = cv::Rect(cluster.roi.x << 1, cluster.roi.y << 1, cluster.roi.width << 1, cluster.roi.height << 1);
    mask_full(cluster.roi).copyTo(cluster.mask);
    cv::resize(cluster.mask, cluster.maskHires, cv::Size(0, 0), 2.0, 2.0, cv::INTER_NEAREST);
  }

};

MAKE_AE(SegmentationResultDisplayer)
