/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
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

#include <uima/api.hpp>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/types/all_types.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <rs/segmentation/array_utils.hpp>

using namespace uima;

class SegmentFilterHelper : public Annotator
{
public:
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &resSpec)
  {
    MEASURE_TIME;
    outInfo("process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    bool isObjectPassed = true;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);

    cas.get(VIEW_CLOUD_OBJECTS, *cloud_ptr);
    if(cloud_ptr->size() == 0)
    {
      outInfo("Input Object cloud address is empty! Using scene cloud");
      cas.get(VIEW_CLOUD, *cloud_ptr);
      cas.get(VIEW_NORMALS, *normals_ptr);
      isObjectPassed = false;
    }
    else
    {
      //get normal cloud
      cas.get(VIEW_NORMALS_OBJECTS, *normals_ptr);
    }
    outInfo("Input cloud size: " << cloud_ptr->size());

    std::vector<int> segmentIds;
    std::vector<int> remainedIds;
    std::vector<int> cloudIds(cloud_ptr->size());
    for(size_t pointId = 0; pointId < cloud_ptr->size(); pointId++)
    {
      cloudIds[pointId] = pointId;
    }

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

    if(segmentIds.size() > 0)
    {
      remainedIds = Difference(cloudIds, segmentIds);
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    //filter
    pcl::copyPointCloud(*cloud_ptr, remainedIds, *cloud);
    pcl::copyPointCloud(*normals_ptr, remainedIds, *normals);
    outInfo("Cloud size after segment filter: " << remainedIds.size());
    if(isObjectPassed)
    {
      cas.set(VIEW_CLOUD_OBJECTS, *cloud);
      cas.set(VIEW_NORMALS_OBJECTS, *normals);
    }
    else
    {
      cas.set(VIEW_CLOUD, *cloud);
      cas.set(VIEW_NORMALS, *normals);
    }


    return UIMA_ERR_NONE;
  }
};

MAKE_AE(SegmentFilterHelper)
