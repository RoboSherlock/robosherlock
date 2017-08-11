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

#include <uima/api.hpp>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/types/all_types.h>

#include <pcl/pcl_macros.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>


#define DEBUG_OUTPUT 0
using namespace uima;

class PointNormalNaNFilter : public Annotator
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

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *normals_ptr);
    outInfo("Input cloud size: " << cloud_ptr->size());

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    //filter
    std::vector<int> non_NaN_ids;
    pcl::removeNaNNormalsFromPointCloud(*normals_ptr, *normals, non_NaN_ids);
    outInfo("Cloud size after filter: " << non_NaN_ids.size());
    pcl::copyPointCloud(*cloud_ptr, non_NaN_ids, *cloud);

    cas.set(VIEW_CLOUD, *cloud);
    cas.set(VIEW_NORMALS, *normals);

    return UIMA_ERR_NONE;
  }
};

MAKE_AE(PointNormalNaNFilter)
