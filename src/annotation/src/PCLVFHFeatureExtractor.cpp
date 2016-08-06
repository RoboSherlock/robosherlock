/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *            Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *            Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
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

#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/features/vfh.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <rs/utils/output.h>

using namespace uima;

//todo:: extend this to other PCL features
typedef pcl::PointXYZRGBA PointT;

class PCLVFHFeatureExtractor : public Annotator
{
private:

  pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;

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
    tf::StampedTransform vp;
    if(scene.viewPoint.has())
    {
      rs::conversion::from(scene.viewPoint.get(), vp);
      tf::Quaternion q = vp.getRotation();
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      outDebug("VIEWPOINT:" << roll << " " << pitch << " " << yaw << std::endl);
    }

    pcl::PointCloud<PointT>::Ptr cloudPtr(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normalsPtr(new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_CLOUD, *cloudPtr);
    cas.get(VIEW_NORMALS, *normalsPtr);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
    vfh.setSearchMethod(tree);
    vfh.setInputCloud(cloudPtr);
    vfh.setInputNormals(normalsPtr);



    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    outDebug("Processing " << clusters.size() << " point clusters");

    for(std::vector<rs::Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
      pcl::PointIndices::Ptr clusterPtr(new pcl::PointIndices);
      rs::ReferenceClusterPoints clusterpoints(it->points());
      rs::conversion::from(clusterpoints.indices(), *clusterPtr);
      pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308> ());
      vfh.setIndices(clusterPtr);
      vfh.compute(*vfhs);

      rs::PclFeature annotation = rs::create<rs::PclFeature>(tcas);
      std::vector<float> vecVFHFeature;
      vecVFHFeature.insert(vecVFHFeature.begin(), vfhs->points[0].histogram, vfhs->points[0].histogram + 308);
      annotation.feat_type.set("VFH");
      annotation.feature.set(vecVFHFeature);
      it->annotations.append(annotation);
    }

    return UIMA_ERR_NONE;
  }
};

MAKE_AE(PCLVFHFeatureExtractor)
