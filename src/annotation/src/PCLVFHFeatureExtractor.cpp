/*
 * Copyright (c) 2015, Ferenc Balint-Benczedi, balintbe@cs.uni-bremen.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
