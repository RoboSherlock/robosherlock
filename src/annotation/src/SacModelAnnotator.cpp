/* Copyright (c) 2013, Nico Lehmann <nicolehm@tzi.de>
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

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>

#define DEBUG_OUTPUT 0
using namespace uima;

class SacModelAnnotator : public Annotator
{

private:
  bool apply_to_clusters;

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


    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);
    outInfo("Processing " << clusters.size() << " point clusters");
    for(std::vector<rs::Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
      pcl::PointIndicesPtr clusterIndices(new pcl::PointIndices());
      rs::conversion::from(((rs::ReferenceClusterPoints)it->points.get()).indices(), *clusterIndices);


      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
      pcl::PointCloud<pcl::Normal>::Ptr clusterNormal(new pcl::PointCloud<pcl::Normal>);
      pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr cylinderInliers(new pcl::PointIndices);

      for(std::vector<int>::const_iterator pit = clusterIndices->indices.begin();
          pit != clusterIndices->indices.end(); pit++)
      {
        clusterCloud->points.push_back(cloud_ptr->points[*pit]);
        clusterNormal->points.push_back(normals_ptr->points[*pit]);
      }

      clusterCloud->width = clusterCloud->points.size();
      clusterCloud->height = 1;
      clusterCloud->is_dense = true;
      clusterNormal->width = normals_ptr->points.size();
      clusterNormal->height = 1;
      clusterNormal->is_dense = true;
      outDebug("Fitting the selected SAC model");

      pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_CYLINDER);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setNormalDistanceWeight(0.1);
      seg.setMaxIterations(1000);
      seg.setDistanceThreshold(0.03);
      seg.setRadiusLimits(0, 0.1);
      seg.setInputCloud(clusterCloud);
      seg.setInputNormals(clusterNormal);

      // Obtain the cylinder inliers and coefficients
      seg.segment(*cylinderInliers, *coefficients_cylinder);

      outDebug("Size of cluster " << it - clusters.begin() << ": " << clusterCloud->width);
      outDebug("Number of CYLINDER inliers: " << cylinderInliers->indices.size());
      if((float)cylinderInliers->indices.size() / clusterCloud->width > 0.5)
      {
        rs::Shape shapeAnnotation = rs::create<rs::Shape>(tcas);
        rs::CylindricalShape cylinderAnnotation = rs::create<rs::CylindricalShape>(tcas);
        shapeAnnotation.shape.set("cylinder");
        cylinderAnnotation.shape.set("cylinder");
        it->annotations.append(shapeAnnotation);
        it->annotations.append(cylinderAnnotation);
      }

#if DEBUG_OUTPUT
      pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
      extract.setInputCloud(clusterCloud);
      extract.setIndices(cylinderInliers);
      extract.setNegative(false);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZRGBA> ());
      extract.filter(*cloud_cylinder);
      pcl::PCDWriter writer;
      if(cloud_cylinder->points.empty())
      {
        std::cerr << "Can't find the cylindrical component." << std::endl;
      }
      else
      {
        std::stringstream fn;
        fn << "cylinder" << it - clusters.begin() << ".pcd";
        writer.write(fn.str(), *cloud_cylinder, false);
      }
#endif
    }
    return UIMA_ERR_NONE;
  }

};

MAKE_AE(SacModelAnnotator)
