/*
 * Copyright (c) 2011, Ferenc Balint-Benczedi <balintbe@tzi.de>
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

#include <uima/api.hpp>

#include <Eigen/Sparse>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <ros/ros.h>
#include <math.h>

#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

#define DEBUG_OUTPUT 0
#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_INFO

using namespace uima;

class PrimitiveShapeAnnotator : public Annotator
{

private:

  pcl::PCDWriter writer;

public:
  PrimitiveShapeAnnotator()
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {

    //  delete ros_helper;
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    // declare variables for kinect data
    outInfo("process start");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;
    std::vector<rs::Plane> planes;
    std::vector<float> plane_model;

    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *normal_ptr);
    scene.identifiables.filter(clusters);
    scene.annotations.filter(planes);
    if(planes.empty())
    {
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;
    }

    plane_model = planes[0].model();

    if(plane_model.size() == 0)
    {
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;
    }
    outInfo("Processing " << clusters.size() << " point clusters");

    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
    plane_coefficients->values.push_back(plane_model[0]);
    plane_coefficients->values.push_back(plane_model[1]);
    plane_coefficients->values.push_back(plane_model[2]);
    plane_coefficients->values.push_back(plane_model[3]);

    pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
    int circles_found = 0;
    int boxes_found = 0;

    for(auto cluster: clusters)
    {
      std::vector<rs::Geometry> geom;
      cluster.annotations.filter(geom);
      if(!geom.empty())
      {
        rs::BoundingBox3D box = geom[0].boundingBox();

        float max_edge = std::max(box.width(), std::max(box.depth(), box.height()));
        float min_edge = std::min(box.width(), std::min(box.depth(), box.height()));
        if(min_edge / max_edge <= 0.25)
        {
          rs::Shape shape = rs::create<rs::Shape>(tcas);
          shape.shape.set("flat");
          cluster.annotations.append(shape);
        }
      }

      pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
      rs::ReferenceClusterPoints clusterpoints(cluster.points());
      rs::conversion::from(clusterpoints.indices(), *cluster_indices);

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster_projected(new pcl::PointCloud<pcl::PointXYZRGBA>());
      pcl::PointCloud<pcl::Normal>::Ptr cluster_normal(new pcl::PointCloud<pcl::Normal>());
      for(std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
          pit != cluster_indices->indices.end(); pit++)
      {
        cluster_cloud->points.push_back(cloud_ptr->points[*pit]);
        cluster_normal->points.push_back(normal_ptr->points[*pit]);
      }
      cluster_cloud->width = cluster_cloud->points.size();
      cluster_cloud->height = 1;
      cluster_cloud->is_dense = true;
      cluster_normal->width = cluster_normal->points.size();
      cluster_normal->height = 1;
      cluster_normal->is_dense = true;

      std::stringstream ss;

#if DEBUG_OUTPUT
      ss << "cluster" << it - clusters.begin() << ".pcd";
      writer.write<pcl::PointXYZRGBA>(ss.str(), *cluster_cloud);
#endif


      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::ConvexHull<pcl::PointXYZRGBA> chull;
      chull.setInputCloud(cluster_cloud);
      chull.reconstruct(*cloud_hull);

      outDebug("Convex hull has: " << cloud_hull->points.size() << " data points.");

      //projecting clusters to the plane
      proj.setModelType(pcl::SACMODEL_PLANE);
      proj.setInputCloud(cluster_cloud);
      proj.setModelCoefficients(plane_coefficients);
      proj.filter(*cluster_projected);

#if DEBUG_OUTPUT
      ss.str(std::string());
      ss.clear();
      ss << "hull" << it - clusters.begin() << ".pcd";
      writer.write<pcl::PointXYZRGBA>(ss.str(), *cloud_hull);

      ss.str(std::string());
      ss.clear();
      ss << "projected_cluster" << it - clusters.begin() << ".pcd";
      writer.write<pcl::PointXYZRGBA>(ss.str(), *cluster_projected);
#endif

      pcl::PointIndices::Ptr circle_inliers(new pcl::PointIndices());
      pcl::ModelCoefficients::Ptr circle_coefficients(new pcl::ModelCoefficients());
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr circle(new pcl::PointCloud<pcl::PointXYZRGBA>());

      pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
      pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>());

      pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
      //finding circles in the projected cluster
      seg.setOptimizeCoefficients(true);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(1000);
      seg.setModelType(pcl::SACMODEL_CIRCLE2D);
      seg.setDistanceThreshold(0.005);
      seg.setRadiusLimits(0.025, 0.15);
      // Give as input the filtered point cloud
      seg.setInputCloud(cluster_projected);
      // Call the segmenting method
      seg.segment(*circle_inliers, *circle_coefficients);

      seg.setOptimizeCoefficients(true);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(50);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setDistanceThreshold(0.001);
      // Give as input the filtered point cloud
      seg.setInputCloud(cluster_cloud);
      // Call the segmenting method
      seg.segment(*plane_inliers, *plane_coefficients);

      outDebug("Projected Cloud has " << cluster_projected->points.size() << " data point");
      outDebug("Number of CIRCLE inliers found " << circle_inliers->indices.size());
      outDebug("Number of plane inliers found " << plane_inliers->indices.size());
      pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

      rs::Shape shape_annot = rs::create<rs::Shape>(tcas);
      if(circle_inliers->indices.size() > 0)
        /*&& (plane_inliers->indices.size() == 0 || plane_inliers->indices.size() < 1100)*/
      {
        circles_found++;
        outDebug("Circle Model Coefficients:");
        outDebug("x= " << circle_coefficients->values[0] << " y= " << circle_coefficients->values[1] << " R= "
                 << circle_coefficients->values[2]);

        extract.setInputCloud(cluster_projected);
        extract.setIndices(circle_inliers);
        extract.setNegative(false);
        extract.filter(*circle);
        shape_annot.shape.set(std::string("round"));

#if DEBUG_OUTPUT
        ss.str(std::string());
        ss.clear();
        ss << "circle_cloud" << it - clusters.begin() << ".pcd";
        writer.write<pcl::PointXYZRGBA>(ss.str(), *circle);
#endif

      }
      else
      {
        boxes_found++;
        extract.setInputCloud(cluster_cloud);
        extract.setIndices(plane_inliers);
        extract.setNegative(false);
        extract.filter(*plane);
        shape_annot.shape.set(std::string("box"));

#if DEBUG_OUTPUT
        ss.str(std::string());
        ss.clear();
        ss << "plane_cloud" << it - clusters.begin() << ".pcd";
        writer.write<pcl::PointXYZRGBA>(ss.str(), *plane);
#endif

      }
      cluster.annotations.append(shape_annot);
    }
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(PrimitiveShapeAnnotator)
