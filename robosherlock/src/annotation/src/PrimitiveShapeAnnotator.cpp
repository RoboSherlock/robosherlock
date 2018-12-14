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
#include <pcl/console/time.h>

#include <ros/ros.h>
#include <math.h>

#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
#include <rs/DrawingAnnotator.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_INFO

using namespace uima;

class PrimitiveShapeAnnotator : public DrawingAnnotator
{

private:

  pcl::PCDWriter writer_;

  typedef pcl::PointXYZRGBA PointT;
  pcl::PointCloud<PointT>::Ptr dispCloudPtr_;

  cv::Mat rgb_;

public:
  PrimitiveShapeAnnotator(): DrawingAnnotator(__func__)
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

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    // declare variables for kinect data
    outInfo("process start");
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);

    dispCloudPtr_.reset(new pcl::PointCloud<PointT>);

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    std::vector<rs::Plane> planes;
    std::vector<float> plane_model;

    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *normal_ptr);
    cas.get(VIEW_COLOR_IMAGE, rgb_);

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

    pcl::ProjectInliers<PointT> proj;
    int circles_found = 0;

    int idx = 0;
    for(auto cluster : clusters)
    {
      pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
      rs::ReferenceClusterPoints clusterpoints(cluster.points());
      rs::conversion::from(clusterpoints.indices(), *cluster_indices);

      //visualization
      rs::ImageROI imageRoi(cluster.rois());
      cv::Rect rect;
      rs::conversion::from(imageRoi.roi(), rect);
      cv::rectangle(rgb_, rect, rs::common::cvScalarColors[idx % clusters.size()], 2);

      pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
      pcl::PointCloud<PointT>::Ptr cluster_projected(new pcl::PointCloud<PointT>());
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

      pcl::console::TicToc tt;
      tt.tic();
      pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> be;
      be.setInputCloud(cluster_cloud);
      be.setInputNormals(cluster_normal);
      be.setSearchMethod(typename pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>));
      be.setAngleThreshold(DEG2RAD(70));
      be.setRadiusSearch(0.03);

      pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
      pcl::PointCloud<PointT>::Ptr boundaryCloud(new pcl::PointCloud<PointT>);
      be.compute(*boundaries);
      assert(boundaries->points.size() == cluster_cloud->points.size());
      for(int k = 0; k < boundaries->points.size(); ++k)
      {
        if((int)boundaries->points[k].boundary_point)
        {
          boundaryCloud->points.push_back(cluster_cloud->points[k]);
        }
      }

      boundaryCloud->width = boundaryCloud->points.size();
      boundaryCloud->height = 1;

      //projecting clusters to the plane
      proj.setModelType(pcl::SACMODEL_PLANE);
      proj.setInputCloud(boundaryCloud); //cluster_cloud
      proj.setModelCoefficients(plane_coefficients);
      proj.filter(*cluster_projected);

      pcl::PointIndices::Ptr circle_inliers(new pcl::PointIndices());
      pcl::ModelCoefficients::Ptr circle_coefficients(new pcl::ModelCoefficients());
      pcl::PointCloud<PointT>::Ptr circle(new pcl::PointCloud<PointT>());

      pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices());
      pcl::ModelCoefficients::Ptr line_coefficients(new pcl::ModelCoefficients());
      pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>());

      pcl::SACSegmentation<PointT> seg;
      //finding circles in the projected cluster
      seg.setOptimizeCoefficients(true);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(500);
      seg.setModelType(pcl::SACMODEL_CIRCLE3D);
      seg.setDistanceThreshold(0.005);
      seg.setRadiusLimits(0.025, 0.13);
      // Give as input the filtered point cloud
      seg.setInputCloud(cluster_projected);
      // Call the segmenting method
      seg.segment(*circle_inliers, *circle_coefficients);

      seg.setOptimizeCoefficients(true);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(500);
      seg.setModelType(pcl::SACMODEL_LINE);
      seg.setDistanceThreshold(0.005);
      // Give as input the filtered point cloud
      //      seg.setInputCloud(cluster_cloud);
      // Call the segmenting method
      seg.segment(*line_inliers, *line_coefficients);

      outDebug("Projected Cloud has " << cluster_projected->points.size() << " data point");
      outDebug("Number of CIRCLE inliers found " << circle_inliers->indices.size());
      outDebug("Number of line inliers found " << line_inliers->indices.size());
      pcl::ExtractIndices<PointT> extract;

      rs::Shape shapeAnnot = rs::create<rs::Shape>(tcas);

      if((float)line_inliers->indices.size() / cluster_projected->points.size() > 0.60)
      {
        for(unsigned int k = 0; k < line_inliers->indices.size(); ++k)
        {
          cluster_projected->points[line_inliers->indices[k]].rgba = rs::common::colors[idx % rs::common::numberOfColors];
        }
        *dispCloudPtr_ += *cluster_projected;
        shapeAnnot.shape.set("box");
        shapeAnnot.confidence.set((float)line_inliers->indices.size() / cluster_projected->points.size());
        cv::putText(rgb_, "box", cv::Point(rect.x, rect.y - 10), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        cluster.annotations.append(shapeAnnot);
      }

      else if((float)circle_inliers->indices.size() / cluster_projected->points.size() > 0.3)
      {
        circles_found++;
        outInfo("Circle Model Coefficients:");
        outInfo("x= " << circle_coefficients->values[0] << " y= " << circle_coefficients->values[1] << " z= " << circle_coefficients->values[1] << " R= "
                << circle_coefficients->values[3]);
        float cx = circle_coefficients->values[0];
        float cy = circle_coefficients->values[1];
        float cz = circle_coefficients->values[2];

        float r = circle_coefficients->values[3];

        float nx = circle_coefficients->values[4];
        float ny = circle_coefficients->values[5];
        float nz = circle_coefficients->values[6];

        float s = 1.0f / (nx * nx + ny * ny + nz * nz);
        float v3x = s * nx;
        float v3y = s * ny;
        float v3z = s * nz;

        s = 1.0f / (v3x * v3x + v3z * v3z);
        float v1x = s * v3z;
        float v1y = 0.0f;
        float v1z = s * -v3x;

        float v2x = v3y * v1z - v3z * v1y;
        float v2y = v3z * v1x - v3x * v1z;
        float v2z = v3x * v1y - v3y * v1x;

        for(int theta = 0; theta < 360; theta += 5)
        {
          pcl::PointXYZRGBA point;
          point.rgba = rs::common::colors[idx % rs::common::numberOfColors];

          point.x = cx + r * (v1x * cos(theta) + v2x * sin(theta));
          point.y = cy + r * (v1y * cos(theta) + v2y * sin(theta));
          point.z = cz + r * (v1z * cos(theta) + v2z * sin(theta));
          dispCloudPtr_->points.push_back(point);
        }

        for(unsigned int k = 0; k < circle_inliers->indices.size(); ++k)
        {
          cluster_projected->points[line_inliers->indices[k]].rgba = rs::common::colors[idx % rs::common::numberOfColors];
        }
        *dispCloudPtr_ += *cluster_projected;

        extract.setInputCloud(cluster_projected);
        extract.setIndices(circle_inliers);
        extract.setNegative(false);
        extract.filter(*circle);
        shapeAnnot.shape.set("round");
        cv::putText(rgb_, "round", cv::Point(rect.x, rect.y - 10), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        shapeAnnot.confidence.set((float)circle_inliers->indices.size() / cluster_projected->points.size());
        cluster.annotations.append(shapeAnnot);
      }

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
          cv::putText(rgb_, "flat", cv::Point(rect.x, rect.y - 25), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
          shape.confidence.set(std::abs(0.25 / 2 - min_edge / max_edge) / 0.125);
          cluster.annotations.append(shape);
        }
      }

      idx++;
    }
    return UIMA_ERR_NONE;
  }
  void drawImageWithLock(cv::Mat &disp)
  {
    disp = rgb_.clone();
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    double pointSize = 1.0;
    if(firstRun)
    {
      visualizer.addPointCloud(dispCloudPtr_, std::string("stuff"));
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, std::string("stuff"));
    }
    else
    {
      visualizer.updatePointCloud(dispCloudPtr_, std::string("stuff"));
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, std::string("stuff"));
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(PrimitiveShapeAnnotator)
