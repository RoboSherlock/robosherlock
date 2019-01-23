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

//uima
#include <uima/api.hpp>
#include <uima/fsfilterbuilder.hpp>

//
#include <opencv2/opencv.hpp>

//pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/impl/convex_hull.hpp>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

//project
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>

//#define DEBUG_OUTPUT 1;
using namespace uima;

/**
 * @brief The PointCloudClusterExtractor class
 * given a plane equation,
 */
class PointCloudClusterExtractor : public DrawingAnnotator
{
private:
  typedef pcl::PointXYZRGBA PointT;

  struct Cluster
  {
    size_t indices_index_;
    cv::Rect roi_, roi_hires_;
    cv::Mat mask, mask_hires_;
  };

  Type cloud_type;
  cv::Mat color;
  int cluster_min_size, cluster_max_size;
  float polygon_min_height, polygon_max_height;
  float cluster_tolerance;
  float plane_distance_threshold;
  pcl::PointCloud<PointT>::Ptr cloud_ptr;

  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<Cluster> clusters;
  double pointSize;

  enum
  {
    EC,
    OEC,
    OEC_prism
  } mode;

public:

  PointCloudClusterExtractor(): DrawingAnnotator(__func__), cluster_min_size(500), cluster_max_size(25000),
    polygon_min_height(0.02), polygon_max_height(0.5), cluster_tolerance(0.02), pointSize(1), mode(OEC)
  {
    cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("mode"))
    {
      std::string sMode;
      ctx.extractValue("mode", sMode);
      outInfo("mode set to: " << sMode);
      if(sMode == "OEC")
      {
        mode = OEC;
      }
      else if(sMode == "EC")
      {
        mode = EC;
      }
      else if(sMode == "OEC_prism")
      {
        mode = OEC_prism;
      }
    }
    if(ctx.isParameterDefined("plane_distance_threshold"))
    {
      ctx.extractValue("plane_distance_threshold", plane_distance_threshold);
    }
    if(ctx.isParameterDefined("polygon_min_height"))
    {
      ctx.extractValue("polygon_min_height", polygon_min_height);
    }
    if(ctx.isParameterDefined("polygon_max_height"))
    {
      ctx.extractValue("polygon_max_height", polygon_max_height);
    }
    if(ctx.isParameterDefined("cluster_tolerance"))
    {
      ctx.extractValue("cluster_tolerance", cluster_tolerance);
    }
    if(ctx.isParameterDefined("cluster_max_size"))
    {
      ctx.extractValue("cluster_max_size", cluster_max_size);
    }
    if(ctx.isParameterDefined("cluster_min_size"))
    {
      ctx.extractValue("cluster_min_size", cluster_min_size);
    }
    setAnnotatorContext(ctx);
    return UIMA_ERR_NONE;
  }

  TyErrorId reconfigure()
  {
    outInfo("Reconfiguring");
    AnnotatorContext &ctx = getAnnotatorContext();
    initialize(ctx);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

private:

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");
    rs::StopWatch clock;
    double t = clock.getTime();

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
    pcl::PointIndices::Ptr prism_inliers(new pcl::PointIndices());
    cluster_indices.clear();

    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *cloud_normals);
    cas.get(VIEW_COLOR_IMAGE_HD, color);

    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);
    if(planes.empty())
    {
      outInfo("NO PLANE COEFFICIENTS SET!! RUN A PLANE ESIMTATION BEFORE!!!");
      outInfo(clock.getTime() << " ms.");
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;
    }

    plane_coefficients->values = planes[0].model();
    plane_inliers->indices = planes[0].inliers();

    if(plane_coefficients->values.empty())
    {
      outInfo("PLane COEFFICIENTS EMPTY");
      outInfo(clock.getTime() << " ms.");
      return UIMA_ERR_NONE;
    }
    outDebug("getting input data took : " << clock.getTime() - t << " ms.");
    t = clock.getTime();

    if(mode == EC)
    {
      cloudPreProcessing(cloud_ptr, plane_coefficients, plane_inliers, prism_inliers);
      outDebug("cloud preprocessing took : " << clock.getTime() - t << " ms.");
      t = clock.getTime();
      pointCloudClustering(cloud_ptr, prism_inliers, cluster_indices);
    }
    else if(mode == OEC)
    {
      organizedCloudClustering(cloud_ptr, cloud_normals, plane_inliers, cluster_indices, prism_inliers);
    }
    else if(mode == OEC_prism)
    {
      cloudPreProcessing(cloud_ptr, plane_coefficients, plane_inliers, prism_inliers);
      organizedCloudClustering(cloud_ptr, cloud_normals, plane_inliers, cluster_indices, prism_inliers);
    }
    outDebug("euclidean clustering took : " << clock.getTime() - t << " ms.");
    t = clock.getTime();

    clusters.resize(cluster_indices.size());

    #pragma omp parallel for schedule(dynamic)
    for(size_t i = 0; i < cluster_indices.size(); ++i)
    {
      Cluster &cluster = clusters[i];
      cluster.indices_index_ = i;

      createImageRoi(cluster);
    }
    outDebug("conversion to image ROI took: " << clock.getTime() - t << " ms.");
    t = clock.getTime();

    for(size_t i = 0; i < cluster_indices.size(); ++i)
    {
      Cluster &cluster = clusters[i];
      const pcl::PointIndices &indices = cluster_indices[i];

      rs::ObjectHypothesis uimaCluster = rs::create<rs::ObjectHypothesis>(tcas);
      rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
      rs::PointIndices uimaIndices = rs::conversion::to(tcas, indices);

      //outDebug("cluster size: " << indices.indices.size());
      rcp.indices.set(uimaIndices);

      rs::ImageROI imageRoi = rs::create<rs::ImageROI>(tcas);
      imageRoi.mask(rs::conversion::to(tcas, cluster.mask));
      imageRoi.mask_hires(rs::conversion::to(tcas, cluster.mask_hires_));
      imageRoi.roi(rs::conversion::to(tcas, cluster.roi_));
      imageRoi.roi_hires(rs::conversion::to(tcas, cluster.roi_hires_));

      uimaCluster.points.set(rcp);
      uimaCluster.rois.set(imageRoi);
      uimaCluster.source.set("EuclideanClustering");
      scene.identifiables.append(uimaCluster);
    }
    outDebug("adding clusters took: " << clock.getTime() - t << " ms.");

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = color.clone();
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      cv::rectangle(disp, clusters[i].roi_hires_, rs::common::cvScalarColors[i % rs::common::numberOfColors]);
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;
    for(size_t i = 0; i < cluster_indices.size(); ++i)
    {
      const pcl::PointIndices &indices = cluster_indices[i];
      for(size_t j = 0; j < indices.indices.size(); ++j)
      {
        size_t index = indices.indices[j];
        cloud_ptr->points[index].rgba = rs::common::colors[i % rs::common::numberOfColors];
      }
    }

    if(firstRun)
    {
      visualizer.addPointCloud(cloud_ptr, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud_ptr, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }

  void cloudPreProcessing(const pcl::PointCloud<PointT>::Ptr &cloud,
                          const pcl::ModelCoefficients::Ptr &plane_coeffs,
                          const pcl::PointIndices::Ptr &plane_inliers,
                          pcl::PointIndices::Ptr &prism_inliers)
  {
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(plane_inliers);
    ei.filter(*cloud_plane);

    // Get convex hull
    pcl::PointCloud<PointT>::Ptr cloud_hull(new pcl::PointCloud<PointT>);
    pcl::ConvexHull<PointT> chull;
    chull.setInputCloud(cloud_plane);
    chull.reconstruct(*cloud_hull);

    outDebug(" Convex hull has: " << cloud_hull->points.size() << " data points.");

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_hull, centroid);
    double hull_shrink_factor = 0;

    for(size_t i = 0; i < cloud_hull->points.size(); ++i)
    {
      Eigen::Vector4f scaled_vector = (cloud_hull->points[i].getVector4fMap() - centroid) * hull_shrink_factor;
      cloud_hull->points[i].getVector4fMap() -= scaled_vector;
    }

    // Get points in polygonal prism
    pcl::ExtractPolygonalPrismData<PointT> epm;
    epm.setInputCloud(cloud);
    epm.setInputPlanarHull(cloud_hull);

    /*TODO::Check why this is so sensitive
    why does this need to be so freaking high? (if set lower it finds
    points that are actually part of the plane)*/
    epm.setHeightLimits(polygon_min_height, polygon_max_height);
    epm.segment(*prism_inliers);
    outDebug("points in the prism: " << prism_inliers->indices.size());
  }

  bool pointCloudClustering(const pcl::PointCloud<PointT>::Ptr &cloud,
                            const pcl::PointIndices::Ptr &indices,
                            std::vector<pcl::PointIndices> &cluster_indices)
  {

    if(indices->indices.size() > 0)
    {
      pcl::EuclideanClusterExtraction<PointT> ec;
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
      tree->setInputCloud(cloud, boost::make_shared<std::vector<int>>(indices->indices));
      ec.setClusterTolerance(cluster_tolerance);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud);
      ec.setIndices(indices);
      ec.setMinClusterSize(cluster_min_size);
      ec.extract(cluster_indices);
      return true;
    }
    else
    {
      return false;
    }
  }

  void organizedCloudClustering(const pcl::PointCloud<PointT>::Ptr &cloud,
                                const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                const pcl::PointIndices::Ptr &plane_inliers,
                                std::vector<pcl::PointIndices> &cluster_indices,
                                const pcl::PointIndices::Ptr &prism_inliers)
  {
    std::vector<bool> ignore_labels;
    pcl::PointCloud<pcl::Label>::Ptr input_labels(new pcl::PointCloud<pcl::Label>);

    pcl::Label label;

    ignore_labels.resize(2);
    ignore_labels[0] = true;
    ignore_labels[1] = false;


    input_labels->height = cloud->height;
    input_labels->width = cloud->width;

    if(prism_inliers->indices.size() == 0)
    {
      label.label = 1;
      input_labels->points.resize(cloud->points.size(), label);
      for(size_t i = 0; i < plane_inliers->indices.size(); i++)
        input_labels->points[plane_inliers->indices[i]].label = 0;//std::numeric_limits<unsigned>::max();

    }
    else
    {
      label.label = 0;
      input_labels->points.resize(cloud->points.size(), label);
      for(size_t i = 0; i < prism_inliers->indices.size(); ++i)
      {
        input_labels->points[prism_inliers->indices[i]].label = 1;
      }
    }

    pcl::PointCloud<pcl::Label>::Ptr output_labels(new pcl::PointCloud<pcl::Label>);
    pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr ecc(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>());
    ecc->setInputCloud(cloud);
    ecc->setLabels(input_labels);
    ecc->setExcludeLabels(ignore_labels);
    ecc->setDistanceThreshold(cluster_tolerance, true);
    ecc->setInputNormals(normals);
    std::vector<pcl::PointIndices> cluster_i;
    pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> segmenter(ecc);
    segmenter.setInputCloud(cloud);
    segmenter.segment(*output_labels, cluster_i);

    int good_clusters = 0;
    for(int i = 0 ; i < cluster_i.size(); ++i)
    {
      if(cluster_i.at(i).indices.size() > cluster_min_size && cluster_i.at(i).indices.size() < cluster_max_size)
      {
        good_clusters++;
        cluster_indices.push_back(cluster_i.at(i));
      }
    }
    outInfo("Found " << good_clusters << " good clusters!");
  }

  /**
   * given orignal_image and reference cluster points, compute an image containing only the cluster
   */
  void createImageRoi(Cluster &cluster) const
  {
    const pcl::PointIndices &indices = cluster_indices[cluster.indices_index_];

    size_t width = cloud_ptr->width;
    size_t height = cloud_ptr->height;

    int min_x = width;
    int max_x = -1;
    int min_y = height;
    int max_y = -1;

    cv::Mat mask_full = cv::Mat::zeros(height, width, CV_8U);

    // get min / max extents (rectangular bounding box in image (pixel) coordinates)
    //#pragma omp parallel for
    for(size_t i = 0; i < indices.indices.size(); ++i)
    {
      const int idx = indices.indices[i];
      const int x = idx % width;
      const int y = idx / width;

      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);

      mask_full.at<uint8_t>(y, x) = 255;
    }

    cluster.roi_ = cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    cluster.roi_hires_ = cv::Rect(cluster.roi_.x << 1, cluster.roi_.y << 1, cluster.roi_.width << 1, cluster.roi_.height << 1);
    mask_full(cluster.roi_).copyTo(cluster.mask);
    cv::resize(cluster.mask, cluster.mask_hires_, cv::Size(0, 0), 2.0, 2.0, cv::INTER_NEAREST);
  }
};

MAKE_AE(PointCloudClusterExtractor)
