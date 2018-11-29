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

#include <uima/api.hpp>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

//RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/types/all_types.h>

#include <rs/symmetrysegmentation/OverSegmenter.h>
#include <rs/symmetrysegmentation/RotationalSymmetryExtractor.h>
#include <rs/symmetrysegmentation/RotationalSymmetrySegmenter.h>
#include <rs/symmetrysegmentation/BilateralSymmetryExtractor.h>
#include <rs/symmetrysegmentation/BilateralSymmetrySegmenter.h>

//graph
#include <rs/graph/Graph.h>
#include <rs/graph/GraphAlgorithms.hpp>

#include <rs/utils/array_utils.hpp>

#include <rs/visualization/Primitives.hpp>

using namespace uima;

class SymmetrySegmentationAnnotator : public DrawingAnnotator
{
private:
  struct Cluster
  {
    pcl::PointIndices indices;
    cv::Rect roi, roiHires;
    cv::Mat mask, maskHires;
  };

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr;
  pcl::PointCloud<pcl::Normal>::Ptr normals_ptr;

  cv::Mat rgb_;

  int numSegments;
  double pointSize;


  //OverSegmenter params
  float minNormalThreshold;
  float maxNormalThreshold;
  float curvatureThreshold;
  float overlapThreshold;
  int minClusterSize;
  int maxClusterSize;
  int neighborNumber;
  int numSegmentation;

  OverSegmenter segmenter;

  //RotationalSymmetryExtractor params
  float rotSymAnn_min_fit_angle;
  float rotSymAnn_max_fit_angle;
  float rotSymAnn_min_occlusion_dist;
  float rotSymAnn_max_occlusion_dist;
  float rotSymAnn_max_sym_score;
  float rotSymAnn_max_occlusion_score;
  float rotSymAnn_max_perpendicular_score;
  float rotSymAnn_min_coverage_score;
  float rotSymAnn_dist_map_resolution;
  float boundaryRadiusSearch;
  float boundaryAngleThreshold;
  float max_angle_diff;
  float max_dist_diff;

  RotationalSymmetryExtractor rot_extractor;

  //RotationalSymmetrySegmenter params
  int numSymmetries;
  bool rotSymSeg_isDownsampled;
  float downsample_leaf_size;
  float rotSymSeg_dist_map_resolution;
  float rotSymSeg_adjacency_radius;
  int rotSymSeg_num_adjacency_neighbors;
  float rotSymSeg_adjacency_sigma_convex;
  float rotSymSeg_adjacency_sigma_concave;
  float rotSymSeg_adjacency_weight_factor;
  float rotSymSeg_min_fit_angle;
  float rotSymSeg_max_fit_angle;
  float rotSymSeg_min_occlusion_dist;
  float rotSymSeg_max_occlusion_dist;
  float rotSymSeg_max_perpendicular_angle;
  float rotSymSeg_fg_weight_factor;
  float rotSymSeg_bg_weight_factor;
  float rotSymSeg_max_sym_score;
  float rotSymSeg_max_occlusion_score;
  float rotSymSeg_max_cut_score;
  int rotSymSeg_min_segment_size;
  float rotSymSeg_overlap_threshold;

  RotationalSymmetrySegmenter rot_segmenter;

  //BilateralSymmetryExtractor params
  bool bilSymAnn_isDownsampled;
  bool naive_detection;
  float bilSymAnn_downsample_voxel_size;
  int angle_division;
  float bilSymAnn_dist_map_resolution;
  float correspondence_search_radius;
  float correspondence_max_normal_fit_error;
  float correspondence_min_sym_dist;
  float bilSymAnn_correspondence_max_sym_reflected_dist;
  int refine_max_iteration;
  float refine_min_inlier_sym_score;
  float refine_max_inlier_sym_score;
  float bilSymAnn_min_occlusion_dist;
  float bilSymAnn_max_occlusion_dist;
  float bilSymAnn_max_occlusion_score;
  float min_segment_inlier_score;
  float min_corres_inlier_score;
  float sym_angle_diff;
  float sym_dist_diff;

  BilateralSymmetryExtractor bil_extractor;

  //BilateralSymmetrySegmenter params
  bool bilSymSeg_isDownsampled;
  float bilSymSeg_downsample_voxel_size;
  float bilSymSeg_dist_map_resolution;
  float bilSymSeg_adjacency_radius;
  int bilSymSeg_num_adjacency_neighbors;
  float bilSymSeg_adjacency_sigma_convex;
  float bilSymSeg_adjacency_sigma_concave;
  float bilSymSeg_adjacency_weight_factor;
  float bilSymSeg_min_fit_angle;
  float bilSymSeg_max_fit_angle;
  float bilSymSeg_min_occlusion_dist;
  float bilSymSeg_max_occlusion_dist;
  float bilSymSeg_min_perpendicular_angle;
  float bilSymSeg_max_perpendicular_angle;
  float bilSymSeg_correspondence_max_sym_reflected_dist;
  float symmetric_weight_factor;
  float bilSymSeg_fg_weight_factor;
  float bilSymSeg_bg_weight_factor;
  float bilSymSeg_max_sym_score;
  float bilSymSeg_max_occlusion_score;
  float bilSymSeg_max_cut_score;
  float min_sym_sypport_overlap;
  int bilSymSeg_min_segment_size;
  float bilSymSeg_overlap_threshold;

  BilateralSymmetrySegmenter bil_segmenter;

public:
  SymmetrySegmentationAnnotator () : DrawingAnnotator(__func__), pointSize(1.0)
  {
    cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    normals_ptr.reset(new pcl::PointCloud<pcl::Normal>());
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    //set params for OverSegmenter
    ctx.extractValue("minNormalThreshold", minNormalThreshold);
    ctx.extractValue("maxNormalThreshold", maxNormalThreshold);
    ctx.extractValue("overlapThreshold", overlapThreshold);
    ctx.extractValue("curvatureThreshold", curvatureThreshold);
    ctx.extractValue("minClusterSize", minClusterSize);
    ctx.extractValue("maxClusterSize", maxClusterSize);
    ctx.extractValue("neighborNumber", neighborNumber);
    ctx.extractValue("numSegmentation", numSegmentation);



    segmenter.initialize(minNormalThreshold,
                         maxNormalThreshold,
                         curvatureThreshold,
                         overlapThreshold,
                         minClusterSize,
                         maxClusterSize,
                         neighborNumber,
                         numSegmentation);

    //set params for RotationalSymmetryExtractor
    ctx.extractValue("rotSymAnn_min_fit_angle", rotSymAnn_min_fit_angle);
    ctx.extractValue("rotSymAnn_max_fit_angle", rotSymAnn_max_fit_angle);
    ctx.extractValue("rotSymAnn_min_occlusion_dist", rotSymAnn_min_occlusion_dist);
    ctx.extractValue("rotSymAnn_max_occlusion_dist", rotSymAnn_max_occlusion_dist);
    ctx.extractValue("rotSymAnn_max_sym_score", rotSymAnn_max_sym_score);
    ctx.extractValue("rotSymAnn_max_occlusion_score", rotSymAnn_max_occlusion_score);
    ctx.extractValue("rotSymAnn_max_perpendicular_score", rotSymAnn_max_perpendicular_score);
    ctx.extractValue("rotSymAnn_min_coverage_score", rotSymAnn_min_coverage_score);
    ctx.extractValue("rotSymAnn_dist_map_resolution", rotSymAnn_dist_map_resolution);
    ctx.extractValue("boundaryRadiusSearch", boundaryRadiusSearch);
    ctx.extractValue("boundaryAngleThreshold", boundaryAngleThreshold);
    ctx.extractValue("max_angle_diff", max_angle_diff);
    ctx.extractValue("max_dist_diff", max_dist_diff);

    rot_extractor.initialize(rotSymAnn_min_fit_angle,
                             rotSymAnn_max_fit_angle,
                             rotSymAnn_min_occlusion_dist,
                             rotSymAnn_max_occlusion_dist,
                             rotSymAnn_max_sym_score,
                             rotSymAnn_max_occlusion_score,
                             rotSymAnn_max_perpendicular_score,
                             rotSymAnn_min_coverage_score,
                             rotSymAnn_dist_map_resolution,
                             boundaryRadiusSearch,
                             boundaryAngleThreshold,
                             max_angle_diff,
                             max_dist_diff);

    //set params for RotationalSymmetrySegmenter
    ctx.extractValue("rotSymSeg_isDownsampled", rotSymSeg_isDownsampled);
    ctx.extractValue("downsample_leaf_size", downsample_leaf_size);
    ctx.extractValue("rotSymSeg_dist_map_resolution", rotSymSeg_dist_map_resolution);
    ctx.extractValue("rotSymSeg_adjacency_radius", rotSymSeg_adjacency_radius);
    ctx.extractValue("rotSymSeg_num_adjacency_neighbors", rotSymSeg_num_adjacency_neighbors);
    ctx.extractValue("rotSymSeg_adjacency_sigma_convex", rotSymSeg_adjacency_sigma_convex);
    ctx.extractValue("rotSymSeg_adjacency_sigma_concave", rotSymSeg_adjacency_sigma_concave);
    ctx.extractValue("rotSymSeg_adjacency_weight_factor", rotSymSeg_adjacency_weight_factor);
    ctx.extractValue("rotSymSeg_min_fit_angle", rotSymSeg_min_fit_angle);
    ctx.extractValue("rotSymSeg_max_fit_angle", rotSymSeg_max_fit_angle);
    ctx.extractValue("rotSymSeg_min_occlusion_dist", rotSymSeg_min_occlusion_dist);
    ctx.extractValue("rotSymSeg_max_occlusion_dist", rotSymSeg_max_occlusion_dist);
    ctx.extractValue("rotSymSeg_max_perpendicular_angle", rotSymSeg_max_perpendicular_angle);
    ctx.extractValue("rotSymSeg_fg_weight_factor", rotSymSeg_fg_weight_factor);
    ctx.extractValue("rotSymSeg_bg_weight_factor", rotSymSeg_bg_weight_factor);
    ctx.extractValue("rotSymSeg_max_sym_score", rotSymSeg_max_sym_score);
    ctx.extractValue("rotSymSeg_max_occlusion_score", rotSymSeg_max_occlusion_score);
    ctx.extractValue("rotSymSeg_max_cut_score", rotSymSeg_max_cut_score);
    ctx.extractValue("rotSymSeg_min_segment_size", rotSymSeg_min_segment_size);
    ctx.extractValue("rotSymSeg_overlap_threshold", rotSymSeg_overlap_threshold);

    rot_segmenter.initialize(rotSymSeg_isDownsampled,
                             downsample_leaf_size,
                             rotSymSeg_dist_map_resolution,
                             rotSymSeg_adjacency_radius,
                             rotSymSeg_num_adjacency_neighbors,
                             rotSymSeg_adjacency_sigma_convex,
                             rotSymSeg_adjacency_sigma_concave,
                             rotSymSeg_adjacency_weight_factor,
                             rotSymSeg_min_fit_angle,
                             rotSymSeg_max_fit_angle,
                             rotSymSeg_min_occlusion_dist,
                             rotSymSeg_max_occlusion_dist,
                             rotSymSeg_max_perpendicular_angle,
                             rotSymSeg_fg_weight_factor,
                             rotSymSeg_bg_weight_factor,
                             rotSymSeg_max_sym_score,
                             rotSymSeg_max_occlusion_score,
                             rotSymSeg_max_cut_score,
                             rotSymSeg_min_segment_size,
                             rotSymSeg_overlap_threshold);

    //set params for BilateralSymmetryExtractor
    ctx.extractValue("bilSymAnn_isDownsampled", bilSymAnn_isDownsampled);
    ctx.extractValue("naive_detection", naive_detection);
    ctx.extractValue("bilSymAnn_downsample_voxel_size", bilSymAnn_downsample_voxel_size);

    ctx.extractValue("angle_division", angle_division);

    ctx.extractValue("bilSymAnn_dist_map_resolution", bilSymAnn_dist_map_resolution);

    ctx.extractValue("correspondence_search_radius", correspondence_search_radius);
    ctx.extractValue("correspondence_max_normal_fit_error", correspondence_max_normal_fit_error);
    ctx.extractValue("correspondence_min_sym_dist", correspondence_min_sym_dist);
    ctx.extractValue("bilSymAnn_correspondence_max_sym_reflected_dist", bilSymAnn_correspondence_max_sym_reflected_dist);

    ctx.extractValue("refine_max_iteration", refine_max_iteration);
    ctx.extractValue("refine_min_inlier_sym_score", refine_min_inlier_sym_score);
    ctx.extractValue("refine_max_inlier_sym_score", refine_max_inlier_sym_score);

    ctx.extractValue("bilSymAnn_min_occlusion_dist", bilSymAnn_min_occlusion_dist);
    ctx.extractValue("bilSymAnn_max_occlusion_dist", bilSymAnn_max_occlusion_dist);

    ctx.extractValue("bilSymAnn_max_occlusion_score", bilSymAnn_max_occlusion_score);
    ctx.extractValue("min_segment_inlier_score", min_segment_inlier_score);
    ctx.extractValue("min_corres_inlier_score", min_corres_inlier_score);

    ctx.extractValue("sym_angle_diff", sym_angle_diff);
    ctx.extractValue("sym_dist_diff", sym_dist_diff);

    bil_extractor.initialize(bilSymAnn_isDownsampled,
                             naive_detection,
                             bilSymAnn_downsample_voxel_size,
                             angle_division,
                             bilSymAnn_dist_map_resolution,
                             correspondence_search_radius,
                             correspondence_max_normal_fit_error,
                             correspondence_min_sym_dist,
                             bilSymAnn_correspondence_max_sym_reflected_dist,
                             refine_max_iteration,
                             refine_min_inlier_sym_score,
                             refine_max_inlier_sym_score,
                             bilSymAnn_min_occlusion_dist,
                             bilSymAnn_max_occlusion_dist,
                             bilSymAnn_max_occlusion_score,
                             min_segment_inlier_score,
                             min_corres_inlier_score,
                             sym_angle_diff,
                             sym_dist_diff);



    //set params for BilateralSymmetrySegmenter
    ctx.extractValue("bilSymSeg_isDownsampled", bilSymSeg_isDownsampled);
    ctx.extractValue("bilSymSeg_downsample_voxel_size", bilSymSeg_downsample_voxel_size);
    ctx.extractValue("bilSymSeg_dist_map_resolution", bilSymSeg_dist_map_resolution);

    ctx.extractValue("bilSymSeg_adjacency_radius", bilSymSeg_adjacency_radius);
    ctx.extractValue("bilSymSeg_num_adjacency_neighbors", bilSymSeg_num_adjacency_neighbors);
    ctx.extractValue("bilSymSeg_adjacency_sigma_convex", bilSymSeg_adjacency_sigma_convex);
    ctx.extractValue("bilSymSeg_adjacency_sigma_concave", bilSymSeg_adjacency_sigma_concave);
    ctx.extractValue("bilSymSeg_adjacency_weight_factor", bilSymSeg_adjacency_weight_factor);

    ctx.extractValue("bilSymSeg_min_fit_angle", bilSymSeg_min_fit_angle);
    ctx.extractValue("bilSymSeg_max_fit_angle", bilSymSeg_max_fit_angle);
    ctx.extractValue("bilSymSeg_min_occlusion_dist", bilSymSeg_min_occlusion_dist);
    ctx.extractValue("bilSymSeg_max_occlusion_dist", bilSymSeg_max_occlusion_dist);
    ctx.extractValue("bilSymSeg_min_perpendicular_angle", bilSymSeg_min_perpendicular_angle);
    ctx.extractValue("bilSymSeg_max_perpendicular_angle", bilSymSeg_max_perpendicular_angle);
    ctx.extractValue("bilSymSeg_correspondence_max_sym_reflected_dist", bilSymSeg_correspondence_max_sym_reflected_dist);

    ctx.extractValue("symmetric_weight_factor", symmetric_weight_factor);
    ctx.extractValue("bilSymSeg_fg_weight_factor", bilSymSeg_fg_weight_factor);
    ctx.extractValue("bilSymSeg_bg_weight_factor", bilSymSeg_bg_weight_factor);

    ctx.extractValue("bilSymSeg_max_sym_score", bilSymSeg_max_sym_score);
    ctx.extractValue("bilSymSeg_max_occlusion_score", bilSymSeg_max_occlusion_score);
    ctx.extractValue("bilSymSeg_max_cut_score", bilSymSeg_max_cut_score);
    ctx.extractValue("min_sym_sypport_overlap", min_sym_sypport_overlap);
    ctx.extractValue("bilSymSeg_min_segment_size", bilSymSeg_min_segment_size);

    ctx.extractValue("bilSymSeg_overlap_threshold", bilSymSeg_overlap_threshold);

    bil_segmenter.initialize(bilSymSeg_isDownsampled,
                             bilSymSeg_downsample_voxel_size,
                             bilSymSeg_dist_map_resolution,
                             bilSymSeg_adjacency_radius,
                             bilSymSeg_num_adjacency_neighbors,
                             bilSymSeg_adjacency_sigma_convex,
                             bilSymSeg_adjacency_sigma_concave,
                             bilSymSeg_adjacency_weight_factor,
                             bilSymSeg_min_fit_angle,
                             bilSymSeg_max_fit_angle,
                             bilSymSeg_min_occlusion_dist,
                             bilSymSeg_max_occlusion_dist,
                             bilSymSeg_min_perpendicular_angle,
                             bilSymSeg_max_perpendicular_angle,
                             bilSymSeg_correspondence_max_sym_reflected_dist,
                             symmetric_weight_factor,
                             bilSymSeg_fg_weight_factor,
                             bilSymSeg_bg_weight_factor,
                             bilSymSeg_max_sym_score,
                             bilSymSeg_max_occlusion_score,
                             bilSymSeg_max_cut_score,
                             min_sym_sypport_overlap,
                             bilSymSeg_min_segment_size,
                             bilSymSeg_overlap_threshold);

    srand (time(NULL));

    numSegments = 0;
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;

    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    /*
     * Input routines. Requirements: non-NaN cloud and normals, ids to transform back to original cloud
     */

    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *normals_ptr);

    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);

    std::vector<int> cloudIds(cloud_ptr->size());
    for(size_t pointId = 0; pointId < cloud_ptr->size(); pointId++)
    {
      cloudIds[pointId] = pointId;
    }

    outInfo("Cloud size: " << cloud_ptr->size());
    outInfo("Normals size: " << normals_ptr->size());

    //filter plane indices out
    pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
    for(size_t planeId = 0; planeId < planes.size(); planeId++)
    {
      std::vector<int> currIds = planes[planeId].inliers();
      plane_indices->indices.insert(plane_indices->indices.end(), currIds.begin(), currIds.end());
    }

    std::vector<int> reverse_plane_mapping = Difference(cloudIds, plane_indices->indices);

    pcl::ExtractIndices<pcl::PointXYZRGBA> extractCloud;
    pcl::ExtractIndices<pcl::Normal> extractNormal;

    extractCloud.setInputCloud(cloud_ptr);
    extractNormal.setInputCloud(normals_ptr);

    extractCloud.setIndices(plane_indices);
    extractNormal.setIndices(plane_indices);

    extractCloud.setNegative(true);
    extractNormal.setNegative(true);

    extractCloud.filter(*cloud_ptr);
    extractNormal.filter(*normals_ptr);

    outInfo("Object cloud size: " << cloud_ptr->size());
    outInfo("Object normal size: " << normals_ptr->size());

    std::vector<int> reverse_non_nan_mapping;

    pcl::removeNaNNormalsFromPointCloud(*normals_ptr, *normals_ptr, reverse_non_nan_mapping);
    outInfo("Object cloud size after filter: " << normals_ptr->size());
    pcl::copyPointCloud(*cloud_ptr, reverse_non_nan_mapping, *cloud_ptr);

    cas.get(VIEW_COLOR_IMAGE, rgb_);

    /*
     * First OverSegmenter process
     */
    segmenter.setInputClouds(cloud_ptr, normals_ptr);

    //main execution
    segmenter.segment();

    std::vector<pcl::PointIndices> over_segments;
    segmenter.getSegments(over_segments);

    /*
     * RotationalSymmetryExtractor process
     */
    rot_extractor.setInputClouds(cloud_ptr, normals_ptr);
    rot_extractor.setInputSegmentIds(over_segments);
    rot_extractor.setInputPlanes(planes);

    //main execution
    rot_extractor.extract();

    std::vector<RotationalSymmetry> rot_symmetries;
    rot_extractor.getSymmetries(rot_symmetries);

    /*
     * RotationalSymmetrySegmenter process
     */
    rot_segmenter.setInputClouds(cloud_ptr, normals_ptr);
    rot_segmenter.setInputSymmetries(rot_symmetries);

    //main execution
    rot_segmenter.segment();

    std::vector< std::vector<int> > rot_segmentIds;
    rot_segmenter.getSegmentIds(rot_segmentIds);

    /*
     * Second OverSegmenter process
     */

    //remove plane if there are any planes
    segmenter.removeSegments(rot_segmentIds);

    //main execution
    segmenter.segment();

    segmenter.getSegments(over_segments);

    /*
     * BilateralSymmetryExtractor process
     */
    bil_extractor.setInputClouds(cloud_ptr, normals_ptr);
    bil_extractor.setInputSegmentIds(over_segments);

    //main execution
    bil_extractor.extract();

    std::vector< BilateralSymmetry > bil_symmetries;
    std::vector< int > bil_supportSizeIds;
    bil_extractor.getSymmetries(bil_symmetries);
    bil_extractor.getSupportIds(bil_supportSizeIds);

    std::vector< std::vector<int> > bil_symSupports;
    for(size_t symId = 0; symId < bil_symmetries.size(); symId++)
    {
      bil_symSupports.push_back(over_segments[bil_supportSizeIds[symId]].indices);
    }

    /*
     * BilateralSymmetrySegmenter process
     */
    bil_segmenter.setInputClouds(cloud_ptr, normals_ptr);
    bil_segmenter.setInputSymmetries(bil_symmetries, bil_symSupports);
    bil_segmenter.setInputPlanes(planes);

    //main execution
    bil_segmenter.segment();

    std::vector< std::vector<int> > bil_segmentIds;
    bil_segmenter.getSegmentIds(bil_segmentIds);

    /*
     * Process visualization and publish to CAS
     */
    numSegments = rot_segmentIds.size() + bil_segmentIds.size();

    std::vector< std::vector<int> > segmentIds;
    segmentIds.insert(segmentIds.end(), rot_segmentIds.begin(), rot_segmentIds.end());
    segmentIds.insert(segmentIds.end(), bil_segmentIds.begin(), bil_segmentIds.end());


    //set all point in cloud to gray
    for(size_t pointId = 0; pointId < cloud_ptr->size(); pointId++)
    {
      cloud_ptr->points[pointId].r = 30;
      cloud_ptr->points[pointId].g = 30;
      cloud_ptr->points[pointId].b = 30;
    }

    for(size_t segmentId = 0; segmentId < segmentIds.size(); segmentId++)
    {
      int r = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));
      int g = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));
      int b = (255 / (numSegments + 4)) * (rand() % (numSegments + 4));

      pcl::PointIndices original_indices;

      for(size_t pointIdIt = 0; pointIdIt < segmentIds[segmentId].size(); pointIdIt++)
      {
        int pointId = segmentIds[segmentId][pointIdIt];
        cloud_ptr->points[pointId].r = r;
        cloud_ptr->points[pointId].g = g;
        cloud_ptr->points[pointId].b = b;

        int original_index = reverse_plane_mapping[reverse_non_nan_mapping[pointId]];
        original_indices.indices.push_back(original_index);
        cv::Point current = indexToCoordinates(original_index, rgb_);

        rgb_.at<cv::Vec3b>(current)[0] = r;
        rgb_.at<cv::Vec3b>(current)[1] = g;
        rgb_.at<cv::Vec3b>(current)[2] = b;
      }

      //publish Clusters to CAS
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


    return UIMA_ERR_NONE;
  }

  bool callbackKey(const int key, const Source source)
  {

  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    const std::string text = "segment_results";

    if(firstRun)
    {
      visualizer.addPointCloud(cloud_ptr, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.addText("Total Segment " + std::to_string(numSegments), 15, 125, 24, 1.0, 1.0, 1.0, text);
    }
    else
    {
      visualizer.updatePointCloud(cloud_ptr, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeShape(text);
      visualizer.addText("Total Segment " + std::to_string(numSegments), 15, 125, 24, 1.0, 1.0, 1.0, text);
    }
  }

  void drawImageWithLock(cv::Mat& disp)
  {
    disp=rgb_.clone();
  }

  cv::Point indexToCoordinates(int index, cv::Mat& rgb)
  {
	   int row = index / rgb.size().width + 1;
	   int col = index % rgb.size().width + 1;

     return cv::Point(col, row);
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

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SymmetrySegmentationAnnotator)
