/* Copyright (c) 2012, Ferenc Balint-Benczdei<balintbe@cs.uni-bremen.de>
 * Based on the handle extractor from Ross Kidson <ross.kidson@gmail.com>
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

#define DEBUG

#include <rs_queryanswering/annotators/HandleExtractor.h>
namespace rs
{

void HandleExtractor::config()
{
  line_ransac_distance = 0.01;
  line_ransac_max_iter = 1000;
  handle_cluster_tolerance = 0.03;
  min_handle_cluster_size = 600;
  minimum_plane_size = 10000;
  plane_segmentation_max_curvature = 0.02;
  min_handle_height = 0.03;
  max_handle_height = 0.10;
  //radius search for neighbors to consider when calculating normals
  normal_calculation_kRadius = 0.03;
}

pcl::PointIndicesConstPtr HandleExtractor::getIndicesPointerFromObject(const pcl::PointIndices &input)
{
  pcl::PointIndicesPtr output_ptr(new pcl::PointIndices);
  for(std::vector<int>::const_iterator itr = input.indices.begin(); itr != input.indices.end(); itr++)
  {
    output_ptr->indices.push_back(*itr);
  }
  return output_ptr;
}

/**
 * extract handles from a pointcloud
 * @param[pointCloudIn] input point cloud
 * @param[pointCloudNormals] normals for the input cloud
 * @param[handle_indices] vector of indices, each item being a set of indices representing a handle
 * @param[handle_coefficients] vector of cofficients, each item being the coefficients of the line representing the handle
 */
void HandleExtractor::extractHandles(PointCloud::Ptr &cloudInput, PointCloudNormal::Ptr &pointCloudNormals,
                                     std::vector< pcl::PointIndices> &handle_indices, std::vector< pcl::ModelCoefficients> &handle_coefficients
                                    )
{

  // setup handle cluster object
  pcl::EuclideanClusterExtraction<Point> handle_cluster_;
  KdTreePtr clusters_tree_(new KdTree);
  handle_cluster_.setClusterTolerance(handle_cluster_tolerance);
  handle_cluster_.setMinClusterSize(min_handle_cluster_size);
  handle_cluster_.setSearchMethod(clusters_tree_);
  handle_cluster_.setInputCloud(cloudInput);

  pcl::PointCloud<Point>::Ptr cloud_filtered(new pcl::PointCloud<Point>());
  pcl::VoxelGrid<Point> vg;
  vg.setInputCloud(cloudInput);
  vg.setLeafSize(0.01, 0.01, 0.01);
  vg.filter(*cloud_filtered);

  // setup line ransac object
  pcl::SACSegmentation<Point> seg_line_;
  seg_line_.setModelType(pcl::SACMODEL_LINE);
  seg_line_.setMethodType(pcl::SAC_RANSAC);
  seg_line_.setInputCloud(cloudInput);
  seg_line_.setDistanceThreshold(line_ransac_distance);
  seg_line_.setOptimizeCoefficients(true);
  seg_line_.setMaxIterations(line_ransac_max_iter);

  // setup Inlier projection object
  pcl::ProjectInliers<Point> proj_;
  proj_.setInputCloud(cloud_filtered);
  proj_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);

  // setup polygonal prism
  pcl::ExtractPolygonalPrismData<Point> prism_;
  prism_.setHeightLimits(min_handle_height, max_handle_height);
  prism_.setInputCloud(cloudInput);

  //setup Plane Segmentation
  outInfo("Segmenting planes");
  pcl::SACSegmentation<Point> seg_plane_;
  seg_plane_.setOptimizeCoefficients(true);
  seg_plane_.setModelType(pcl::SACMODEL_PLANE);
  seg_plane_.setMethodType(pcl::SAC_RANSAC);
  seg_plane_.setDistanceThreshold(0.03);
  seg_plane_.setMaxIterations(500);
  seg_plane_.setInputCloud(cloud_filtered);
  pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
  seg_plane_.segment(*plane_inliers, *plane_coefficients);

  if(plane_inliers->indices.size() != 0)
  {
    outDebug("Plane model: "
             << plane_coefficients->values[0] << ","
             << plane_coefficients->values[1] << ","
             << plane_coefficients->values[2] << ","
             << plane_coefficients->values[3] << " with "
             << (int)plane_inliers->indices.size() << "inliers ");

    //Project inliers of the planes into a perfect plane
    PointCloud::Ptr cloud_projected(new PointCloud());
    proj_.setIndices(plane_inliers);
    proj_.setModelCoefficients(plane_coefficients);
    proj_.filter(*cloud_projected);

    // Create a Convex Hull representation using the projected inliers
    PointCloud::Ptr cloud_hull(new PointCloud());
    pcl::ConvexHull<Point> chull_;
    chull_.setDimension(2);
    chull_.setInputCloud(cloud_projected);
    chull_.reconstruct(*cloud_hull);
    outInfo("Convex hull has: " << (int)cloud_hull->points.size() << " data points.");
    if((int) cloud_hull->points.size() == 0)
    {
      outInfo("Convex hull has: no data points. Returning.");
      return;
    }

    // Extract the handle clusters using a polygonal prism
    pcl::PointIndices::Ptr handlesIndicesPtr(new pcl::PointIndices());
    prism_.setInputPlanarHull(cloud_hull);
    prism_.segment(*handlesIndicesPtr);

    // cluster the points found in the prism
    std::vector< pcl::PointIndices> handle_clusters;
    handle_cluster_.setIndices(handlesIndicesPtr);
    handle_cluster_.extract(handle_clusters);
    for(size_t j = 0; j < handle_clusters.size(); j++)
    {
      // for each cluster in the prism, attempt to fit a line using ransac
      pcl::PointIndices single_handle_indices;
      pcl::ModelCoefficients handle_line_coefficients;
      seg_line_.setIndices(getIndicesPointerFromObject(handle_clusters[j]));
      seg_line_.segment(single_handle_indices, handle_line_coefficients);
      if(single_handle_indices.indices.size() > 0)
      {
        outInfo("Found a handle with " << single_handle_indices.indices.size() << " inliers.");
        outDebug("Handle Line coefficients: " << handle_line_coefficients);
        handle_indices.push_back(single_handle_indices);
        handle_coefficients.push_back(handle_line_coefficients);
      }
    }
  }
  else
  {
    outInfo("no planes found");
    return;
  }
}

}
