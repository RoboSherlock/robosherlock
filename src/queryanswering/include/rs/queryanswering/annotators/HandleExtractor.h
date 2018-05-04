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

#include <iostream>

// pcl

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/tf.h>

#include<rs/utils/output.h>

namespace rs
{

class HandleExtractor
{

public:

  typedef pcl::PointXYZRGBA Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef pcl::Normal PointNormal;
  typedef pcl::PointCloud<PointNormal> PointCloudNormal;
  typedef pcl::search::KdTree<Point> KdTree;
  typedef KdTree::Ptr KdTreePtr;

  HandleExtractor()
  {
    config();
  }
  ~HandleExtractor()
  {
  }
  void config();

  void extractHandles(PointCloud::Ptr& cloudInput, PointCloudNormal::Ptr& pointCloudNormals,
                      std::vector<pcl::PointIndices>& handle_indices,
                      std::vector<pcl::ModelCoefficients>& handle_coefficients);
private:
  double line_ransac_distance;
  int line_ransac_max_iter;

  double handle_cluster_tolerance;
  double min_handle_cluster_size;

  int minimum_plane_size;
  double plane_segmentation_max_curvature;

  double min_handle_height;
  double max_handle_height;

  //radius search for neighbors to consider when calculating normals
  double normal_calculation_kRadius;

  pcl::PointIndicesConstPtr getIndicesPointerFromObject(const pcl::PointIndices &);

};
}
