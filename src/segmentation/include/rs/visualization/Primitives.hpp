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

#ifndef __PRIMITIVES_HPP__
#define __PRIMITIVES_HPP__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rs/segmentation/BilateralSymmetry.hpp>
#include <rs/segmentation/RotationalSymmetry.hpp>

#include <rs/segmentation/Geometry.hpp>

inline void addSymmetryPlane(pcl::visualization::PCLVisualizer &visualizer, BilateralSymmetry &symmetry, std::string &id, float width, float height)
{
  Eigen::Affine3f pose;
  pose.translation() = symmetry.getOrigin();
  pose.linear() = getAlignMatrix<float>(Eigen::Vector3f::UnitZ(), symmetry.getNormal());

  float halfWidth = width / 2.0f;
  float halfHeight = height / 2.0f;

  pcl::PointCloud<pcl::PointXYZ>::Ptr rect(new pcl::PointCloud<pcl::PointXYZ>);
  rect->resize(4);
  rect->points[0] = pcl::PointXYZ(-halfWidth, -halfHeight, 0);
  rect->points[1] = pcl::PointXYZ(halfWidth, -halfHeight, 0);
  rect->points[2] = pcl::PointXYZ(halfWidth, halfHeight, 0);
  rect->points[3] = pcl::PointXYZ(-halfWidth, halfHeight, 0);

  pcl::transformPointCloud<pcl::PointXYZ>(*rect, *rect, pose);

  visualizer.addPolygon<pcl::PointXYZ>(rect, id);
  visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
  visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, id);
  visualizer.addPolygon<pcl::PointXYZ>(rect, id + "_border");
  visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id + "_border");
}

inline void addSymmetryPlanes(pcl::visualization::PCLVisualizer &visualizer, std::vector< std::vector<BilateralSymmetry> > &symmetries, float width, float height)
{
  for(size_t segmentId = 0; segmentId < symmetries.size(); segmentId++)
  {
    for(size_t symId = 0; symId < symmetries[segmentId].size(); symId++)
    {
      std::string id = "BilSym" + std::to_string(segmentId * symmetries[segmentId].size() + symId);
      addSymmetryPlane(visualizer, symmetries[segmentId][symId], id, width, height);
    }
  }
}

inline void addSymmetryPlanes(pcl::visualization::PCLVisualizer &visualizer, std::vector<BilateralSymmetry> &symmetries, float width, float height)
{
  for(size_t symId = 0; symId < symmetries.size(); symId++)
  {
    std::string id = "BilSym" + std::to_string(symId);
    addSymmetryPlane(visualizer, symmetries[symId], id, width, height);
  }
}

inline void addSymmetryLine(pcl::visualization::PCLVisualizer& visualizer, RotationalSymmetry &symmetry, std::string &id, float length, float lineWidth){
  pcl::PointXYZ p1, p2;
  p1.getVector3fMap() = symmetry.getOrigin() + symmetry.getOrientation() * length / 2;
  p2.getVector3fMap() = symmetry.getOrigin() - symmetry.getOrientation() * length / 2;

  visualizer.addLine(p1, p2, id);
  visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, id);
  visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, id);
}


inline void addSymmetryLines(pcl::visualization::PCLVisualizer& visualizer, std::vector< std::vector<RotationalSymmetry> >& symmetries, float length, float lineWidth){
  for(size_t segmentId = 0; segmentId < symmetries.size(); segmentId++)
  {
    for(size_t symId = 0; symId < symmetries[segmentId].size(); symId++){
      std::string symname = "RotSym" + std::to_string(segmentId * symmetries[segmentId].size() + symId);
      addSymmetryLine(visualizer, symmetries[segmentId][symId], symname, length, lineWidth);
    }
  }
}

inline void addSymmetryLines(pcl::visualization::PCLVisualizer& visualizer, std::vector<RotationalSymmetry>& symmetries, float length, float lineWidth){
  for(size_t symId = 0; symId < symmetries.size(); symId++){
    std::string symname = "RotSym" + std::to_string(symId);
    addSymmetryLine(visualizer, symmetries[symId], symname, length, lineWidth);
  }
}

#endif // __PRIMITIVES_HPP__
