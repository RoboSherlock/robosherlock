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

#ifndef __ROTATIONAL_SYMMETRY_ANNOTATOR_HPP__
#define __ROTATIONAL_SYMMETRY_ANNOTATOR_HPP__

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <rs/segmentation/Geometry.hpp>
#include <rs/utils/output.h>

/** \brief Class representing a rotational symmetry in 3D space. A symmetry
 * is represented as a 3D axis.
 */
class RotationalSymmetry
{
private:
  Eigen::Vector3f origin;
  Eigen::Vector3f orientation;

public:

  /** \brief Default constructor. */
  RotationalSymmetry() : origin(Eigen::Vector3f::Zero()), orientation(Eigen::Vector3f::Zero()) {}

  /** \brief Constructor to initialize origin point and symmetry axis. */
  RotationalSymmetry(const Eigen::Vector3f &point, const Eigen::Vector3f &vec) : origin(point), orientation(vec.normalized()) {}

  /** \brief Destructor. */
  ~RotationalSymmetry() {}

  /** \brief Get origin point. */
  Eigen::Vector3f getOrigin() const
  {
    return origin;
  }

  /** \brief Get orientation axis. */
  Eigen::Vector3f getOrientation() const
  {
    return orientation;
  }

  /** \brief Set origin point.
   *  \param[in]  point   input origin
   */
  void setOrigin(Eigen::Vector3f &point)
  {
    origin = point;
  }

  /** \brief Set projected point to symmetry line from input point as symmetry origin.
   *  \param[in]  point   input point
   */
  void setProjectedOrigin(Eigen::Vector3f &point)
  {
    origin = projectPoint(point);
  }

  /** \brief Set orientation.
   *  \param[in]  vec  input axis
   */
  void setOrientation(Eigen::Vector3f &vec)
  {
    orientation = vec.normalized();
  }

  /** \brief Project point to the symmetry line.
   *  \param[in]  point   input point
   *  \return projected point on the symmetry line
   */
  Eigen::Vector3f projectPoint(const Eigen::Vector3f &point)
  {
    return pointToLineProjection<float>(point, origin, origin + orientation);
  }

  /** \brief compute distance from point to symmetry line.
   *  \param[in]  point   input point
   *  \return distance
   */
  float pointDistance(const Eigen::Vector3f &point)
  {
    return pointToLineNorm<float>(point, origin, origin + orientation);
  }

  /** \brief get rotational matrix around symmetry axis
   *  \param[in]  angle   rotational angle
   *  \return 3x3 rotational matrix
   */
  Eigen::Matrix3f getRotationMatrix(float angle)
  {
    return Eigen::AngleAxisf(angle, orientation).toRotationMatrix();
  }

  /** \brief rotate point around symmetry axis
   *  \param[in]  point   input point
   *  \param[in]  angle   rotational angle
   *  \return rotated point
   */
  Eigen::Vector3f rotatePoint(const Eigen::Vector3f &point, float angle)
  {
    Eigen::Vector3f projectedPoint = projectPoint(point);
    return projectedPoint + getRotationMatrix(angle) * (point - projectedPoint);
  }

  /** \brief rotate cloud around symmetry axis
   *  \param[in]   cloud_in   input cloud
   *  \param[out]  cloud_out  rotated cloud
   *  \param[in]   angle      rotational angle
   */
  template<typename PointT>
  void rotateCloud(typename pcl::PointCloud<PointT> &cloud_in, typename pcl::PointCloud<PointT> &cloud_out, float angle)
  {
    pcl::copyPointCloud(cloud_in, cloud_out);

    for(size_t it = 0; it < cloud_in.points.size(); it++)
    {
      cloud_out.points[it].getVector3fMap() = rotatePoint(cloud_in.points[it].getVector3fMap(), angle);
    }
  }

  /** \brief rotate cloud around symmetry axis
   *  \param[in]   cloud_in   input cloud
   *  \param[in]   indices    input cloud indices
   *  \param[out]  cloud_out  rotated cloud
   *  \param[in]   angle      rotational angle
   */
  template<typename PointT>
  void rotateCloud(typename pcl::PointCloud<PointT>& cloud_in, std::vector<int> &indices, typename pcl::PointCloud<PointT> &cloud_out, float angle)
  {
    pcl::copyPointCloud(cloud_in, indices, cloud_out);

    for(size_t it = 0; it < indices.size(); it++)
    {
      cloud_out.points[indices[it]].getVector3fMap() = rotatePoint(cloud_in.points[indices[it]].getVector3fMap(), angle);
    }
  }

  /** \brief generate redundant cloud around symmetry axis by accumulating rotated cloud
   *  \param[in]   cloud_in   input cloud
   *  \param[in]   indices    input cloud indices
   *  \param[out]  cloud_out  rotated cloud
   *  \param[in]   step_angle
   */
  template<typename PointT>
  inline void populateCloud(typename pcl::PointCloud<PointT> &cloud_in,
                            std::vector<int> &indices,
                            typename pcl::PointCloud<PointT> &cloud_out,
                            float step_angle = M_PI / 4)
  {
    cloud_out.clear();
    int redundant_factor = static_cast<int>(M_PI * 2 / step_angle) + 1;

    pcl::PointCloud<PointT> rotatedCloud;
    for(size_t it = 0;  it < redundant_factor; it++)
    {
      rotateCloud(cloud_in, indices, rotatedCloud, step_angle * it);
      cloud_out += rotatedCloud;
    }
  }

  /** \brief generate redundant cloud around symmetry axis by accumulating rotated cloud
   *  \param[in]   cloud_in   input cloud
   *  \param[out]  cloud_out  rotated cloud
   *  \param[in]   step_angle
   */
  template<typename PointT>
  inline void populateCloud(typename pcl::PointCloud<PointT> &cloud_in,
                            typename pcl::PointCloud<PointT> &cloud_out,
                            float step_angle = M_PI / 4)
  {
    std::vector<int> indices(cloud_in.points.size());
    for(size_t it = 0; it < cloud_in.points.size(); it++)
    {
      indices[it] = it;
    }

    populateCloud(cloud_in, indices, cloud_out, step_angle);
  }

  /** \brief get angle and distance difference between this symmetry and target symmetry
   *  \param[in]   target      target symmetry
   *  \param[out]  angle       angle difference
   *  \param[out]  dist        distance difference
   */
  inline void getRotSymDifference(const RotationalSymmetry &target, float &angle, float &dist)
  {
    angle = lineLineAngle<float>(orientation, target.getOrientation());
    dist = lineToLineNorm<float>(origin, origin + orientation, target.getOrigin(), target.getOrigin() + target.getOrientation());
  }
};

/** \brief get angle error between perpendicular line of point to axis and symmetry axis
 *  \param[in]  point    input point
 *  \param[in]  normal   input normal
 *  \param[in]  symmetry RotationalSymmetry
 *  \return angle error
 */
inline float getRotSymFitError(Eigen::Vector3f &point,
                               Eigen::Vector3f &normal,
                               RotationalSymmetry &symmetry)
{
  Eigen::Vector3f projectedPoint = symmetry.projectPoint(point);
  Eigen::Vector3f planeNormal = (point - projectedPoint).cross(symmetry.getOrientation());

  float angle = std::abs(planeNormal.dot(normal) / planeNormal.norm());
  angle = clamp(angle, 0.0f, 1.0f);

  return std::asin(angle);
}

/** \brief calculate scaled angle error between point normal and symmetry axis
 *  \param[in]  normal    input normal
 *  \param[in]  symmetry  RotationalSymmetry
 *  \param[in]  threshold denominator to scale
 */
inline float getRotSymPerpendicularity(Eigen::Vector3f &normal,
                                       RotationalSymmetry &symmetry,
                                       float threshold = M_PI / 2)
{
  if(threshold == 0.0)
  {
    return 1.0f;
  }

  float angle = lineLineAngle(symmetry.getOrientation(), normal);

  angle /= threshold;
  angle = std::min(angle, 1.0f);

  return 1.0f - angle;
}

#endif // __ROTATIONAL_SYMMETRY_ANNOTATOR_HPP__
