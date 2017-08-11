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

#ifndef __GEOMETRY_HPP__
#define __GEOMETRY_HPP__

#include <eigen3/Eigen/Dense>
#include <cmath>

#include <rs/utils/output.h>

#include <pcl/search/kdtree.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

template<typename Type>
inline Type clamp(Type val, Type minVal, Type maxVal)
{
  Type result = val;
  result = std::max(minVal, result);
  result = std::min(maxVal, result);
  return result;
}

template<class Type>
inline Type pointToPointNorm(const Eigen::Matrix<Type, 3, 1> &point1, const Eigen::Matrix<Type, 3, 1> &point2)
{
  return (point2 - point1).norm();
}

template<class Type>
inline Type pointToLineNorm(const Eigen::Matrix<Type, 3, 1> &point, const Eigen::Matrix<Type, 3, 1> &linePoint1, const Eigen::Matrix<Type, 3, 1> &linePoint2)
{
  return (point - linePoint1).cross(point - linePoint2).norm() / (linePoint2 - linePoint1).norm();
}

template<class Type>
inline Eigen::Matrix<Type, 3, 1> pointToLineProjection(const Eigen::Matrix<Type, 3, 1> &point, const Eigen::Matrix<Type, 3, 1> &linePoint1, const Eigen::Matrix<Type, 3, 1> &linePoint2)
{
  Eigen::Vector3f line = linePoint2 - linePoint1;
  return linePoint1 + (point - linePoint1).dot(line) * line / line.norm();
}

template<class Type>
inline void planeToPointNormal(const Eigen::Matrix<Type, 4, 1> &plane, Eigen::Matrix<Type, 3, 1> &point, Eigen::Matrix<Type, 3, 1> &normal)
{
  normal << plane[0], plane[1], plane[2];
  Type denom = normal.norm();
  normal /= denom; // normalize plane normal
  point = normal * (- plane[3] / denom);
}

template<class Type>
inline void pointNormalToPlane(const Eigen::Matrix<Type, 3, 1> &point, const Eigen::Matrix<Type, 3, 1> &normal, Eigen::Matrix<Type, 4, 1> &plane)
{
  plane.head(3) = normal;
  plane(3) = -normal.dot(point);
}

template<class Type>
inline Type pointToPlaneSignedNorm(const Eigen::Matrix<Type, 3, 1> &point, const Eigen::Matrix<Type, 3, 1> &planePoint, const Eigen::Matrix<Type, 3, 1> &planeNormal)
{
  Eigen::Matrix<Type, 3, 1> line = point - planePoint;
  return line.dot(planeNormal);
}

template<class Type>
inline Type pointToPlaneSignedNorm(const Eigen::Matrix<Type, 3, 1> &point, const Eigen::Matrix<Type, 4, 1> &plane)
{
  Eigen::Vector3f planePoint, planeNormal;
  planeToPointNormal(plane, planePoint, planeNormal);
  return pointToPlaneSignedNorm(point, planePoint, planeNormal);
}

template<class Type>
inline Eigen::Matrix<Type, 3, 1> pointToPlaneProjection(const Eigen::Matrix<Type, 3, 1> &point, const Eigen::Matrix<Type, 3, 1> &planePoint, const Eigen::Matrix<Type, 3, 1> &planeNormal)
{
  Eigen::Matrix<Type, 3, 1> line = point - planePoint;
  return point - planeNormal * pointToPlaneSignedNorm(point, planePoint, planeNormal);
}

template<typename PointT>
inline void cloudToPlaneProjection(typename pcl::PointCloud<PointT>::Ptr &cloud_in, const Eigen::Vector3f &planePoint, const Eigen::Vector3f &planeNormal, typename pcl::PointCloud<PointT>::Ptr &cloud_out)
{
  cloud_out.reset(new pcl::PointCloud<PointT>);
  pcl::copyPointCloud(*cloud_in, *cloud_out);
  for(size_t pointId = 0; pointId < cloud_in->points.size(); pointId++)
  {
    cloud_out->points[pointId].getVector3fMap() = pointToPlaneProjection<float>(cloud_in->points[pointId].getVector3fMap(), planePoint, planeNormal);
  }
}

template<class Type>
inline Type lineToLineNorm(const Eigen::Matrix<Type, 3, 1> &line1Point1,
                           const Eigen::Matrix<Type, 3, 1> &line1Point2,
                           const Eigen::Matrix<Type, 3, 1> &line2Point1,
                           const Eigen::Matrix<Type, 3, 1> &line2Point2,
                           const Type eps = 1e-10){
  Eigen::Matrix<Type, 3, 1> line1 = line1Point1 - line1Point2;
  Eigen::Matrix<Type, 3, 1> line2 = line2Point1 - line2Point2;

  Type denom = line1.cross(line2).norm();
  if(denom < eps)
  {
    return pointToLineNorm(line1Point1, line2Point1, line2Point2);
  }

  Eigen::Matrix<Type, 3, 1> line3 = line1Point1 - line2Point1;
  return std::abs(line3.dot(line1.cross(line2))) / denom;
}

template<class Type>
inline Type vecVecAngle(const Eigen::Matrix<Type, 3, 1> &v1,
                        const Eigen::Matrix<Type, 3, 1> &v2)
{
  v1.normalized();
  v2.normalized();

  return std::acos(clamp(v1.dot(v2), -1.0f, 1.0f));
}

//vecter vector angle clockwise
template<class Type>
inline Type vecVecAngleClockwise(const Eigen::Matrix<Type, 3, 1> &v1,
                                 const Eigen::Matrix<Type, 3, 1> &v2,
                                 const Eigen::Matrix<Type, 3, 1> &normal)
{
  v1.normalized();
  v2.normalized();
  Type denom = v1.dot(v2);
  Type nom = clamp(normal.dot(v1.cross(v2)), -1.0f, 1.0f);

  return std::atan2(nom, denom);
}

template<class Type>
inline Type lineLineAngle(const Eigen::Matrix<Type, 3, 1> &v1,
                          const Eigen::Matrix<Type, 3, 1> &v2)
{
  return std::abs(vecVecAngle<Type>(v1, v2));
}

//This function return bounded angle in 2 * PI
template<class Type>
inline Type angleDifferent(const Type angle1, const Type angle2)
{
  Type result = angle2 - angle1;
  if(result < 0)
  {
    result += 2 * M_PI;
  }

  return result;
}

inline bool generateHemisphere (const int division, std::vector<Eigen::Vector3f> &points)
{
  points.clear();

  if(division < 2)
  {
    return false;
  }

  const float step = M_PI / division;
  bool polarPoint = (division % 2 == 0);

  if(polarPoint)
  {
    points.push_back(Eigen::Vector3f::UnitZ());
  }

  Eigen::Matrix3f rotation;
  std::vector<Eigen::Vector3f> equator;
  equator.push_back(Eigen::Vector3f::UnitX());
  for(size_t it = 1; it < division; it++)
  {
    rotation = Eigen::AngleAxisf(static_cast<float>(step * it), Eigen::Vector3f::UnitZ());
    equator.push_back(rotation * equator[0]);
  }
  points.insert(points.end(), equator.begin(), equator.end());

  Eigen::Vector3f rotAxis;
  for(size_t it = 1; it < division; it++)
  {
    if(polarPoint && it * 2 == division)
    {
      continue;
    }

    for(size_t equatorId = 0; equatorId < equator.size(); equatorId++)
    {
      rotAxis = equator[equatorId].cross(Eigen::Vector3f::UnitZ());
      rotation = Eigen::AngleAxisf(static_cast<float>(step * it), rotAxis);
      points.push_back(rotation * equator[equatorId]);
    }
  }

  return true;
}

template<typename Type>
inline Eigen::Matrix<Type, 3, 3> getAlignMatrix(Eigen::Vector3f source, Eigen::Vector3f target)
{
  source.normalize();
  target.normalize();

  if(source == target)
  {
    return Eigen::Matrix<Type, 3, 3>::Identity();
  }

  Eigen::Matrix<Type, 1, 3> k = source.cross(target);
  Type sinTheta = k.norm();
  Type cosTheta = target.dot(source);

  Eigen::Matrix<Type, 3, 3> K;
  K <<  0, -k(2), k(1),
     k(2),     0, -k(0),
    -k(1),  k(0),     0;

  Eigen::Matrix<Type, 3, 3> R;
  R = Eigen::Matrix<Type, 3, 3>::Identity() + sinTheta * K + (1 - cosTheta) * K * K;
  return R;
}

#endif // __GEOMETRY_HPP__
