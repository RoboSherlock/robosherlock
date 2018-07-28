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

#ifndef __BILATERAL_SYMMETRY_HPP__
#define __BILATERAL_SYMMETRY_HPP__

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <rs/segmentation/Geometry.hpp>
#include <rs/utils/output.h>

/** \brief Class representing a bilateral symmetry in 3D space. A symmetry
 * is represented as a 3D plane.
 */
class BilateralSymmetry
{
private:
  Eigen::Vector3f origin;
  Eigen::Vector3f normal;
public:

  /** \brief Default constructor. */
  BilateralSymmetry() : origin(Eigen::Vector3f::Zero()), normal(Eigen::Vector3f::Zero()) {}

  /** \brief Constructor to initialize origin point and plane normal. */
  BilateralSymmetry(const Eigen::Vector3f &orig, const Eigen::Vector3f &nor) : origin(orig), normal(nor.normalized()) {}

  /** \brief Constructor to initialize symmetry coefficients. */
  BilateralSymmetry(const Eigen::Vector4f &plane)
  {
    planeToPointNormal<float>(plane, origin, normal);
  }

  /** \brief Destructor. */
  ~BilateralSymmetry() {}

  /** \brief Get origin point. */
  Eigen::Vector3f getOrigin() const { return origin;}

  /** \brief Get normal. */
  Eigen::Vector3f getNormal() const { return normal;}

  /** \brief Get symmetry plane coefficients. */
  Eigen::Vector4f getPlane() const
  {
    Eigen::Vector4f plane;
    plane.head(3) = normal;
    plane[3] = -origin.dot(normal);
    return plane;
  }

  /** \brief Set origin point.
   *  \param[in]  orig   input origin
   */
  inline void setOrigin(const Eigen::Vector3f &orig) { origin = orig;}

  /** \brief Set normal.
   *  \param[in]  orig   input origin
   */
  inline void setNormal(const Eigen::Vector3f &nor)  { normal = nor; }

  /** \brief Project point to the symmetry plane.
   *  \param[in]  point   input point
   *  \return projected point on the symmetry plane
   */
  Eigen::Vector3f projectPoint(const Eigen::Vector3f &point) const
  {
    return pointToPlaneProjection<float>(point, origin, normal);
  }

  /** \brief Set projected point from input point as symmetry origin.
   *  \param[in]  point   input point
   */
  inline void setProjectedOrigin(const Eigen::Vector3f &point)
  {
    origin = this->projectPoint(point);
  }

  /** \brief compute signed distance from point to symmetry plane.
   *  \param[in]  point   input point
   *  \return signed distance
   */
  inline float pointSignedDist(const Eigen::Vector3f &point) const
  {
    return pointToPlaneSignedNorm<float>(point, origin, normal);
  }

  /** \brief compute angle and distance difference between this symmetry and target symmetry
   *  \param[in]   target   target symmetry
   *  \param[out]  angle    angle difference
   *  \param[out]  dist     distance difference
   */
  inline void bilateralSymDiff(const BilateralSymmetry &target, float &angle, float &dist)
  {
    angle = lineLineAngle<float>(this->normal, target.getNormal());
    dist = pointToPointNorm<float>(this->origin, target.getOrigin());
  }

  /** \brief reflect point through symmetry plane
   *  \param[in]  point   input point
   *  \return reflected point
   */
  inline Eigen::Vector3f reflectPoint(const Eigen::Vector3f &point) const
  {
    return (point - 2 * this->normal * this->normal.dot(point - this->origin));
  }

  /** \brief reflect nomral through symmetry plane
   *  \param[in]  normal   input normal
   *  \return reflected normal
   */
  inline Eigen::Vector3f reflectNormal(const Eigen::Vector3f &normal) const
  {
    return (normal - 2 * normal.dot(this->normal) * this->normal);
  }

  /** \brief reflect second normal and compute angle difference of reflected normal and the first normal
   *  \param[in]  normal1   first normal
   *  \param[in]  normal2   second normal
   *  \return angle difference
   */
  inline float getBilSymNormalFitError(const Eigen::Vector3f &normal1, const Eigen::Vector3f &normal2)
  {
    Eigen::Vector3f reflectedNormal2 = reflectNormal(normal2);
    float value = clamp(reflectedNormal2.dot(normal1), -1.0f, 1.0f);
    return std::acos(value);
  }

  /** \brief compute mid point of two input point and compute distance from mid point to symmetry plane
   *  \param[in]  point1   first point
   *  \param[in]  point2   second point
   *  \return distance
   */
  inline float getBilSymPositionFitError(const Eigen::Vector3f &point1, const Eigen::Vector3f &point2)
  {
    Eigen::Vector3f mid = (point1 + point2) / 2;
    return pointSignedDist(mid);
  }

  /** \brief overrided method to output symmetry plane
   */
  std::ostream& operator<<(std::ostream &output)
  {
    output << "Origin: " << origin.transpose() << '\n';
    output << "Normal: " << normal.transpose() << '\n';
    return output;
  }
};

enum CorrespondenceMethod
{
  NEAREST,
  NEIGHBOR_RADIUS
};

/** \brief Finding correspondences symmetric points from original cloud and downsampled cloud.
 *  Method NEIGHBOR_RADIUS: for each downsampled point, reflect downsampled point and find all neighbor
 *  within search radius, then find the minimum angle between reflected normal and these neighbor cloud normal. Add that minimum angle as correspondence if not exceed max fit error
 *  Method NEAREST: for each downsampled point, reflect downsampled point and find nearest neighbor
 *  then find angle between reflected normal and nearest neighbor cloud normal. Add that angle as correspondence if not exceed max fit error
 *  \param[in]  cloud            original cloud
 *  \param[in]  normals          original cloud normals
 *  \param[in]  dsCloud          downsampled cloud
 *  \param[in]  dsNormals        downsampled cloud normals
 *  \param[in]  tree             search tree of scene cloud
 *  \param[in]  symmetry         input symmetry
 *  \param[out] correspondences  symmetric correspondences
 *  \param[in]  method           finding correspondences method
 *  \param[in]  search_radius    search radius of method NEIGHBOR_RADIUS
 *  \param[in]  max_normal_fit_error                      maximum error normal
 *  \param[in]  min_sym_corresspondence_dist              minimum distance between correspondences to be considered
 *  \param[in]  max_sym_corresspondence_reflected_dist    maximum distance between reflected point and original point to be considered
 *  \return false if cloud size or dsCloud size is zero
 */
template<typename PointT>
inline bool findBilateralSymmetryCorrespondences(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                                 pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                                 typename pcl::PointCloud<PointT>::Ptr &dsCloud,
                                                 pcl::PointCloud<pcl::Normal>::Ptr &dsNormals,
                                                 typename pcl::search::KdTree<PointT>::Ptr &tree,
                                                 BilateralSymmetry &symmetry,
                                                 pcl::Correspondences &correspondences,
                                                 CorrespondenceMethod method,
                                                 float search_radius = 0.01f,
                                                 float max_normal_fit_error = 0.174f,
                                                 float min_sym_corresspondence_dist = 0.02f,
                                                 float max_sym_corresspondence_reflected_dist = 0.005
                                                 )
{
  if(cloud->size() == 0 || dsCloud->size() == 0)
  {
    outWarn("No point in cloud! Cloud need at least one point!");
    return false;
  }

  correspondences.clear();
  Eigen::Vector3f symOrigin = symmetry.getOrigin();
  Eigen::Vector3f symNormal = symmetry.getNormal();

  for(size_t pointId = 0; pointId < dsCloud->size(); pointId++)
  {
    Eigen::Vector3f srcPoint = dsCloud->points[pointId].getVector3fMap();
    Eigen::Vector3f srcNormal(dsNormals->points[pointId].normal_x, dsNormals->points[pointId].normal_y, dsNormals->points[pointId].normal_z);

    Eigen::Vector3f reflectedSrcPoint = symmetry.reflectPoint(srcPoint);
    Eigen::Vector3f reflectedSrcNormal = symmetry.reflectNormal(srcNormal);

    std::vector<float> dists;
    std::vector<int> neighborIndices;
    PointT searchPoint;
    searchPoint.getVector3fMap() = reflectedSrcPoint;

    if(method == NEIGHBOR_RADIUS)
    {
      tree->radiusSearch(searchPoint, search_radius, neighborIndices, dists);

      int bestId = -1;
      float minNormalFitError = std::numeric_limits<float>::max();
      for(size_t it = 0; it < neighborIndices.size(); it++)
      {
        int id = neighborIndices[it];
        Eigen::Vector3f tgtPoint = cloud->points[id].getVector3fMap();
        Eigen::Vector3f tgtNormal(normals->points[id].normal_x, normals->points[id].normal_y, normals->points[id].normal_z);

        if(std::abs(symmetry.pointSignedDist(srcPoint) - symmetry.pointSignedDist(tgtPoint)) < min_sym_corresspondence_dist)
        {
          continue;
        }

        float currNormalFitError = symmetry.getBilSymNormalFitError(reflectedSrcNormal, tgtNormal);

        if(currNormalFitError > max_normal_fit_error)
        {
          continue;
        }

        if(currNormalFitError < minNormalFitError)
        {
          minNormalFitError = currNormalFitError;
          bestId = id;
        }
      }

      if(bestId != -1)
      {
        correspondences.push_back(pcl::Correspondence(pointId, bestId, minNormalFitError));
      }
    }
    else if(method == NEAREST)
    {
      tree->nearestKSearch(searchPoint, 1, neighborIndices, dists);

      Eigen::Vector3f tgtPoint = cloud->points[neighborIndices[0]].getVector3fMap();
      Eigen::Vector3f tgtNormal(normals->points[neighborIndices[0]].normal_x, normals->points[neighborIndices[0]].normal_y, normals->points[neighborIndices[0]].normal_z);

      if(std::abs(symmetry.pointSignedDist(srcPoint) - symmetry.pointSignedDist(tgtPoint)) < min_sym_corresspondence_dist)
      {
        continue;
      }

      if(dists[0] > max_sym_corresspondence_reflected_dist * max_sym_corresspondence_reflected_dist)
      {
        continue;
      }

      float normalFitError = symmetry.getBilSymNormalFitError(reflectedSrcNormal, tgtNormal);

      if(normalFitError > max_normal_fit_error)
      {
        continue;
      }

      correspondences.push_back(pcl::Correspondence(pointId, neighborIndices[0], normalFitError));
    }
  }

  pcl::registration::CorrespondenceRejectorOneToOne corresRejectOneToOne;
  corresRejectOneToOne.getRemainingCorrespondences(correspondences, correspondences);

  if(correspondences.size() == 0)
  {
    return false;
  }
  return true;
}

#endif //__BILATERAL_SYMMETRY_HPP__
