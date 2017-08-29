/**
 * Copyright 2017 University of Bremen, Institute for Artificial Intelligence
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

#ifndef __GEOMETRY_CV_H__
#define __GEOMETRY_CV_H__

#include <tuple>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/CameraInfo.h>

// PCL
#include <pcl/common/norms.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>


/// \struct PoseRT GeometryCV.h
/// \brief Rigid 3d transformation presented as rodrigues and translation vector
struct PoseRT
{
  /// \brief Rotation rodrigues vector
  cv::Vec3f rot;

  /// \brief Translation vector
  cv::Vec3f trans;
};

// \class Camera GeometryCV.h
/// \brief Camera model
class Camera
{
  /// \brief 3x3 camera matrix
  public: cv::Mat matrix = cv::Mat::eye(3, 3, CV_32FC1);

  /// \brief Camera distortion coefficients
  public: std::vector<float> distortion;

  /// \brief Set camera parameters from ROS message
  /// \param[in]  cameraInfo Camera parameters message
  public: void setFromMsgs(const sensor_msgs::CameraInfo &cameraInfo);
};

/// \namespace GeometryCV GeometryCV.h
/// \brief Contains various geometry functions based on OpenCV primitives
namespace GeometryCV
{
  /// \brief Convert ::PoseRT to cv::Mat representation
  /// \param[in] pose Input pose
  /// \return         Affine transformation 3x4 matrix
  cv::Mat poseRTToAffine(const ::PoseRT &pose);

  /// \brief Per-component pose addition
  /// \param[in] a  First addendum
  /// \param[in] b  Second addendum
  inline ::PoseRT operator+(const ::PoseRT &a, const ::PoseRT &b)
  {
    ::PoseRT result;

    result.rot = a.rot + b.rot;
    result.trans = a.trans + b.trans;

    return result;
  }

  /// \brief Per-component pose multiplication
  /// \param[in] a  Scalar multiplier
  /// \param[in] b  Pose
  /// \return       Scaled pose
  ///   Translation is scaled along translation vector,
  ///   rotation angle is scaled along the same axis
  inline ::PoseRT operator*(const double a, const ::PoseRT &b)
  {
    ::PoseRT result;

    result.rot = b.rot * a;
    result.trans = b.trans * a;

    return result;
  }

  /// \brief Apply affine transformation to point
  /// \param[in] M  2x3 matrix
  /// \param[in] pt Point
  /// \return       Transformed point
  cv::Point2f transform(const cv::Mat &M, const cv::Point2f &pt);

  /// \brief Apply affine transformation to point
  /// \param[in] M  3x4 matrix
  /// \param[in] pt Point
  /// \return       Transformed point
  cv::Point3f transform(const cv::Mat &M, const cv::Point3f &pt);

  /// \brief Apply affine transformation to vector
  /// \param[in] M    3x4 matrix
  /// \param[in] vec  Vector
  /// \return         Transformed vector
  cv::Vec3f transform(const cv::Mat &M, const cv::Vec3f &vec);

  /// \brief Apply ::PoseRT transformation to point
  /// \param[in] pose ::PoseRT transformation
  /// \param[in] pt   Point
  cv::Point3f transform(const ::PoseRT &pose, const cv::Point3f &pt);

  /// \brief Apply ::PoseRT transformation to vector
  /// \param[in] pose ::PoseRT transformation
  /// \param[in] vec  Vector
  /// \return         Transformed point
  cv::Vec3f transform(const ::PoseRT &pose, const cv::Vec3f &vec);

  /// \brief Apply affine transformation to points
  /// \param[in] M      2x3 matrix
  /// \param[in] points Points
  /// \return           Transformed points
  std::vector<cv::Point2f> transform(const cv::Mat &M,
                                     const std::vector<cv::Point2f> &points);

  /// \brief Apply affine transformation to points
  /// \param[in] M      2x3 matrix
  /// \param[in] points Points
  /// \return           Transformed points
  std::vector<cv::Point2f> transform(const cv::Mat &M,
                                     const std::vector<cv::Point2i> &points);

  /// \brief Apply affine transformation to points
  /// \param[in] M      3x4 matrix
  /// \param[in] points Points
  /// \return           Transformed points
  std::vector<cv::Point3f> transform(const cv::Mat &M,
                                     const std::vector<cv::Point3f> &points);

  /// \brief Apply affine transformation to vectors
  /// \param[in] M       3x4 matrix
  /// \param[in] vectors Points
  /// \return            Transformed vectors
  std::vector<cv::Vec3f> transform(const cv::Mat &M,
                                   const std::vector<cv::Vec3f> &vectors);

  /// \brief Get floating point bounding rect for a set of points
  /// \param[in] points Vector of points
  /// \return           Rectangle which contains all the points from input
  ///    Starting from some OpenCV version (like 2.4.8) function is present in cv ns.
  cv::Rect_<float> getBoundingRect(const std::vector<cv::Point2f> &points);

  /// \brief Create KdTree for a set of points
  /// \param[in] points Points to create KdTree on
  /// \return           pcl::search::KdTree
  pcl::search::KdTree<pcl::PointXY> getKdTree(const std::vector<cv::Point2f> &points);

  /// \brief Find nearest point to template
  /// \param[in] template_kd_tree Search template as KdTree
  /// \param[in] pt               Input point
  /// \return                     Nearest point from the set to the input `pt`
  cv::Point2f getNearestPoint(pcl::search::KdTree<pcl::PointXY> &templateKdTree,
                              const cv::Point2f &pt);

  /// \brief Find the distance of each 2d point to template, and weights which mark outliers
  /// \param[in] data           N Input points
  /// \param[in] templateKdTree Template as KdTree
  /// \return                   Tuple (residuals[Nx1], weights[Nx1])
  std::tuple<cv::Mat, cv::Mat> compute2dDisparityResidualsAndWeights(
      const std::vector<cv::Point2f> &data,
      pcl::search::KdTree<pcl::PointXY> &templateKdTree);

  /// \brief Shortcut to project transformed points using camera
  /// \param[in] points 3D points
  /// \param[in] pose   Points transformation
  /// \param[in] camera Projection camera
  /// \return           2D points
  std::vector<cv::Point2f> projectPoints(const std::vector<cv::Point3f> &points,
                                         const ::PoseRT &pose,
                                         const ::Camera &camera);

  /// \brief Get the most similar rotation but along another axis
  /// \param[in] rodrigues  Input rotation
  /// \param[in] axis       New rotation axis
  /// \return               New rodrigues rotation
  cv::Vec3f projectRotationOnAxis(const cv::Vec3f &rodrigues,
                                  const cv::Vec3f &axis);

  /// \brief Offset pose along some single axis by given value
  /// \param[in] input  Initial pose
  /// \param[in] dofId Index of DOF in range [0,5]
  /// \param[in] offset Offset to apply to given DOF
  /// \return           New pose
  ::PoseRT offsetPose(const ::PoseRT &input, const int dofId,
                      const float offset);

  /// \brief Compute Jacobian of pose to residuals transformation
  /// \param[in] pose        Point to compute jacobi matrix at
  /// \param[in] points3D    3d points to project
  /// \param[in] h           Transformation axis delta
  /// \param[in] template2D  2D template
  /// \param[in] weights     Outliers mask
  /// \param[in] camera      Camera to project 3d points
  /// \return                Jacobi Nx6 matrix
  cv::Mat computeProximityJacobianForPoseRT(const ::PoseRT &pose,
      const std::vector<cv::Point3f> &points3D, const float h,
      const std::vector<cv::Point2f> &template2D,
      const cv::Mat &weights, const ::Camera &camera);

  /// \brief Find a better pose to match points on projection using ICP algorithm
  /// \param[in] points           Input 3d points
  /// \param[in] init_pose        Initial pose to refine
  /// \param[in] template2D       2d template to fit
  /// \param[in] camera           Camera to project points
  /// \param[in] iterationsLimit Maximal number of iterations to perform
  /// \param[in] normalConstraint Normalised vector, if != 0 then at each iteration pose is aligned with normal
  /// \return                     Tuple (refinedPose, distance, lastJacobian)
  std::tuple<::PoseRT, double, cv::Mat> fit2d3d(
      const std::vector<cv::Point3f> &points, const ::PoseRT &init_pose,
      const std::vector<cv::Point2f> &template2D, const ::Camera &camera,
      const size_t iterationsLimit,
      cv::Vec3f normalConstraint = cv::Vec3f(0, 0, 0));

  /// \brief Calculate distance between two 2d point sets using distance transform
  /// \param[in] a                      First point set
  /// \param[in] b                      Template point set
  /// \param[in] workArea               Size of the area to perform distance transform on
  /// \param[in,out] distanceTransform  Matrix with computed distance transform (will be calculated if empty)
  /// \return                           Tuple (distance, rate_of_points_hit)
  std::tuple<double, double> getChamferDistance(
      const std::vector<cv::Point2f> &a, const std::vector<cv::Point2f> &b,
      const cv::Size workArea, cv::Mat &distanceTransform);

  /// \brief Unbias the point set and scale such that it's standard deviation is one
  /// \param[in] points Input point set
  /// \return           A new point set
  std::vector<cv::Point2f> normalizePoints(
      const std::vector<cv::Point2f> &points);

  /// \breaf Copy points from std::vector to pcl::PointCloud
  /// \param[in]  points  Input cv points
  /// \param[out] pc      PointCloud to store points to
  void vectorToPointCloud(const std::vector<cv::Point2f> &points,
      pcl::PointCloud<pcl::PointXYZ> &pc);

  /// \breaf Copy points from std::vector to pcl::PointCloud
  /// \param[in]  pc      Input PointCloud
  /// \param[out] points  std::vector to store cv points to
  void pointCloudToVector(pcl::PointCloud<pcl::PointXYZ> &pc,
      std::vector<cv::Point2f> &points);

  /// \brief Fit rigid point set onto template and get corresponding transformation
  /// \param[in] test   Point set
  /// \param[in] model  Template to fit points on
  /// \return           Tuple (affine[2x3], fitnessDistance)
  std::tuple<cv::Mat, double> fitICP(const std::vector<cv::Point2f> &test,
      const std::vector<cv::Point2f> &model);

  /// \brief Get mean and standard deviation of point set
  /// \param[in] points Input point set
  /// \return           Tuple (mean, stdDev)
  std::tuple<cv::Vec2f, float> getMeanAndStdDev(
      const std::vector<cv::Point2f> &points);

  /// \brief Find 2d affine transformation to fit one point set to another
  /// \param[in] points         Input point set
  /// \param[in] templatePoints Template to fit
  /// \return         Tuple (affine[2x3], fitnessDistance)
  std::tuple<cv::Mat, double> fitProcrustes2d(
      const std::vector<cv::Point2f> &points,
      const std::vector<cv::Point2f> &templatePoints);
}

#endif /*__GEOMETRY_CV_H__*/
