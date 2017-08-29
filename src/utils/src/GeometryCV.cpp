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

#include <rs/utils/GeometryCV.h>

#include <pcl/registration/icp.h>

// RS
#include <rs/utils/output.h>


//////////////////////////////////////////////////
void Camera::setFromMsgs(const sensor_msgs::CameraInfo &cameraInfo)
{
  float *it = this->matrix.ptr<float>(0);
  for (size_t i = 0; i < 9; ++i, ++it)
    *it = cameraInfo.K[i];

  distortion.clear();
  for(size_t i = 0; i < cameraInfo.D.size(); ++i)
    this->distortion.push_back(cameraInfo.D[i]);
}

namespace GeometryCV
{
//////////////////////////////////////////////////
cv::Mat poseRTToAffine(const ::PoseRT &pose)
{
  cv::Mat affine3DTransformation(3, 4, CV_32FC1);

  cv::Rodrigues(pose.rot, affine3DTransformation.colRange(0, 3));
  affine3DTransformation.at<float>(0, 3) = pose.trans(0);
  affine3DTransformation.at<float>(1, 3) = pose.trans(1);
  affine3DTransformation.at<float>(2, 3) = pose.trans(2);

  return affine3DTransformation;
}

//////////////////////////////////////////////////
cv::Point2f transform(const cv::Mat &M, const cv::Point2f &pt)
{
  cv::Mat vec(3, 1, CV_32FC1);

  vec.at<float>(0, 0) = pt.x;
  vec.at<float>(1, 0) = pt.y;
  vec.at<float>(2, 0) = 1.f;

  cv::Mat dst = M * vec;

  return cv::Point2f(dst.at<float>(0, 0), dst.at<float>(1, 0));
}

//////////////////////////////////////////////////
cv::Point3f transform(const cv::Mat &M, const cv::Point3f &pt)
{
  cv::Mat vec(4, 1, CV_32FC1);

  vec.at<float>(0, 0) = pt.x;
  vec.at<float>(1, 0) = pt.y;
  vec.at<float>(2, 0) = pt.z;
  vec.at<float>(3, 0) = 1.f;

  cv::Mat dst = M * vec;

  return cv::Point3f(dst.at<float>(0, 0),
                     dst.at<float>(1, 0),
                     dst.at<float>(2, 0));
}

//////////////////////////////////////////////////
cv::Vec3f transform(const cv::Mat &M, const cv::Vec3f &vec)
{
  cv::Point3f pt(vec[0], vec[1], vec[2]);

  return transform(M, pt);
}

//////////////////////////////////////////////////
cv::Point3f transform(const ::PoseRT &pose, const cv::Point3f &pt)
{
  cv::Mat M = poseRTToAffine(pose);

  return transform(M, pt);
}

//////////////////////////////////////////////////
cv::Vec3f transform(const ::PoseRT &pose, const cv::Vec3f &vec)
{
  cv::Mat M = poseRTToAffine(pose);

  return transform(M, vec);
}

//////////////////////////////////////////////////
std::vector<cv::Point2f> transform(const cv::Mat &M,
                                   const std::vector<cv::Point2f> &points)
{
  std::vector<cv::Point2f> result;

  for (const auto &pt : points)
    result.push_back(transform(M, pt));

  return result;
}

//////////////////////////////////////////////////
std::vector<cv::Point2f> transform(const cv::Mat &M,
                                   const std::vector<cv::Point2i> &points)
{
  std::vector<cv::Point2f> result;

  for (const auto &pt : points)
  {
    cv::Point2f ptf(pt.x, pt.y);

    result.push_back(transform(M, ptf));
  }

  return result;
}

//////////////////////////////////////////////////
std::vector<cv::Point3f> transform(const cv::Mat &M,
                                   const std::vector<cv::Point3f> &points)
{
  std::vector<cv::Point3f> result;
  result.reserve(points.size());

  for (const auto &pt : points)
    result.push_back(transform(M, pt));

  return result;
}

//////////////////////////////////////////////////
std::vector<cv::Vec3f> transform(const cv::Mat &M,
                                 const std::vector<cv::Vec3f> &vectors)
{
  std::vector<cv::Vec3f> result;
  result.reserve(vectors.size());

  for (const auto &vec : vectors)
    result.push_back(transform(M, vec));

  return result;
}

//////////////////////////////////////////////////
cv::Rect_<float> getBoundingRect(const std::vector<cv::Point2f> &points)
{
  cv::Rect_<float> pointsBoundingRect;

  auto h_it = std::minmax_element(points.cbegin(), points.cend(),
    [](const cv::Point2f &a, const cv::Point2f &b) {
      return a.x < b.x;});
  auto v_it = std::minmax_element(points.cbegin(), points.cend(),
    [](const cv::Point2f &a, const cv::Point2f &b) {
      return a.y < b.y;});

  pointsBoundingRect.x = h_it.first->x;
  pointsBoundingRect.y = v_it.first->y;
  pointsBoundingRect.width = h_it.second->x - pointsBoundingRect.x;
  pointsBoundingRect.height = v_it.second->y - pointsBoundingRect.y;

  return pointsBoundingRect;
}

//////////////////////////////////////////////////
pcl::search::KdTree<pcl::PointXY> getKdTree(
    const std::vector<cv::Point2f> &points)
{
  pcl::PointCloud<pcl::PointXY>::Ptr inputCloud{
    new pcl::PointCloud<pcl::PointXY>};

  inputCloud->width = points.size();
  inputCloud->height = 1;
  inputCloud->is_dense = false;

  inputCloud->points.resize(inputCloud->width * inputCloud->height);

  for(size_t i = 0; i < inputCloud->size(); ++i)
  {
    inputCloud->points[i] = {points[i].x, points[i].y};
  }

  pcl::search::KdTree<pcl::PointXY> kdTree(false);

  kdTree.setInputCloud(inputCloud);

  return kdTree;
}

//////////////////////////////////////////////////
cv::Point2f getNearestPoint(pcl::search::KdTree<pcl::PointXY> &templateKdTree,
                            const cv::Point2f &pt)
{
  pcl::PointXY searchPoint = {pt.x, pt.y};

  std::vector<int> indices;
  std::vector<float> l2SqrDistances;

  templateKdTree.nearestKSearch(searchPoint, 1, indices, l2SqrDistances);

  auto cloud = templateKdTree.getInputCloud();
  auto out_pt = cloud->points[indices.front()];

  return cv::Point2f(out_pt.x, out_pt.y);
}

//////////////////////////////////////////////////
std::tuple<cv::Mat, cv::Mat> compute2dDisparityResidualsAndWeights(
    const std::vector<cv::Point2f> &data,
    pcl::search::KdTree<pcl::PointXY> &templateKdTree)
{
  cv::Mat residuals(data.size(), 1, CV_32FC1);
  cv::Mat weights(data.size(), 1, CV_32FC1);

  int i = 0;
  for (const auto &pt : data)
  {
    auto nearestTemplatePoint = getNearestPoint(templateKdTree, pt);

    float distance = cv::norm(nearestTemplatePoint - pt);
    residuals.at<float>(i, 0) = distance * distance;
    // or how do we check if point matches???
    weights.at<float>(i, 0) = (distance <= 5);

    i++;
  }

  return std::tie(residuals, weights);
}

//////////////////////////////////////////////////
std::vector<cv::Point2f> projectPoints(const std::vector<cv::Point3f> &points,
                                       const ::PoseRT &pose,
                                       const ::Camera &camera)
{
  std::vector<cv::Point2f> points_2d;
  cv::projectPoints(points, pose.rot, pose.trans,
      camera.matrix, camera.distortion, points_2d);

  return points_2d;
}

//////////////////////////////////////////////////
cv::Vec3f projectRotationOnAxis(const cv::Vec3f &rodrigues,
                                const cv::Vec3f &axis)
{
  // find axes rotation transformation to align object's up to plane normal
  cv::Vec3f objectsUpDirectionLocal(0, 1, 0);
  ::PoseRT objectToCameraRotation{rodrigues, cv::Vec3f(0, 0, 0)};
  auto objectsUpDirectionCamspace =
      GeometryCV::transform(objectToCameraRotation, objectsUpDirectionLocal);

  double phi = std::acos(axis.ddot(objectsUpDirectionCamspace));

  cv::Vec3f upToAxisRotationAxis = objectsUpDirectionCamspace.cross(axis);
  cv::Vec3f rodriguesUpToAxis = upToAxisRotationAxis *
                                    (phi / cv::norm(upToAxisRotationAxis));

  cv::Mat upToAxisRotation;
  cv::Rodrigues(rodriguesUpToAxis, upToAxisRotation);


  cv::Mat initialRotation;
  cv::Rodrigues(rodrigues, initialRotation);
  cv::Mat finalRotation = upToAxisRotation * initialRotation;

  cv::Mat result;
  cv::Rodrigues(finalRotation, result);

  return result;
}

//////////////////////////////////////////////////
::PoseRT offsetPose(const ::PoseRT &input, const int dofId, const float offset)
{
  ::PoseRT poseDelta {{0, 0, 0}, {0, 0, 0}};

  switch (dofId / 3)
  {
  case 0:
    poseDelta.rot[dofId] += offset;
    break;

  case 1:
    poseDelta.trans[dofId % 3] += offset;
    break;

  default:
    outError("Invalid dofId: " << dofId << ", acceptable range is [1-6]");
  }

  return input + poseDelta;
}

//////////////////////////////////////////////////
cv::Mat computeProximityJacobianForPoseRT(const ::PoseRT &pose,
    const std::vector<cv::Point3f> &points3D, const float h,
    const std::vector<cv::Point2f> &template2D,
    const cv::Mat &weights, const ::Camera &camera)
{
  size_t dof = 6;

  auto templateKdTree = getKdTree(template2D);

  cv::Mat jacobian(points3D.size(), dof, CV_32FC1);
  for (size_t j = 0; j < dof; ++j)
  {
    ::PoseRT posePlusH  = offsetPose(pose, j, h);
    ::PoseRT poseMinusH = offsetPose(pose, j, -h);

    std::vector<cv::Point2f> transformedPointsPlus =
        projectPoints(points3D, posePlusH, camera);
    std::vector<cv::Point2f> transformedPointsMinus =
        projectPoints(points3D, poseMinusH, camera);

    #pragma omp parallel for
    for (size_t i = 0; i < transformedPointsPlus.size(); ++i)
    {
      auto nearestPointPlus = getNearestPoint(templateKdTree,
                                              transformedPointsPlus[i]);
      auto nearestPointMinus = getNearestPoint(templateKdTree,
                                               transformedPointsMinus[i]);

      double d1 = std::pow(cv::norm(nearestPointPlus -
                                        transformedPointsPlus[i]), 2);
      double d2 = std::pow(cv::norm(nearestPointMinus -
                                        transformedPointsMinus[i]), 2);

      float dEi_dAj = weights.at<float>(i) * (d1 - d2) / (2 * h);

      jacobian.at<float>(i, j) = dEi_dAj;
    }
  }

  return jacobian;
}

//////////////////////////////////////////////////
std::tuple<::PoseRT, double, cv::Mat> fit2d3d(
    const std::vector<cv::Point3f> &points, const ::PoseRT &init_pose,
    const std::vector<cv::Point2f> &template2D, const ::Camera &camera,
    const size_t iterationsLimit, cv::Vec3f normalConstraint)
{
  ::PoseRT currentPose = init_pose;
  float learningRate = 2;
  double limitEpsilon = 1e-5;
  size_t iterationsLeft = iterationsLimit;
  size_t stallCounter = 0;
  double h = 1e-3;

  double lastError = 0;
  cv::Mat jacobian;

  auto templateKdTree = getKdTree(template2D);

  bool done {false};
  while (!done && iterationsLeft)
  {

    std::vector<cv::Point2f> projectedPoints =
        projectPoints(points, currentPose, camera);

    cv::Mat residuals;
    cv::Mat weights;
    std::tie(residuals, weights) =
        compute2dDisparityResidualsAndWeights(projectedPoints, templateKdTree);

    double num{0};
    double sum{0};
    for (int i = 0; i < residuals.rows; ++i)
    {
      if (residuals.at<float>(i) < 0.9f)
      {
        sum += residuals.at<float>(i);
        num++;
      }
    }

    double currentError = std::numeric_limits<double>::max();
    if (num != 0)
      currentError = std::sqrt(sum) / num;
    else
    {
      outInfo("Something's wrong - the original pose if too far from desired");
      break;
    }

    // outInfo("currentError: " << currentError
    //                          << " (" << num << "/" << weights.rows << ")") ;

    if (std::abs(lastError - currentError) < limitEpsilon)
    {
      if (stallCounter == 5)
      {
        outInfo("Done");
        done = true;
      }
      stallCounter++;
    }
    else
      stallCounter = 0;

    lastError = currentError;

    jacobian = computeProximityJacobianForPoseRT(currentPose, points, h,
                                                 template2D, weights, camera);
    if (cv::countNonZero(jacobian) == 0 || cv::sum(weights)[0] == 0)
    {
      outInfo("Already at best approximation, or `h` is too small");
      currentError = cv::norm(residuals, cv::NORM_L2);
      break;
    }
    // jacobian = jacobian / cv::norm(jacobian, cv::NORM_INF);
    // outInfo("Jacobian: " << jacobian);

    cv::Mat deltaPoseMat;
    cv::solve(jacobian, residuals, deltaPoseMat, cv::DECOMP_SVD);

    ::PoseRT deltaPoseRT;
    deltaPoseRT.rot = cv::Vec3f(deltaPoseMat.at<float>(0),
                                deltaPoseMat.at<float>(1),
                                deltaPoseMat.at<float>(2));
    deltaPoseRT.trans = cv::Vec3f(deltaPoseMat.at<float>(3),
                                  deltaPoseMat.at<float>(4),
                                  deltaPoseMat.at<float>(5));

    currentPose = currentPose + (-1 * learningRate) * deltaPoseRT;

    // apply up-direction constraint if present
    if (cv::norm(normalConstraint) > 0.1)
    {
      currentPose.rot = projectRotationOnAxis(currentPose.rot,
                                              normalConstraint);
    }

    --iterationsLeft;
  }

  return std::tie(currentPose, lastError, jacobian);
}

//////////////////////////////////////////////////
std::tuple<double, double> getChamferDistance(const std::vector<cv::Point2f> &a,
                                              const std::vector<cv::Point2f> &b,
                                              const cv::Size workArea,
                                              cv::Mat &distanceTransform)
{
  double distancesSum = 0;
  size_t numberOfPointsInWorkArea = 0;

  if (distanceTransform.empty())
  {
    cv::Mat silhouetteOfPointsB = cv::Mat::ones(workArea, CV_8UC1);
    for (auto it = b.cbegin(); it != std::prev(b.cend()); it = std::next(it))
      cv::line(silhouetteOfPointsB, *it, *std::next(it), cv::Scalar(0));
    cv::line(silhouetteOfPointsB, b.back(), b.front(), cv::Scalar(0));

    // auto rect_a = ::getBoundingRect(a);
    // auto rect_b = ::getBoundingRect(b);
    // cv::Rect roi =
    //     (rect_a | rect_b) & cv::Rect_<float>(cv::Point(0, 0), workArea);

    distanceTransform = cv::Mat::zeros(silhouetteOfPointsB.size(), CV_32FC1);
    // cv::distanceTransform(silhouetteOfPointsB(roi), distanceTransform(roi),
    //                       CV_DIST_L2, 3);
    cv::distanceTransform(silhouetteOfPointsB, distanceTransform,
                          CV_DIST_L2, 3);
  }

  auto workRect = cv::Rect(cv::Point(0, 0), workArea);
  for (const auto &pt : a)
  {
    auto pti = cv::Point(pt.x, pt.y);
    if (workRect.contains(pti))
    {
      distancesSum += distanceTransform.at<float>(pti);
      numberOfPointsInWorkArea += 1;
    }
  }

  double distance;
  double confidence = static_cast<double>(numberOfPointsInWorkArea) / a.size();

  if (confidence != 0)
    distance = distancesSum / numberOfPointsInWorkArea;
  else
  {
    outWarn("Input contour is too large or contains no points");
    distance = std::numeric_limits<double>::max();
  }

  return std::tie(distance, confidence);
}

//////////////////////////////////////////////////
std::vector<cv::Point2f> normalizePoints(const std::vector<cv::Point2f> &points)
{
  std::vector<cv::Point2f> result;
  for (const auto &pt : points)
    result.push_back(pt);

  auto mean = std::accumulate(points.cbegin(), points.cend(), cv::Point2f());
  mean *= (1.f / points.size());

  float stdDev = 0;

  for (auto &pt : result)
  {
    pt = pt - mean;
    stdDev += std::pow(cv::norm(pt), 2);
  }

  stdDev = std::sqrt(stdDev / points.size());

  for (auto &pt : result)
    pt *= 1.f / stdDev;

  return result;
}

//////////////////////////////////////////////////
void vectorToPointCloud(const std::vector<cv::Point2f> &points,
    pcl::PointCloud<pcl::PointXYZ> &pc)
{
  pc.width = points.size();
  pc.height = 1;
  pc.is_dense = false;

  pc.resize(pc.width * pc.height);

  for (size_t i = 0; i < points.size(); ++i)
  {
    pc.points[i] = {points[i].x, points[i].y, 0};
  }
}

//////////////////////////////////////////////////
void pointCloudToVector(pcl::PointCloud<pcl::PointXYZ> &pc,
    std::vector<cv::Point2f> &points)
{
  points.clear();

  assert(pc.height == 1);

  for (size_t i = 0; i < pc.width; ++i)
  {
    points.push_back(cv::Point2f(pc.points[i].x, pc.points[i].y));
  }
}

//////////////////////////////////////////////////
std::tuple<cv::Mat, double> fitICP(const std::vector<cv::Point2f> &test,
    const std::vector<cv::Point2f> &model)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTest{
      new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudModel{
      new pcl::PointCloud<pcl::PointXYZ>};

  vectorToPointCloud(test, *cloudTest);
  vectorToPointCloud(model, *cloudModel);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloudTest);
  icp.setInputTarget(cloudModel);

  pcl::PointCloud<pcl::PointXYZ> cloudResult;

  icp.align(cloudResult);

  assert(icp.hasConverged());

  double score = icp.getFitnessScore();


  Eigen::Matrix4f eigenTransformationMat = icp.getFinalTransformation ();

  cv::Mat cvTransformationMat(3, 3, CV_32FC1, cv::Scalar(0.f));
  cvTransformationMat.at<float>(0, 0) = eigenTransformationMat(0, 0);
  cvTransformationMat.at<float>(0, 1) = eigenTransformationMat(0, 1);
  cvTransformationMat.at<float>(1, 0) = eigenTransformationMat(1, 0);
  cvTransformationMat.at<float>(1, 1) = eigenTransformationMat(1, 1);

  // assume that translation is small enough
  cvTransformationMat.at<float>(0, 2) = 0; //eigenTransformationMat(0, 3);
  cvTransformationMat.at<float>(1, 2) = 0; //eigenTransformationMat(1, 3);
  cvTransformationMat.at<float>(2, 2) = 1.f;

  return std::tie(cvTransformationMat, score);
}

//////////////////////////////////////////////////
std::tuple<cv::Vec2f, float> getMeanAndStdDev(
  const std::vector<cv::Point2f> &points)
{
  cv::Point2f mean = std::accumulate(points.cbegin(), points.cend(),
                                     cv::Point2f());
  mean *= (1.f / points.size());

  float stdDev = 0;
  for (auto &pt : points)
    stdDev += std::pow(cv::norm(cv::Point2f(pt.x, pt.y) - mean), 2);

  stdDev = std::sqrt(stdDev / points.size());

  return std::tie(mean, stdDev);
}

//////////////////////////////////////////////////
std::tuple<cv::Mat, double> fitProcrustes2d(
    const std::vector<cv::Point2f> &points,
    const std::vector<cv::Point2f> &templatePoints)
{
  cv::Vec2f pointsMean, templatesMean;
  float pointsDeviation, templatesDeviation;

  // FIXME: avoid double mean/deviation computation
  std::tie(pointsMean, pointsDeviation) = getMeanAndStdDev(points);
  std::tie(templatesMean, templatesDeviation) =
      getMeanAndStdDev(templatePoints);

  auto normalizedPointsSet = normalizePoints(points);
  auto normalizetTemplateSet = normalizePoints(templatePoints);

  cv::Mat icpRotationMat;
  double fitness;
  std::tie(icpRotationMat, fitness) = fitICP(normalizedPointsSet,
                                             normalizetTemplateSet);

  cv::Mat Ts_inv = (cv::Mat_<float>(3, 3) << 1, 0, -pointsMean(0),
                                             0, 1, -pointsMean(1),
                                             0, 0, 1);
  cv::Mat Ss_inv = (cv::Mat_<float>(3, 3) << 1/pointsDeviation, 0, 0,
                                             0, 1/pointsDeviation, 0,
                                             0, 0, 1);
  cv::Mat Rst = icpRotationMat;
  cv::Mat St = (cv::Mat_<float>(3, 3) << templatesDeviation, 0, 0,
                                         0, templatesDeviation, 0,
                                         0 , 0, 1);
  cv::Mat Tt = (cv::Mat_<float>(3, 3) << 1, 0, templatesMean(0),
                                         0, 1, templatesMean(1),
                                         0 , 0, 1);

  cv::Mat pointsToTemplateTransformationMat = Tt * St * Rst * Ss_inv * Ts_inv;

  return std::tie(pointsToTemplateTransformationMat, fitness);
}

}
