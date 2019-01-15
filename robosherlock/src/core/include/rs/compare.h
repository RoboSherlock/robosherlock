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

#include <rs/types/annotation_types.h>
#include <rs/conversion/conversion.h>

#include <cmath>

#include <opencv2/opencv.hpp>

namespace rs
{
template<class T>
double compare(T &a, T &b)
{
  return NAN;
}

template<>
double compare(PoseAnnotation &a, PoseAnnotation &b)
{
  tf::Stamped<tf::Pose> aPose, bPose;
  tf::Vector3 aTrans, bTrans;

  conversion::from(a.world.get(), aPose);
  conversion::from(b.world.get(), bPose);
  aTrans = aPose.getOrigin();
  bTrans = bPose.getOrigin();

  return std::min(1.0, aTrans.distance(bTrans) * 4.0); // everything further away than 25 cm is 1.0
}

template<>
double compare(TFLocation &a, TFLocation &b)
{
  if(a.frame_id() == b.frame_id() && a.reference_desc() == b.reference_desc()) {
    return 0;
  }
  return 1;
}

template<>
double compare(Shape &a, Shape &b)
{
  if(a.shape() == b.shape()) {
    return 0;
  }
  return 1;
}

template<>
double compare(Geometry &a, Geometry &b)
{
  BoundingBox3D boxA = a.boundingBox();
  BoundingBox3D boxB = b.boundingBox();

  const double aW = boxA.width();
  const double bW = boxB.width();
  const double minW = std::min(aW, bW);
  const double diffW = fabs(aW - bW);
  const double distW = std::min(1.0, diffW / minW);

  const double aH = boxA.height();
  const double bH = boxB.height();
  const double minH = std::min(aH, bH);
  const double diffH = fabs(aH - bH);
  const double distH = std::min(1.0, diffH / minH);

  const double aD = boxA.depth();
  const double bD = boxB.depth();
  const double minD = std::min(aD, bD);
  const double diffD = fabs(aD - bD);
  const double distD = std::min(1.0, diffD / minD);

  return (distW + distH + distD) / 3.0;
}

template<>
double compare(SemanticColor &a, SemanticColor &b)
{
  const std::string &colorsA = a.color.get();
  const std::string &colorsB = b.color.get();
  const float &ratiosA = a.ratio.get();
  const float &ratiosB = b.ratio.get();
  double dist = 0.0;

  if(colorsA == colorsB) {
    dist += fabs(ratiosA - ratiosB);
  }

  return dist /= 2;
}

template<>
double compare(ColorHistogram &a, ColorHistogram &b)
{
  cv::Mat histA, histB;
  rs::conversion::from(a.hist(), histA);
  rs::conversion::from(b.hist(), histB);

  return cv::compareHist(histA, histB, CV_COMP_HELLINGER);;
}

template<>
double compare(Features &a, Features &b)
{
  if(a.source() != b.source() || a.descriptorType() != b.descriptorType()) {
    outDebug("Features do not match.");
    return 1;
  }

  cv::Mat descA, descB;
  rs::conversion::from(a.descriptors(), descA);
  rs::conversion::from(b.descriptors(), descB);
  if(descA.rows == 0 || descB.rows == 0) {
    outDebug("one of the features is empty.");
    return 1;
  }

  cv::BFMatcher matcher;
  double maxDist = 0;
  if(a.descriptorType() == "binary") {
    matcher = cv::BFMatcher(cv::NORM_HAMMING, true);
    maxDist = (double)((descA.cols * descA.elemSize() * 8) >> 1); // theoretical maxium is number of bits, but the half is used as max
  }
  else {
    matcher = cv::BFMatcher(cv::NORM_L2, false);
    maxDist = 1.0; // theoretical max is 2 for normalized vectors, but the half is used as max
  }

  std::vector<cv::DMatch> matches;
  matcher.match(descA, descB, matches);
  if(matches.empty()) {
    outDebug("no matches found.");
    return 1.0;
  }

  double dist = 0;
  for(size_t i = 0; i < matches.size(); ++i) {
    const cv::DMatch &match = matches[i];
    dist += match.distance;
  }
  dist /= (matches.size() * maxDist);
  return std::min(dist, 1.0);
}

template<>
double compare(PclFeature &a, PclFeature &b)
{
  if(a.feat_type() != b.feat_type() && a.feature.size() != b.feature.size()) {
    outDebug("Features do not match.");
    return 1;
  }
  const std::vector<float> &valuesA = a.feature();
  const std::vector<float> &valuesB = b.feature();

  cv::Mat featA(valuesA.size(), 1, CV_32F), featB(valuesB.size(), 1, CV_32F);
  std::memcpy(featA.data, &valuesA[0], valuesA.size() * sizeof(float));
  std::memcpy(featB.data, &valuesB[0], valuesB.size() * sizeof(float));

  featA = featA / std::sqrt(featA.dot(featA));
  featB = featB / std::sqrt(featB.dot(featB));

  cv::Mat res = featA - featB;
  return std::sqrt(res.dot(res)) / 2.0f;
}

template<>
double compare(Detection &a, Detection &b)
{
  if(a.source() != b.source()) {
    outDebug("Features do not match.");
    return 1;
  }
  if(a.name() == b.name()) {
    return 0.0;
  }
  return 1.0;
}

}
