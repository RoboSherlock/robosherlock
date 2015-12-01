/* Copyright (c) 2013, Thiemo Wiedemeyer  <wiedemeyer@informatik.uni-bremen.de>
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

  return std::min(1.0, aTrans.distance(bTrans));
}

template<>
double compare(TFLocation &a, TFLocation &b)
{
  if(a.frame_id() == b.frame_id() && a.reference_desc() == b.reference_desc())
  {
    return 0;
  }
  return 1;
}

template<>
double compare(Shape &a, Shape &b)
{
  if(a.shape() == b.shape())
  {
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
  const std::vector<std::string> &colorsA = a.color.get();
  const std::vector<std::string> &colorsB = b.color.get();
  const std::vector<float> &ratiosA = a.ratio.get();
  const std::vector<float> &ratiosB = b.ratio.get();
  double dist = 0.0;

  for(size_t i = 0; i < colorsA.size(); ++i)
  {
    const std::string &colorA = colorsA[i];
    for(size_t j = 0; j < colorsB.size(); ++j)
    {
      if(colorA == colorsB[j])
      {
        dist += fabs(ratiosA[i] - ratiosB[j]);
        break;
      }
    }
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
  if(a.extractor() != b.extractor() || a.descriptorType() != b.descriptorType())
  {
    outInfo("Features do not match.");
    return 1;
  }

  cv::Mat descA, descB;
  rs::conversion::from(a.descriptors(), descA);
  rs::conversion::from(b.descriptors(), descB);
  if(descA.rows == 0 || descB.rows == 0)
  {
    outInfo("one of the features is empty.");
    return 1;
  }

  cv::BFMatcher matcher;
  double maxDist = 0;
  if(a.descriptorType() == "binary")
  {
    matcher = cv::BFMatcher(cv::NORM_HAMMING, true);
    maxDist = (double)((descA.cols * descA.elemSize() * 8) >> 1);
  }
  else
  {
    matcher = cv::BFMatcher(cv::NORM_L2, false);
    maxDist = sqrt((double)(descA.cols * 255 * 255)) / 2.0;
  }

  std::vector<cv::DMatch> matches;
  matcher.match(descA, descB, matches);
  if(matches.empty())
  {
    outInfo("no matches found.");
    return 1;
  }

  double dist = 0;
  for(size_t i = 0; i < matches.size(); ++i)
  {
    const cv::DMatch &match = matches[i];
    dist += match.distance;
  }
  dist /= (matches.size() * maxDist);
  return std::min(dist, 1.0);
}

}
