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

#include <math.h>
#include <ostream>

#include <rs/utils/output.h>
#include <rs/segmentation/ImageSegmentation.h>

ImageSegmentation::ImageSegmentation()
{
}

ImageSegmentation::~ImageSegmentation()
{
}

void ImageSegmentation::thresholding(const cv::Mat &grey, cv::Mat &bin, const double threshold, const int method, const cv::Rect roi)
{
  if(roi.area() == 0)
  {
    cv::threshold(grey, bin, threshold, 255, method);
  }
  else
  {
    cv::threshold(grey(roi), bin, threshold, 255, method);
  }
}

void ImageSegmentation::segment(const cv::Mat &bin, std::vector<Segment> &segments, const size_t minSize, const size_t minHoleSize, const cv::Rect &roi)
{
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  segments.clear();

  cv::findContours(bin.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  for(int32_t i = 0; i < contours.size(); ++i)
  {
    Segment seg;
    const int32_t parent = hierarchy[i][3];
    if(parent >= 0)
    {
      // not a root segment
      continue;
    }
    if(toSegment(contours, hierarchy, i, seg, minSize, minHoleSize, roi))
    {
      segments.push_back(seg);
    }
  }
}

bool ImageSegmentation::toSegment(const std::vector<std::vector<cv::Point>> &contours, const std::vector<cv::Vec4i> &hierarchy, const size_t index,
                                  Segment &seg, const size_t minSize, const size_t minHoleSize, const cv::Rect &roi)
{
  seg.rect = cv::boundingRect(contours[index]);
  if(!roi.contains(seg.rect.tl()) || !roi.contains(seg.rect.br()))
  {
    // outside ROI
    return false;
  }

  seg.area = cv::contourArea(contours[index]);
  seg.childrenArea = 0;
  seg.holes = 0;

  if(seg.area < minSize)
  {
    return false;
  }

  Segment child;
  for(int32_t childIndex = hierarchy[index][2]; childIndex >= 0; childIndex = hierarchy[childIndex][0])
  {
    if(toSegment(contours, hierarchy, childIndex, child, minHoleSize, minHoleSize, roi))
    {
      seg.children.push_back(child);
      seg.area -= child.area;
      seg.childrenArea += child.area;
      ++seg.holes;
    }
  }

  if(seg.area < minSize)
  {
    return false;
  }

  seg.contour = contours[index];
  computeMoments(contours, hierarchy, index, seg);
  compute2DAttributes(seg);

  return true;
}

void ImageSegmentation::computeMoments(const std::vector<std::vector<cv::Point>> &contours, const std::vector<cv::Vec4i> &hierarchy, const size_t index, Segment &seg)
{
  cv::Mat bin;

  // Compute bounding image
  bin = cv::Mat::zeros(seg.rect.size(), CV_8U);

  // Draw contour and holes
  cv::drawContours(bin, contours, index, CV_RGB(255, 255, 255), CV_FILLED, 8, hierarchy, 0, -seg.rect.tl());
  for(int32_t child = hierarchy[index][2]; child >= 0; child = hierarchy[child][0])
  {
    cv::drawContours(bin, contours, child, CV_RGB(0, 0, 0), CV_FILLED, 8, hierarchy, 0, -seg.rect.tl());
  }

  // Compute moments
  seg.moments = cv::moments(bin, true);
  cv::HuMoments(seg.moments, seg.huMoments);
}

void ImageSegmentation::compute2DAttributes(Segment &seg)
{
  double x = seg.moments.m10 / seg.moments.m00;
  double y = seg.moments.m01 / seg.moments.m00;
  seg.center.x = seg.rect.x + x;
  seg.center.y = seg.rect.y + y;

  seg.alpha = 0.5 * atan2(2 * seg.moments.mu11, seg.moments.mu20 - seg.moments.mu02);

  double sinAlpha = sin(seg.alpha);
  double cosAlpha = cos(seg.alpha);

  double xx = (seg.moments.m20 / seg.moments.m00) - (x * x);
  double xy = (seg.moments.m11 / seg.moments.m00) - (x * y);
  double yy = (seg.moments.m02 / seg.moments.m00) - (y * y);

  double sin2 = sinAlpha * sinAlpha;
  double cos2 = cosAlpha * cosAlpha;
  double cs2xy = 2 * sinAlpha * cosAlpha * xy;
  double lengthX = 2 * sqrt(cos2 * xx + cs2xy + sin2 * yy);
  double lengthY = 2 * sqrt(sin2 * xx - cs2xy + cos2 * yy);

  cv::Point2d axisX = cv::Point2d(cosAlpha, sinAlpha);
  cv::Point2d axisY = cv::Point2d(-sinAlpha, cosAlpha);

  const double distance = axisX.dot(seg.center);
  double maxPosDist = 0;
  double maxNegDist = 0;
  for(size_t i = 0; i < seg.contour.size(); ++i)
  {
    cv::Point2d p(seg.contour[i].x, seg.contour[i].y);
    double dist = axisX.dot(p) - distance;

    if(dist < maxNegDist)
    {
      maxNegDist = dist;
    }
    if(dist > maxPosDist)
    {
      maxPosDist = dist;
    }
  }
  const double maxDist = maxPosDist > -maxNegDist ? maxPosDist : maxNegDist;
  const double diffDist = fabs(maxPosDist + maxNegDist);

  seg.axisX = axisX * lengthX;
  seg.axisY = axisY * lengthY;
  // let main axis always be positive in x direction if the negative and positive max distance from y axis are similar, or let it direct to the longer side.
  if((diffDist < 25.0 && axisX.x < 0) || (diffDist >= 25.0 && maxDist < 0.0))
  {
    seg.axisX = -seg.axisX;
    seg.axisY = -seg.axisY;
  }
}

void ImageSegmentation::computePose(std::vector<Segment> &segments, const cv::Mat &cameraMatrix,
                                    const cv::Mat &distCoefficients, const cv::Mat &planeNormal,
                                    const double planeDistance)
{
  if(segments.empty())
  {
    return;
  }

  std::vector<cv::Point2d> pointsImg, pointsUndist;
  pointsImg.resize(segments.size() * 3);

  for(size_t i = 0, j = 0; i < segments.size(); ++i)
  {
    Segment &seg = segments[i];
    pointsImg[j++] = seg.center;
    pointsImg[j++] = seg.center + seg.axisX;
    pointsImg[j++] = seg.center + seg.axisY;
  }
  cv::undistortPoints(pointsImg, pointsUndist, cameraMatrix, distCoefficients);

  for(size_t i = 0, j = 0; i < segments.size(); ++i)
  {
    Segment &seg = segments[i];
    cv::Mat axisX3D, axisY3D;
    backProject(planeNormal, planeDistance, pointsUndist[j++], seg.translation);
    backProject(planeNormal, planeDistance, pointsUndist[j++], axisX3D);
    backProject(planeNormal, planeDistance, pointsUndist[j++], axisY3D);

    axisX3D = axisX3D - seg.translation;
    axisY3D = axisY3D - seg.translation;
    seg.lengthX = sqrt(axisX3D.dot(axisX3D));
    seg.lengthY = sqrt(axisY3D.dot(axisY3D));
    axisX3D = axisX3D / seg.lengthX;
    double &aX = axisX3D.at<double>(0);
    double &aY = axisX3D.at<double>(1);
    double &aZ = axisX3D.at<double>(2);
    const double &nX = planeNormal.at<double>(0);
    const double &nY = planeNormal.at<double>(1);
    const double &nZ = planeNormal.at<double>(2);
    if(abs(nX) > abs(nY) && abs(nX) > abs(nZ))
    {
      aX = (-(nY * aY) - (nZ * aZ)) / nX;
    }
    else if(abs(nY) > abs(nZ))
    {
      aY = (-(nX * aX) - (nZ * aZ)) / nY;
    }
    else
    {
      aZ = (-(nX * aX) - (nY * aY)) / nZ;
    }
    axisY3D = -axisX3D.cross(planeNormal);

    seg.rotation = cv::Mat(3, 3, CV_64F);
    seg.rotation.at<double>(0, 0) = axisX3D.at<double>(0);
    seg.rotation.at<double>(1, 0) = axisX3D.at<double>(1);
    seg.rotation.at<double>(2, 0) = axisX3D.at<double>(2);
    seg.rotation.at<double>(0, 1) = axisY3D.at<double>(0);
    seg.rotation.at<double>(1, 1) = axisY3D.at<double>(1);
    seg.rotation.at<double>(2, 1) = axisY3D.at<double>(2);
    seg.rotation.at<double>(0, 2) = planeNormal.at<double>(0);
    seg.rotation.at<double>(1, 2) = planeNormal.at<double>(1);
    seg.rotation.at<double>(2, 2) = planeNormal.at<double>(2);

  }
}

void ImageSegmentation::backProject(const cv::Mat &planeNormal, const double planeDistance, const cv::Point2d &pointImage, cv::Mat &pointWorld)
{
  pointWorld = cv::Mat(3, 1, CV_64F);

  pointWorld.at<double>(0) = pointImage.x;
  pointWorld.at<double>(1) = pointImage.y;
  pointWorld.at<double>(2) = 1;
  double t = -planeDistance / planeNormal.dot(pointWorld);
  pointWorld = pointWorld * t;
}

void ImageSegmentation::drawSegments2D(cv::Mat &disp, const std::vector<Segment> &segments, const std::vector<std::string> &text,
                                       const int sizeLine, const double sizeText, const int font)
{
  std::ostringstream oss;

  for(int32_t i = 0; i < segments.size(); ++i)
  {
    const Segment &seg = segments[i];
    drawSegment(disp, CV_RGB(128, 128, 128), CV_RGB(0, 0, 0), seg, 0, 1, true, sizeLine);

    cv::rectangle(disp, seg.rect, CV_RGB(0, 0, 255), sizeLine);
    cv::circle(disp, seg.center, 5, CV_RGB(255, 255, 255), sizeLine, CV_AA);

    cv::line(disp, seg.center, seg.center + seg.axisX, CV_RGB(255, 0, 0), sizeLine);
    cv::line(disp, seg.center, seg.center + seg.axisY, CV_RGB(0, 255, 0), sizeLine);
  }

  for(int32_t i = 0; i < text.size(); ++i)
  {
    const Segment &seg = segments[i];

    oss.str("");
    oss << i << " : " << text[i];
    int baseLine;
    cv::Size textSize = cv::getTextSize(oss.str(), font, sizeText, sizeLine, &baseLine);
    cv::putText(disp, oss.str(), cv::Point(seg.rect.x + (seg.rect.width - textSize.width) / 2, seg.rect.y - textSize.height), font, sizeText, CV_RGB(255, 255, 255), sizeLine, CV_AA);
  }
}

void ImageSegmentation::drawSegments3D(cv::Mat &disp, const std::vector<Segment> &segments, const cv::Mat &cameraMatrix, const cv::Mat &distCoefficients,
                                       const std::vector<std::string> &text, const int sizeLine, const double sizeText, const int font)
{
  std::ostringstream oss;

  for(int32_t i = 0; i < segments.size(); ++i)
  {
    const Segment &seg = segments[i];
    drawSegment(disp, CV_RGB(255, 255, 255), CV_RGB(0, 0, 0), seg, 0, 1, false, sizeLine);

    std::vector<cv::Point2f> pointsImage;
    std::vector<cv::Point3f> axis(4);
    axis[0] = cv::Point3f(0, 0, 0);
    axis[1] = cv::Point3f(1, 0, 0) * seg.lengthX;
    axis[2] = cv::Point3f(0, 1, 0) * seg.lengthY;
    axis[3] = cv::Point3f(0, 0, 1) * 0.02;

    cv::projectPoints(axis, seg.rotation, seg.translation, cameraMatrix, distCoefficients, pointsImage);

    //draw the axes on the colored image
    cv::line(disp, pointsImage[0], pointsImage[1], CV_RGB(255, 0, 0), sizeLine, CV_AA);
    cv::line(disp, pointsImage[0], pointsImage[2], CV_RGB(0, 255, 0), sizeLine, CV_AA);
    cv::line(disp, pointsImage[0], pointsImage[3], CV_RGB(0, 0, 255), sizeLine, CV_AA);
  }

  for(int32_t i = 0; i < text.size(); ++i)
  {
    const Segment &seg = segments[i];

    oss.str("");
    oss << i << " : " << text[i];
    int baseLine;
    cv::Size textSize = cv::getTextSize(oss.str(), font, sizeText, sizeLine, &baseLine);
    cv::putText(disp, oss.str(), cv::Point(seg.rect.x + (seg.rect.width - textSize.width) / 2, seg.rect.y - textSize.height), font, sizeText, CV_RGB(255, 255, 255), sizeLine, CV_AA);
  }
}

void ImageSegmentation::drawSegment(cv::Mat &disp, const cv::Scalar &colorOut, const cv::Scalar &colorIn, const Segment &seg, const int32_t level, const int32_t levels, const bool fill, const int lineSize, const cv::Point &offset)
{
  cv::Scalar color = level % 2 ? colorIn : colorOut;

  std::vector<std::vector<cv::Point>> tmp;
  tmp.push_back(seg.contour);
  if(fill)
  {
    cv::fillPoly(disp, tmp, color, 8, 0, offset);
  }
  else
  {
    cv::polylines(disp, tmp, true, color, lineSize, CV_AA);
  }

  // Stop after first inner contour
  if(level == levels)
  {
    return;
  }

  for(size_t i = 0; i < seg.children.size(); ++i)
  {
    drawSegment(disp, colorOut, colorIn, seg.children[i], level + 1, levels, fill, lineSize, offset);
  }
}
