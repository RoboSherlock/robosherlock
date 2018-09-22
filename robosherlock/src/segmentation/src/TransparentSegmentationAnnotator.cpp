/**
 * Copyright 2017 University of Bremen, Institute for Artificial Intelligence
 * Author(s):
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

#include <iterator>

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>

// CV
#include <opencv2/opencv.hpp>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

#include <rs/segmentation/TransparentSegmentationAnnotator.h>
#include <rs/segmentation/ImageSegmentation.h>


TyErrorId TransparentSegmentationAnnotator::initialize(AnnotatorContext &ctx)
{
  outInfo("initialize");

  ctx.extractValue("open_iterations", this->openIterations);
  ctx.extractValue("close_iterations", this->closeIterations);
  ctx.extractValue("gc_erode_iterations", this->gcErodeIterations);
  ctx.extractValue("gc_dilate_iterations", this->gcDilateIterations);
  ctx.extractValue("grabcut_iterations", this->grabcutIterations);
  ctx.extractValue("morph_kernel_size", this->morphKernelSize);
  ctx.extractValue("convex_hull_segments", this->convexHullSegments);
  ctx.extractValue("min_contour_area", minContourArea);

  return UIMA_ERR_NONE;
}

TyErrorId TransparentSegmentationAnnotator::destroy()
{
  outInfo("destroy");
  return UIMA_ERR_NONE;
}

TyErrorId TransparentSegmentationAnnotator::processWithLock(
  CAS &tcas, ResultSpecification const &res_spec)
{
  outInfo("process start");
  rs::SceneCas cas(tcas);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  if(!this->hasRGBDData)
  {
    cv::Mat casImageRGB;
    cv::Mat casImageDepth;

    if(!cas.get(VIEW_DEPTH_IMAGE, casImageDepth))
    {
      outError("No depth image");
      return UIMA_ERR_NONE;
    }

    if(!cas.get(VIEW_COLOR_IMAGE, casImageRGB))
    {
      outError("No color image");
      return UIMA_ERR_NONE;
    }
    if(!cas.get(VIEW_CLOUD, *cloud))
    {
      outError("No Point cloud in CAS: did you run an ImagepreProcesor?");
      return UIMA_ERR_NONE;
    }

    casImageDepth.convertTo(this->imageDepth, CV_8UC1, 1, 0);
    cv::threshold(this->imageDepth, this->imageDepth,
                  1, 255, cv::THRESH_BINARY_INV);
    cv::resize(casImageRGB, this->imageRGB, this->imageDepth.size());

    this->hasRGBDData = true;
    outInfo("Got RGBD input");
  }

  rs::Scene scene = cas.getScene();

  std::vector<rs::Plane> planes;
  scene.annotations.filter(planes);

  if(planes.empty())
  {
    outInfo("No plane found, deferring until second run");
    return UIMA_ERR_ANNOTATOR_MISSING_INFO;
  }

  rs::Plane &plane = planes[0];
  std::vector<float> planeModel = plane.model();

  if(planeModel.empty() || planeModel.size() != 4)
  {
    outInfo("Plane is incorrect");
    return UIMA_ERR_NONE;
  }
  else
  {
    sensor_msgs::CameraInfo cameraInfoMsg;
    cas.get(VIEW_CAMERA_INFO, cameraInfoMsg);

    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;
    std::tie(cameraMatrix, distortionCoefficients) =
      readCameraInfo(cameraInfoMsg);

    cv::Mat planeNormal(3, 1, CV_64F);
    planeNormal.at<double>(0) = planeModel[0];
    planeNormal.at<double>(1) = planeModel[1];
    planeNormal.at<double>(2) = planeModel[2];
    const double planeDistance = planeModel[3];

    this->searchMask = getPlaneMaskClosed(plane, cameraInfoMsg);

    cv::Mat depthFailedMask = (this->imageDepth == 255) & searchMask;

    preprocessFailedMask(depthFailedMask);

    auto roiMasks = getProbableCGRegions(depthFailedMask);

    cv::Mat refinedFailedMask(depthFailedMask.size(), CV_8UC1, cv::GC_BGD);

    for(auto & region : roiMasks)
    {
      gcRefineRegion(region, refinedFailedMask, this->imageRGB, searchMask);
    }

    refinedFailedMask = ((refinedFailedMask == cv::GC_FGD) |
                         (refinedFailedMask == cv::GC_PR_FGD));

    this->segments.clear();

    ImageSegmentation::segment(refinedFailedMask, this->segments,
                               this->minContourArea, this->minHoleSize,
                               cv::Rect(0, 0, refinedFailedMask.cols, refinedFailedMask.rows));
    ImageSegmentation::computePose(this->segments, cameraMatrix,
                                   distortionCoefficients, planeNormal, planeDistance);


    for(size_t i = 0; i < segments.size(); ++i)
    {
      ImageSegmentation::Segment &seg = segments[i];
      cv::Rect roi_hires(seg.rect.x * 2, seg.rect.y * 2, seg.rect.width * 2, seg.rect.height * 2);
      cv::Mat mask, mask_hires;

      mask = cv::Mat::zeros(seg.rect.height, seg.rect.width, CV_8U);
      ImageSegmentation::drawSegment(mask, CV_RGB(255, 255, 255), CV_RGB(0, 0, 0), seg, 0, 1, true, 4, -seg.rect.tl());

      cv::resize(mask, mask_hires, roi_hires.size(), 0, 0, cv::INTER_NEAREST);

      rs::ImageROI roi = rs::create<rs::ImageROI>(tcas);
      roi.roi_hires(rs::conversion::to(tcas, roi_hires));
      roi.roi(rs::conversion::to(tcas, seg.rect));
      roi.mask(rs::conversion::to(tcas, mask));
      roi.mask_hires(rs::conversion::to(tcas, mask_hires));

      rs::ReferenceClusterPoints points = rs::create<rs::ReferenceClusterPoints>(tcas);
      pcl::PointIndices indices;
      indices.header = cloud->header;

      for(int r = 0; r < mask.rows; ++r)
      {
        size_t index = (seg.rect.y + r) * cloud->width + seg.rect.x;
        const uint8_t *itM = mask.ptr<uint8_t>(r);
        const pcl::PointXYZRGBA *itP = &cloud->points[index];

        for(int c = 0; c < mask.cols; ++c, ++itM, ++itP, ++index)
        {
          if(*itM > 0 && itP->z > 0)
          {
            indices.indices.push_back(index);
          }
        }
      }

      outInfo("3D points for object: " << indices.indices.size());
      rs::PointIndices pIndices = rs::create<rs::PointIndices>(tcas);
      pIndices = rs::conversion::to(tcas, indices);
      points.indices(pIndices);

      rs::Cluster cluster = rs::create<rs::Cluster>(tcas);
      cluster.rois(roi);
      cluster.points(points);
      cluster.source.set("TransparentSegmentation");
      scene.identifiables.append(cluster);
    }

    // finish until next pipeline run
    hasRGBDData = false;
  }

  return UIMA_ERR_NONE;
}

using Contour = std::vector<cv::Point>;

void TransparentSegmentationAnnotator::drawImageWithLock(cv::Mat &disp)
{
  cv::Mat mask;
  cv::cvtColor(this->searchMask, mask, CV_GRAY2BGR);

  disp = this->imageRGB.mul(mask, 0.2 / 255) + this->imageRGB * 0.3;
  ImageSegmentation::drawSegments2D(disp, segments, {});
}

cv::Mat TransparentSegmentationAnnotator::getPlaneMaskClosed(
  rs::Plane &plane, sensor_msgs::CameraInfo const &cameraInfoMsg) const
{
  cv::Mat planeMask, localPlaneMask;
  cv::Rect planeROI;

  rs::conversion::from(plane.mask(), localPlaneMask);
  rs::conversion::from(plane.roi(), planeROI);

  planeMask = cv::Mat::zeros(this->imageDepth.size(), CV_8UC1);
  localPlaneMask.copyTo(planeMask(planeROI));

  std::vector<Contour> planeContours;
  std::vector<cv::Vec4i> contoursHierarchy;

  cv::findContours(planeMask, planeContours, contoursHierarchy,
                   CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  auto longestPlaneContourIt =
    std::max_element(
      planeContours.begin(), planeContours.end(),
      [](const Contour & a, const Contour & b)
  {
    return a.size() < b.size();
  });

  Contour hull;
  cv::convexHull(*longestPlaneContourIt, hull);

  std::vector<Contour> contourVec {hull};
  cv::fillPoly(planeMask, contourVec, cv::Scalar(255));

  return planeMask;
}

void TransparentSegmentationAnnotator::preprocessFailedMask(
  cv::Mat &failedDepthMask) const
{
  cv::Size mKernelSize(this->morphKernelSize, this->morphKernelSize);
  cv::Mat mKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, mKernelSize);

  cv::morphologyEx(failedDepthMask, failedDepthMask, cv::MORPH_CLOSE,
                   mKernel, cv::Point(-1, -1), this->closeIterations);
  cv::morphologyEx(failedDepthMask, failedDepthMask, cv::MORPH_OPEN,
                   mKernel, cv::Point(-1, -1), this->openIterations);

  std::vector<Contour> contours;
  cv::Mat contoursMat = failedDepthMask.clone();
  std::vector<Contour> hulls;
  cv::findContours(contoursMat, contours,
                   cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  for(auto & contour : contours)
  {
    std::vector<Contour> contourVec;

    if(this->convexHullSegments)
    {
      Contour hull;
      cv::convexHull(contour, hull);
      contourVec.push_back(hull);
    }
    else
    {
      contourVec.push_back(contour);
    }

    cv::fillPoly(failedDepthMask, contourVec, cv::Scalar(255));
  }
}

std::vector<cv::Mat> TransparentSegmentationAnnotator::getProbableCGRegions(
  cv::Mat const &src) const
{
  cv::Size mKernelSize(this->morphKernelSize, this->morphKernelSize);
  cv::Mat mKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, mKernelSize);

  cv::Mat eroded, dilated;
  auto mkoffset = cv::Point(-1, -1);
  cv::erode(src, eroded, mKernel, mkoffset, this->gcErodeIterations);
  cv::dilate(src, dilated, mKernel, mkoffset, this->gcDilateIterations);

  std::vector<Contour> regionContours;
  cv::Mat contoursMat = src.clone();
  cv::findContours(contoursMat, regionContours,
                   cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  std::vector<cv::Rect> rois;
  std::vector<cv::Mat> roiMasks;
  for(auto & contour : regionContours)
  {
    if(cv::contourArea(contour) < this->minContourArea)
    {
      continue;
    }

    constexpr int kMargin = 20;
    cv::Rect roi = cv::boundingRect(contour);
    roi.x = std::max(0, roi.x - kMargin);
    roi.y = std::max(0, roi.y - kMargin);
    roi.width = std::min(src.cols - roi.x, roi.width + 2 * kMargin);
    roi.height = std::min(src.rows - roi.y, roi.height + 2 * kMargin);

    cv::Mat outRegionMask(src.size(), CV_8UC1, cv::GC_BGD);
    outRegionMask(roi).setTo(cv::GC_PR_BGD, dilated(roi));
    outRegionMask(roi).setTo(cv::GC_PR_FGD, src(roi));
    outRegionMask(roi).setTo(cv::GC_FGD, eroded(roi));

    roiMasks.push_back(outRegionMask(roi));
  }

  return roiMasks;
}

void TransparentSegmentationAnnotator::gcRefineRegion(
  cv::Mat const &gcRegionMask, cv::Mat &refinedMask,
  cv::Mat const &rgb, cv::Mat const &searchMask) const
{
  cv::Size ROISize;
  cv::Point ROIOffset;

  gcRegionMask.locateROI(ROISize, ROIOffset);
  cv::Rect roi(ROIOffset, gcRegionMask.size());

  cv::Mat backgroundModel;
  cv::Mat foregroundModel;

  cv::grabCut(rgb(roi), gcRegionMask, cv::Rect(),
              backgroundModel, foregroundModel,
              this->grabcutIterations, cv::GC_INIT_WITH_MASK);

  cv::Mat refinedMaskROI = refinedMask(roi);
  cv::Mat copyMask = (gcRegionMask != cv::GC_BGD) &
                     (gcRegionMask != cv::GC_PR_BGD) &
                     searchMask(roi);

  gcRegionMask.copyTo(refinedMaskROI, copyMask);
}

std::tuple<cv::Mat, cv::Mat> TransparentSegmentationAnnotator::readCameraInfo(
  sensor_msgs::CameraInfo const &cameraInfoMsg)
{
  cv::Mat cameraMatrix(3, 3, CV_64F);

  double *it = cameraMatrix.ptr<double>(0);
  for(size_t i = 0; i < 8; ++i, ++it)
  {
    *it = cameraInfoMsg.K[i];
  }

  cv::Mat distortionCoefficients(1, cameraInfoMsg.D.size(), CV_64F);

  it = distortionCoefficients.ptr<double>(0);
  for(size_t i = 0; i < cameraInfoMsg.D.size(); ++i, ++it)
  {
    *it = cameraInfoMsg.D[i];
  }

  return std::tie(cameraMatrix, distortionCoefficients);
}
