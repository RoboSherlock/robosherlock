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

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>

// CV
#include <opencv2/opencv.hpp>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

#include <rs/segmentation/ImageSegmentation.h>


using namespace uima;

class TransparentSegmentationAnnotator : public DrawingAnnotator
{
  friend class TransparentSegmentationUnitTest;

public:
  TransparentSegmentationAnnotator() : DrawingAnnotator(__func__)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx) override;
  TyErrorId destroy() override;

  TyErrorId processWithLock(CAS &tcas,
                            ResultSpecification const &res_spec) override;

protected:
  void drawImageWithLock(cv::Mat &disp) override;

  cv::Mat getPlaneMaskClosed(rs::Plane &plane,
      sensor_msgs::CameraInfo const &cameraInfoMsg) const;

  void preprocessFailedMask(cv::Mat &failedDepthMask) const;

  std::vector<cv::Mat> getProbableCGRegions(cv::Mat const &src) const;

  void gcRefineRegion(cv::Mat const &gcRegionMask, cv::Mat &refinedMask,
                      cv::Mat const &rgb, cv::Mat const &searchMask) const;

  static std::tuple<cv::Mat, cv::Mat> readCameraInfo(
      sensor_msgs::CameraInfo const &cameraInfoMsg);

private:
  int openIterations{6};
  int closeIterations{12};
  int gcErodeIterations{6};
  int gcDilateIterations{12};
  int grabcutIterations{2};
  int morphKernelSize{3};
  bool convexHullSegments{false};
  int minContourArea{100};
  int minHoleSize{50};

  bool hasRGBDData{false};
  cv::Mat imageRGB;
  cv::Mat imageDepth;
  cv::Mat searchMask;

  std::vector<ImageSegmentation::Segment> segments;
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(TransparentSegmentationAnnotator)