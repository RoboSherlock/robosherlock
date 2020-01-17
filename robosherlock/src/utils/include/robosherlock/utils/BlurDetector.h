/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
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

#ifndef __BLUR_DETECTOR_H__
#define __BLUR_DETECTOR_H__

// STL
#include <vector>
#include <list>
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

class BlurDetector
{
private:
  const double threshold;
  const size_t maxHist;
  const size_t minHist;
  const size_t waitBlur;

  std::list<double> history;
  double sum;
  double avg;
  int wasBlurred;

  const size_t maxStatSize;

public:
  static std::list<double> results;
  static std::list<bool> isBlurred;

  BlurDetector();

  static double funcLaplaceSum(const cv::Mat &img);

  static double funcLaplaceMean(const cv::Mat &img);

  static double funcLaplaceStdDev(const cv::Mat &img);

  static double funcSobelSum(const cv::Mat &img);

  static double funcSobelMean(const cv::Mat &img);

  static double funcSobelStdDev(const cv::Mat &img);

  static double funcSobelStdDevOptimized(const cv::Mat &img);

  static double funcModifiedLaplace(const cv::Mat &img);

  static double funcTenengrad(const cv::Mat &img);

  static double funcNormalizedGraylevelVariance(const cv::Mat &img);

  bool detectBlur(const cv::Mat &image);

  bool detectBlur(const double result);
};

#endif // __BLUR_DETECTOR_H__
