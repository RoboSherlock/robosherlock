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

#pragma once
#ifndef __LINEMODINTERFACE_H__
#define __LINEMODINTERFACE_H__

// System
#include <vector>
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd/linemod.hpp>
#endif

struct Result
{
  std::string name;
  float
  response,
  rotation,
  scale;
  cv::Rect roi;

  // Only valid till next frame is processed
  const cv::linemod::Match *match;
};

class LinemodInterface
{
public:
  cv::Ptr<cv::linemod::Detector> detector;
  std::vector<cv::linemod::Match> matches;

  LinemodInterface();

  /*
   * Processes a frame
   */
  void process(const cv::Mat &color, const cv::Mat &depth, std::vector<Result> &results, const float minResponse = 85.0f,
               const std::vector<cv::String> &classes = std::vector<cv::String>(), const cv::Mat &mask = cv::Mat());

  /*
   * Reading models from resource directory
   */
  bool readModels(const std::string &resourcePath);

  /*
   * Reading templates of model
   */
  void readModel(const std::string &filename);

  /*
   * Writing models to resource directory
   */
  void writeModels(const std::string &resourcePath);

  /*
   * Writing templates of model
   */
  void writeModel(const std::string &filename, const std::vector<std::string> &classIds);

  /*
   * Draws a result to the image
   */
  void drawResult(cv::Mat &image, const Result &result);

  /*
   * Draws results to the image
   */
  void drawResults(cv::Mat &image, const std::vector<Result> &results);

private:
  /*
   * Sets the result structure
   */
  void setResult(const cv::linemod::Match &match, Result &result);
};

#endif //__LINEMODINTERFACE_H__
