/* Copyright (c) 2013, Thiemo Wiedemeyer  <wiedemeyer@informatik.uni-bremen.de>
 * All rights reserved.
 */

#pragma once
#ifndef __LINEMODINTERFACE_H__
#define __LINEMODINTERFACE_H__

// System
#include <vector>
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

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
               const std::vector<std::string> &classes = std::vector<std::string>(), const cv::Mat &mask = cv::Mat());

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
