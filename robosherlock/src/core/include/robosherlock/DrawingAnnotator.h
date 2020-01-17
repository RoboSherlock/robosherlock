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

#ifndef __DRAWING_ANNOTATOR_H__
#define __DRAWING_ANNOTATOR_H__

#include <vector>
#include <map>
#include <string>
#include <mutex>

#include <uima/api.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <robosherlock/utils/output.h>
#include <robosherlock/io/Visualizable.h>

class DrawingAnnotator : public uima::Annotator, public Visualizable
{
public:
  typedef VisualizableDataType Source; // This is necessary in order to
                                       // don't break the Key callback signature in annotators

  bool hasRun = false;

private:

  std::mutex drawLock;

public:
  DrawingAnnotator(const std::string &name);

  uima::TyErrorId process(uima::CAS &tcas, uima::ResultSpecification const &res_spec);

  void drawImage(cv::Mat &disp);
  bool fillVisualizer(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun);

protected:
  virtual uima::TyErrorId processWithLock(uima::CAS &tcas, uima::ResultSpecification const &res_spec) = 0;

  virtual void drawImageWithLock(cv::Mat &disp);
  virtual void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun);
};

#endif //__DRAWING_ANNOTATOR_H__
