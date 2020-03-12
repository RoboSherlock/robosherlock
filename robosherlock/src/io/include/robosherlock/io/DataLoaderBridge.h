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

#ifndef __DATA_LOADER_BRIDGE_H__
#define __DATA_LOADER_BRIDGE_H__

#include <thread>
#include <mutex>

//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/output.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/io/CamInterface.h>
#include <sensor_msgs/CameraInfo.h>

//OpenCV
#include <opencv2/opencv.hpp>

class DataLoaderBridge : public CamInterface
{
private:
  std::string path_to_cloud;
  std::string path_to_rgb;
  std::string path_to_depth;
  std::string path_to_viewpoint;

  std::vector<std::string> clouds;
  std::vector<std::string> images;
  std::vector<std::string> depths;
  std::vector<std::string> viewpoints;

  bool isLoop;

  bool isCloudFile;
  bool isRGBFile;
  bool isDepthFile;
  bool isViewpointFile;

  bool haveCloud;
  bool haveRGB;
  bool haveDepth;
  bool haveCameraInfo;
  bool haveViewpoint;


  cv::Mat color;
  cv::Mat depth;

  double depth_scaling_factor;

  int index_; // aka frameID, indexing for file list
  int data_size;

  double frameRate;
  sensor_msgs::CameraInfo cameraInfo;

  tf::StampedTransform viewpoint;

  bool done; // we are done reading files, no new data to be posted
  std::thread updateTimerThread;
  std::mutex updateLock;

  bool readConfig(const boost::property_tree::ptree &pt);
  bool getListFile(std::string &path, std::vector<std::string> &filenames, std::string &pattern, bool &isFile);
  bool checkConsistency();
  void updateTimerWorker(const std::chrono::milliseconds period);

public:
  DataLoaderBridge(const boost::property_tree::ptree &pt);
  ~DataLoaderBridge();

  bool setData(uima::CAS &tcas, uint64_t ts);
  inline void getColorImage(cv::Mat &c)
  {
    c = this->color.clone();
  }

  inline void getDepthImage(cv::Mat &d)
  {
    d = this->depth.clone();
  }
};


#endif //  __DATA_LOADER_BRIDGE_H__
