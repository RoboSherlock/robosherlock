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

#include <vector>
#include <thread>
#include <mutex>

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// ROS
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/image_encodings.h>

// RS
#include <rs/DrawingAnnotator.h>

namespace rs
{

class Visualizer
{
private:
  const std::string windowImage;
  const std::string windowCloud;

  DrawingAnnotator *annotator;
  std::vector<std::string> names;
  std::vector<std::string> activeAnnotators;
  size_t index;

  std::thread imageViewerThread;
  std::thread cloudViewerThread;
  std::mutex lock;

  bool running;
  bool updateImage;
  bool updateCloud;
  bool changedAnnotator;

  bool save;
  size_t saveFrameImage;
  size_t saveFrameCloud;
  std::string savePath;
  std::vector<int> saveParams;

  std_msgs::Header header;
  ros::NodeHandle nh;
  ros::Publisher pub;

public:
  static bool *trigger;

  Visualizer(const std::string &savePath);
  ~Visualizer();

  bool start();
  void stop();
  void setActiveAnnotators(std::vector<std::string> annotators);
  void nextAnnotator();
  void prevAnnotator();
  bool selectAnnotator(std::string annotator);

private:
  static void callbackMouse(const int event, const int x, const int y, const int flags, void *object);
  void callbackMouseHandler(const int event, const int x, const int y);
  void callbackKeyHandler(const char key, const DrawingAnnotator::Source source);

  void checkAnnotator();
  void shutdown();

  void imageViewer();
  void cloudViewer();

  void keyboardEventImageViewer(const cv::Mat &disp);
  void keyboardEventCloudViewer(const pcl::visualization::KeyboardEvent &event, void *);

  void saveImage(const cv::Mat &disp);
  void saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::visualization::PCLVisualizer::Ptr &visualizer);
};

}
