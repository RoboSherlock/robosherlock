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
#include <robosherlock_msgs/RSActiveAnnotatorList.h>
#include <robosherlock_msgs/RSVisControl.h>

// RS
#include <rs/DrawingAnnotator.h>
#include "VisualizerAnnotatorManager.h"

namespace rs
{

class Visualizer
{
private:
  std::string aeName_;


  const std::string windowImage;
  const std::string windowCloud;

//  DrawingAnnotator *annotator;
//  std::vector<std::string> names;
//  std::vector<std::string> activeAnnotators;
//  size_t index;

  std::thread imageViewerThread;
  std::thread cloudViewerThread;
  std::mutex lock;

  bool running;
//  bool updateImage;
//  bool updateCloud;
//  bool changedAnnotator;

  bool save, headless_;
  size_t saveFrameImage;
  size_t saveFrameCloud;
  std::string savePath;
  std::vector<int> saveParams;

  std_msgs::Header header;
  ros::NodeHandle nh_;
  ros::Publisher pub, pubAnnotList;
  ros::ServiceServer vis_service_;

  // drawingAnnotators handled by this class
//  std::map<std::string, DrawingAnnotator *> drawingAnnotators;
  VisualizerAnnotatorManager visualizerAnnotatorManager_;

public:
  static bool *trigger;

  Visualizer(bool headless, std::string aeName = std::string());
  ~Visualizer();

  bool start();
  void stop();

  // TODO change this interfaces so it doesn't break the current api
  // RS ProcessManager is calling this from the outside
  void setActiveAnnotators(std::vector<std::string> annotators);
  std::string nextAnnotator();
  std::string prevAnnotator();
  std::string selectAnnotator(std::string annotator);

  // This method will copy all pointers p from DrawingAnnotator::annotators
  // into this class' drawingAnnotators property.
  // DrawingAnnotator::annotators will be emptied after the copy process
  //
  // Returns: The number of elements copied
//  int consumeRecentDrawingAnnotators();

private:
  static void callbackMouse(const int event, const int x, const int y, const int flags, void *object);
  void callbackMouseHandler(const int event, const int x, const int y);
  void callbackKeyHandler(const char key, const DrawingAnnotator::Source source);

//  void checkAnnotator();
  void shutdown();

  void imageViewer();
  void cloudViewer();

  void keyboardEventImageViewer(const cv::Mat &disp);
  void keyboardEventCloudViewer(const pcl::visualization::KeyboardEvent &event, void *);

  void saveImage(const cv::Mat &disp);
  void saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::visualization::PCLVisualizer::Ptr &visualizer);

  bool visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
      robosherlock_msgs::RSVisControl::Response &res);

//  void getAnnotatorNames(std::vector<std::string> &names);
//  DrawingAnnotator *getAnnotator(const std::string &name);

};

}
