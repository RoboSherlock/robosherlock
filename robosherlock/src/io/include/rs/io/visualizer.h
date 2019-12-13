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

/*
 * This visualizer can be run with only one AAE running or also in MultiAAE setups.
 * Please note that the MultiAAE features are new and experimental.
 * If you want to use them, instantiate the RSVisualizer with multiAAEVisualizer=true and
 * add the DrawingAnnotators per Engien with the TODO method.
 */
class Visualizer
{
private:
//  std::string aeName_;

//  const std::string windowImage;
//  const std::string windowCloud;

  std::thread imageViewerThread;
  std::thread cloudViewerThread;
  std::mutex lock;

  bool running;
  bool multiAAEVisualizer_;

  bool save, headless_;
  size_t saveFrameImage;
  size_t saveFrameCloud;
  std::string savePath;
  std::vector<int> saveParams;

  std_msgs::Header header;
  ros::NodeHandle nh_;
  ros::Publisher pub, pubAnnotList;
  ros::ServiceServer vis_service_;

  std::map<std::string, std::shared_ptr<VisualizerAnnotatorManager>> visualizerAnnotatorManagers_;

public:
  static bool *trigger;

  Visualizer(bool headless, bool multiAAEVisualizer=false);
  ~Visualizer();

  bool start();
  void stop();

  // TODO change this interfaces so it doesn't break the current api
  // RS ProcessManager is calling this from the outside
  // Note: When running in MultiAAE mode, this will only affect the first
  // AAE (or in more detail: rs::Visualizer::visualizerAnnotatorManagers_.begin().second
  void setActiveAnnotators(std::vector<std::string> annotators);
  std::string nextAnnotator();
  std::string prevAnnotator();
  std::string selectAnnotator(std::string annotator);

  // Add a Visualizer for a given identifier.
  // Please do this directly after creating a new RSAAE.
  // This create an VisualizerAnnotatorManager that takes care of the drawing state
  // of the different AAEs that might be run.
  //
  // @param identifier Is right now the name of the AAE. In the future, we want to extend this to a more general
  //                   concept of Drawing classes so not only annotators can draw into Visualizers
  void addVisualizerManager(std::string identifier);

private:
  static void callbackMouse(const int event, const int x, const int y, const int flags, void *object);
  void callbackMouseHandler(const int event, const int x, const int y);

  //
  // @param activeVAM The Associated VisualizerAnnotatorManager in which window the detected key has been pressed
  void callbackKeyHandler(const char key, const DrawingAnnotator::Source source, std::shared_ptr<VisualizerAnnotatorManager> activeVAM);

  void shutdown();

  void imageViewer();
  void cloudViewer();

  void keyboardEventImageViewer(const cv::Mat &disp);
  void keyboardEventCloudViewer(const pcl::visualization::KeyboardEvent &event, void *);

  void saveImage(const cv::Mat &disp);
  void saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::visualization::PCLVisualizer::Ptr &visualizer);

  bool visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
      robosherlock_msgs::RSVisControl::Response &res);

  inline const std::string imageWindowName(VisualizerAnnotatorManager &vam){
    return vam.getAEName() + "/Image Viewer";
  }
  inline const std::string cloudWindowName(VisualizerAnnotatorManager &vam){
    return vam.getAEName() + "/Cloud Viewer";
  }

  // https://stackoverflow.com/a/478960
  std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
      throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
      result += buffer.data();
    }
    return result;
  }
  std::string getActiveWindowTitle();

  // Returns true if the active window could be mapped to a VisualizerAnnotatorManager.
  std::shared_ptr<VisualizerAnnotatorManager> getAnnotatorManagerForActiveWindow(bool &success, const DrawingAnnotator::Source windowType);


};

}
