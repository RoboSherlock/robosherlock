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
//#include <rs/DrawingAnnotator.h>
#include <rs/io/Visualizable.h>

namespace rs
{

class VisualizerAnnotatorManager
{
  // TODO change to private and make Visualizer a friend
private:


  std::string identifier_;

  Visualizable *currentVisualizable;


  std::vector<std::string> names;
  std::vector<std::string> activeVisualizables;
  size_t index;

  std::mutex lock;

  bool running;


  bool save, headless_;
  size_t saveFrameImage;
  size_t saveFrameCloud;
  std::string savePath;
//  std::vector<int> saveParams;

  std_msgs::Header header;
  ros::NodeHandle nh_;
  ros::Publisher outputImagePub, pubAnnotList;
  ros::ServiceServer vis_service_;

  // drawingAnnotators handled by this class
  std::map<std::string, Visualizable *> visualizables;

  static bool *trigger;

public:
  VisualizerAnnotatorManager(bool headless, std::string identifier);
  ~VisualizerAnnotatorManager();

  bool start();
  void stop();
  void setActiveAnnotators(std::vector<std::string> visualizable);
  std::string nextAnnotator();
  std::string prevAnnotator();
  std::string selectAnnotator(std::string visualizable);

  // This method will copy all pointers p from DrawingAnnotator::annotators
  // into this class' drawingAnnotators property.
  // DrawingAnnotator::annotators will be emptied after the copy process
  //
  // Returns: The number of elements copied
  int consumeRecentVisualizables(); // TODO should be private
  void getAnnotatorNames(std::vector<std::string> &names); // TODO rename
  Visualizable *getAnnotator(const std::string &name); // TODO rename
  std::string getCurrentVisualizableName();

  void checkVisualizable();

  bool visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
      robosherlock_msgs::RSVisControl::Response &res);

  void callbackMouseHandler(const int event, const int x, const int y);

  const std::string &getIdentifier() const;
  Visualizable *getCurrentVisualizable() const;

  void publishOutputImage(cv::Mat &disp);

  bool updateImage;
  bool updateCloud;
  bool changedVisualizable;

};

}
