/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *         Patrick Mania <pmania@cs.uni-bremen.de>
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
#include <robosherlock/io/Visualizable.h>

namespace rs
{
/**
 * This class is a communicator between a single Visualizer and a group of Visualizables.
 * A group of visualizables could, for example, be all DrawingAnnotators in a single Pipeline/AAE.
 *
 * Visualizers must be unique, because GUI related actions are problematic to handle in multiple threads.
 * The logic which Visualizables should currently be shown or if updates have been occured are handled in this class.
 * Usually, for each AAE that wants to visualize stuff, an object of this class will be instantiated by the Visualizer.
 *
 */
class VisualizableGroupManager
{
private:
  std::string identifier_;

  Visualizable* currentVisualizable;

  std::vector<std::string> names; /** List of names with all Visualizables this class is responsible for */
  std::vector<std::string> activeVisualizables;
  size_t index;

  std::mutex lock;

  bool running;

  bool save;
  size_t saveFrameImage;
  size_t saveFrameCloud;
  std::string savePath;

  std_msgs::Header header;
  ros::NodeHandle nh_;
  ros::Publisher outputImagePub, pubAnnotList;
  ros::ServiceServer vis_service_;

  std::map<std::string, Visualizable*> visualizables; /** (The group of) Visualizables this class is responsible for */

  static bool* trigger;

  /**
   * This method will copy all pointers p from Visualizable::visualizables
   * into this class' drawingAnnotators property.
   * Visualizable::visualizables will be emptied after the copy process
   *
   * Returns: The number of elements copied
   * */
  int consumeRecentVisualizables();

  void getVisualizableNames(std::vector<std::string>& names);

  /**
   * Get Pointer to a Visualizable in this group by its name
   *
   * Returns: Pointer to the Visualizable with name == name or NULL if no Visualizable with the given name can be found.
   * */
  Visualizable* getVisualizable(const std::string& name);

public:
  VisualizableGroupManager(std::string identifier);
  ~VisualizableGroupManager();

  bool start();
  void stop();
  void setActiveVisualizable(std::vector<std::string> visualizable);
  std::string nextVisualizable();
  std::string prevVisualizable();
  std::string selectVisualizable(std::string visualizable);

  std::string getCurrentVisualizableName();

  void checkVisualizable();

  bool visControlCallback(robosherlock_msgs::RSVisControl::Request& req,
                          robosherlock_msgs::RSVisControl::Response& res);

  void callbackMouseHandler(const int event, const int x, const int y);

  const std::string& getIdentifier() const;
  Visualizable* getCurrentVisualizable() const;

  void publishOutputImage(cv::Mat& disp);

  bool updateImage;
  bool updateCloud;
  bool changedVisualizable;
};

}  // namespace rs
