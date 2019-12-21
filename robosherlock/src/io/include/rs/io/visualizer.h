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
#include "VisualizableGroupManager.h"

namespace rs
{
/**
 * This class handles visualization for everything that is a Visualizable.
 * It creates two threads. One for openCV Image visualization and one for PCL.
 * Since multithreading in GUI handling is problematic, do not instantiate this class more than once.
 *
 * This visualizer can be run with only one AAE or also in MultiAAE setups.
 * Please note that the MultiAAE features are new and experimental.
 * If you want to use them, instantiate the RSVisualizer with multiAAEVisualizer=true.
 * After that, instantiate all Visualizables you want to group and then call this->addVisualizableGroupManager
 *
 */
class Visualizer
{
private:
  std::thread imageViewerThread;
  std::thread cloudViewerThread;
  std::mutex lock;

  bool running;
  bool multiAAEVisualizer_;

  bool save;
  bool saveImageToDisk;
  bool headless_;
  std::shared_ptr<VisualizableGroupManager> imageVgmToBeSaved;

  std::string saveVisualizerWithIdentifier;
  size_t saveFrameImage;
  size_t saveFrameCloud;
  std::string savePath;
  std::vector<int> saveParams;

  std_msgs::Header header;
  ros::NodeHandle nh_;
  ros::Publisher pub, pubAnnotList;

  /**
   * Map that relates an identifier (or in our case right now: the AAE name)
   * to the Manager that keeps track of a group of Visualizables
   */
  std::map<std::string, std::shared_ptr<VisualizableGroupManager>> visualizableGroupManagers_;

public:
  static bool* trigger;

  Visualizer(bool headless, bool multiAAEVisualizer = false);
  ~Visualizer();

  bool start();
  void stop();

  // TODO provide *Visualizables methods instead of *Annotators so it reflects the new API
  // TODO forward all calls to *Annotator to *Visualizables so the change in the  interface doesn't break the current
  // api RS ProcessManager is calling this from the outside Note: When running in MultiAAE mode, this will only affect
  // the first AAE (or in more detail: rs::Visualizer::visualizableGroupManagers_.begin().second
  void setActiveAnnotators(std::vector<std::string> annotators);
  std::string nextAnnotator();
  std::string prevAnnotator();
  std::string selectAnnotator(std::string annotator);

  /** Add a VisualizableGroupManager for a given identifier.
   * Only required in MultiAAE mode.
   *
   * Please use this method directly after creating a new RSAAE
   * or instantiating your Visualizables in any other way.
   * This create an VisualizableGroupManager that takes care of the drawing state
   * of the different AAEs or other visualizing entities that might be run.
   *
   * @param identifier A unique identifier. This is usually the name of your AAE when you only execute AAEs.   */
  void addVisualizableGroupManager(std::string identifier);

private:
  static void callbackMouse(const int event, const int x, const int y, const int flags, void* object);

  /**
   * This method will be called when the Visualizer detected a Keypress.
   *  @param activeVGM The Associated VisualizableGroupManager in which window the detected key has been pressed
   */
  void callbackKeyHandler(const char key, const Visualizable::VisualizableDataType source,
                          std::shared_ptr<VisualizableGroupManager> activeVGM);

  void shutdown();

  void imageViewer();
  void cloudViewer();

  void keyboardEventImageViewer(const cv::Mat& disp);
  void keyboardEventCloudViewer(const pcl::visualization::KeyboardEvent& event, void*);

  void saveImage(const cv::Mat& disp, std::shared_ptr<VisualizableGroupManager> vgm);
  void saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                 pcl::visualization::PCLVisualizer::Ptr& visualizer);

  inline const std::string imageWindowName(VisualizableGroupManager& vgm)
  {
    return vgm.getIdentifier() + "/Image Viewer";
  }
  inline const std::string cloudWindowName(VisualizableGroupManager& vgm)
  {
    return vgm.getIdentifier() + "/Cloud Viewer";
  }

  /**
   * This method will execute a command on the system and return the output of it as a string
   *
   * @param cmd Command to be executed
   * @return Output from stdout
   */
  std::string exec(const char* cmd);

  std::string getActiveWindowTitle();

  /**
   * This method tries to guess which window was currently active when the user interacted with it.
   * It will then return a pointer to the responsible VisualizableGroupManager so further actions can be
   * taken on the VisualizableGroup that was interacted with.
   * @param bool: Pass a reference to a bool. After execution it will be true if the active window could be determined
   * and the appropriate VGM has been found. false otherwise.
   */
  std::shared_ptr<VisualizableGroupManager>
  getAnnotatorManagerForActiveWindow(bool& success, const Visualizable::VisualizableDataType windowType);
};

}  // namespace rs
