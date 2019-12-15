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

// PCL
#include <pcl/io/pcd_io.h>

// ROS
#include <cv_bridge/cv_bridge.h>

// RS
#include <rs/utils/output.h>
#include <rs/io/VisualizableGroupManager.h>

using namespace rs;

bool *VisualizableGroupManager::trigger = NULL;
// TODO remove headless

VisualizableGroupManager::VisualizableGroupManager(bool headless, std::string identifier) :
  identifier_(identifier),
  currentVisualizable(NULL), names(), index(0), running(false),
  save(false), headless_(headless), saveFrameImage(0), saveFrameCloud(0), nh_("~"),
  updateImage(true), updateCloud(true), changedVisualizable(true)
{
  this->savePath = std::string(getenv("USER")) +"./ros/";
  if(this->savePath[this->savePath.size() - 1] != '/')
  {
    this->savePath += '/';
  }
  vis_service_ = nh_.advertiseService(identifier_ + "/vis_command", &VisualizableGroupManager::visControlCallback, this);
}

VisualizableGroupManager::~VisualizableGroupManager()
{
    stop();
}

bool VisualizableGroupManager::start()
{
  outInfo("start with Identifier=" << identifier_);
  consumeRecentVisualizables(); // Claim the responsibility for all Visualizables in this VisualizableGroupManager

  getVisualizableNames(names);
  if(names.empty()) {
    outInfo("No visualizables do visualize. Aborting Visualizer start.");
    return false;
  }
  //Initially, all visualizables are active
  activeVisualizables = names;
//
  outputImagePub = nh_.advertise<sensor_msgs::Image>(identifier_ + "/output_image", 1, true);
  pubAnnotList = nh_.advertise<robosherlock_msgs::RSActiveAnnotatorList>(identifier_ +"/vis/active_annotators", 1, true);
  index = 0;

  currentVisualizable = getVisualizable(names[index]);

  running = true;
  return true;
}

void VisualizableGroupManager::stop()
{
  outInfo("stopping VisualizableGroupManager!");
  if(running) {
    running = false;
    pubAnnotList.shutdown();
  }
  outInfo("VisualizableGroupManager stopped!");
}

void VisualizableGroupManager::setActiveVisualizable(std::vector<std::string> annotators)
{
  if(!annotators.empty()){
  std::vector<std::string> activeDrawingAnnotators(names.size()); // TODO rename
  std::vector<std::string>::iterator it;
  std::sort(annotators.begin(), annotators.end());
  std::sort(names.begin(), names.end());
  it = std::set_intersection(annotators.begin(), annotators.end(), names.begin(), names.end(), activeDrawingAnnotators.begin());
  activeDrawingAnnotators.resize(it - activeDrawingAnnotators.begin());
  activeVisualizables = activeDrawingAnnotators;

  robosherlock_msgs::RSActiveAnnotatorList listMsg;
  listMsg.annotators = activeVisualizables;
  pubAnnotList.publish(listMsg);
  }

}


std::string VisualizableGroupManager::nextVisualizable()
{
  {
    std::lock_guard<std::mutex> lock_guard(lock);
    if(activeVisualizables.empty()) return "";
    index = (index + 1) % activeVisualizables.size();
//    annotator = DrawingAnnotator::getAnnotator(activeAnnotators[index]);
    currentVisualizable = getVisualizable(activeVisualizables[index]);
    currentVisualizable->update = false;
    updateImage = true;
    updateCloud = true;
    changedVisualizable = true;
  }
  outDebug("switching to annotator: " << activeVisualizables[index]);
  return activeVisualizables[index];
}

std::string VisualizableGroupManager::prevVisualizable()
{
  {
    std::lock_guard<std::mutex> lock_guard(lock);
    if(activeVisualizables.empty()) return "";
    index = (activeVisualizables.size() + index - 1) % activeVisualizables.size();
//    annotator = DrawingAnnotator::getAnnotator(activeAnnotators[index]);
    currentVisualizable = getVisualizable(activeVisualizables[index]);
    currentVisualizable->update = false;
    updateImage = true;
    updateCloud = true;
    changedVisualizable = true;
  }
  outDebug("switching to annotator: " << activeVisualizables[index]);
  return activeVisualizables[index];
}

std::string VisualizableGroupManager::selectVisualizable(std::string visualizable)
{
  std::lock_guard<std::mutex> lock_guard(lock);
  ptrdiff_t pos = distance(activeVisualizables.begin(), find(activeVisualizables.begin(), activeVisualizables.end(), visualizable));
  index = pos;
  if(index >= activeVisualizables.size())
    return "";
//  annotator = DrawingAnnotator::getAnnotator(activeAnnotators[index]);
  currentVisualizable = getVisualizable(activeVisualizables[index]);
  currentVisualizable->update = false;
  updateImage = true;
  updateCloud = true;
  changedVisualizable = true;
  outDebug("switching to annotator: " << activeVisualizables[index]);
  return activeVisualizables[index];
}


void VisualizableGroupManager::checkVisualizable()
{
  std::lock_guard<std::mutex> lock_guard(lock);
  if(currentVisualizable->update) {
    currentVisualizable->update = false;
    updateImage = true;
    updateCloud = true;
  }
}
bool VisualizableGroupManager::visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
                                                  robosherlock_msgs::RSVisControl::Response &res)
{
  std::string command = req.command;
  bool result = true;
  std::string activeVisualizable = "";

  if(command == "next")
    activeVisualizable = this->nextVisualizable();
  else if(command == "previous")
    activeVisualizable = this->prevVisualizable();
  else if(command != "")
    activeVisualizable = this->selectVisualizable(command);
  if(activeVisualizable == "")
    result = false;
  res.success = result;
  res.active_annotator = activeVisualizable;
  return result;
}

int VisualizableGroupManager::consumeRecentVisualizables() {
  int copiedElements = Visualizable::copyVisualizableList(visualizables);
  Visualizable::clearVisualizableList();
  return copiedElements;
}

void VisualizableGroupManager::getVisualizableNames(std::vector<std::string> &names) {
  std::map<std::string, Visualizable *>::const_iterator it;
  names.clear();
  names.reserve(visualizables.size());

  for(it = visualizables.begin(); it != visualizables.end(); ++it)
  {
    names.push_back(it->first);
  }
}

Visualizable *VisualizableGroupManager::getVisualizable(const std::string &name) {
  std::map<std::string, Visualizable *>::const_iterator it;
  it = visualizables.find(name);
  if(it != visualizables.end())
  {
    return it->second;
  }
  return NULL;
}

std::string VisualizableGroupManager::getCurrentVisualizableName(){
  return currentVisualizable->name;
}

const std::string &VisualizableGroupManager::getIdentifier() const {
  return identifier_;
}

Visualizable *VisualizableGroupManager::getCurrentVisualizable() const {
  return currentVisualizable;
}

void VisualizableGroupManager::publishOutputImage(cv::Mat &disp) {
  sensor_msgs::Image image_msg;
  cv_bridge::CvImage cv_image;
  cv_image.image = disp;
  cv_image.encoding = "bgr8";
  cv_image.toImageMsg(image_msg);
  outputImagePub.publish(image_msg);
}

void VisualizableGroupManager::callbackMouseHandler(const int event, const int x, const int y)
{
  try {
    bool needupdate_img = currentVisualizable->callbackMouse(event, x, y, Visualizable::VisualizableDataType::IMAGE_VIEWER);
    updateImage = needupdate_img | updateImage;
    updateCloud = needupdate_img | updateCloud;
  }
  catch(...) {
    outError("Exception in " << currentVisualizable->name << "::callbackMouse!");
  }
}