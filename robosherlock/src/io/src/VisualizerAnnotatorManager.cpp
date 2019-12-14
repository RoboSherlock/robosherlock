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
#include <rs/io/VisualizerAnnotatorManager.h>

using namespace rs;

bool *VisualizerAnnotatorManager::trigger = NULL;
// TODO remove headless

VisualizerAnnotatorManager::VisualizerAnnotatorManager(bool headless, std::string identifier) :
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
  vis_service_ = nh_.advertiseService(identifier_ + "/vis_command", &VisualizerAnnotatorManager::visControlCallback, this);
}

VisualizerAnnotatorManager::~VisualizerAnnotatorManager()
{
    stop();
}

bool VisualizerAnnotatorManager::start()
{
  outInfo("start with Identifier=" << identifier_);
  consumeRecentVisualizables(); // Claim the responsibility for all DrawingAnnotators in this VisualizerAnnotatorManager

  getAnnotatorNames(names);
  if(names.empty()) {
    outInfo("No annotators do visualize. Aborting Visualizer start.");
    return false;
  }
  //Initially, all annotators are active
  activeVisualizables = names;
//
  outputImagePub = nh_.advertise<sensor_msgs::Image>(identifier_ + "/output_image", 1, true);
  pubAnnotList = nh_.advertise<robosherlock_msgs::RSActiveAnnotatorList>(identifier_ +"/vis/active_annotators", 1, true);
  index = 0;

  currentVisualizable = getAnnotator(names[index]);

  running = true;
  return true;
}

void VisualizerAnnotatorManager::stop()
{
  outInfo("stopping VisualizerAnnotatorManager!");
  if(running) {
    running = false;
//    imageViewerThread.join();
//    if(!headless_)
//      cloudViewerThread.join();
//    pub.shutdown();
    pubAnnotList.shutdown();
  }
  outInfo("VisualizerAnnotatorManager stopped!");
}

//void VisualizerAnnotatorManager::callbackMouse(const int event, const int x, const int y, const int flags, void *object)
//{
//  ((VisualizerAnnotatorManager *)object)->callbackMouseHandler(event, x, y);
//}

//void VisualizerAnnotatorManager::callbackMouseHandler(const int event, const int x, const int y)
//{
//  try {
//    bool needupdate_img = currentDrawingAnnotator->callbackMouse(event, x, y, DrawingAnnotator::IMAGE_VIEWER);
//    updateImage = needupdate_img | updateImage;
//    updateCloud = needupdate_img | updateCloud;
//  }
//  catch(...) {
//    outError("Exception in " << currentDrawingAnnotator->name << "::callbackMouse!");
//  }
//}
//
//void VisualizerAnnotatorManager::callbackKeyHandler(const char key, const DrawingAnnotator::Source source)
//{
//  // Catch space for triggering
//  if(key == ' ') {
//    if(trigger) {
//      *trigger = true;
//    }
//    return;
//  }
//  try {
//    bool needupdate_img = currentDrawingAnnotator->callbackKey(key, source);
//    updateImage = needupdate_img | updateImage;
//    updateCloud = needupdate_img | updateCloud;
//  }
//  catch(...) {
//    outError("Exception in " << currentDrawingAnnotator->name << "::callbackKey!");
//  }
//}

void VisualizerAnnotatorManager::setActiveAnnotators(std::vector<std::string> annotators)
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


std::string VisualizerAnnotatorManager::nextAnnotator()
{
  {
    std::lock_guard<std::mutex> lock_guard(lock);
    if(activeVisualizables.empty()) return "";
    index = (index + 1) % activeVisualizables.size();
//    annotator = DrawingAnnotator::getAnnotator(activeAnnotators[index]);
    currentVisualizable = getAnnotator(activeVisualizables[index]);
    currentVisualizable->update = false;
    updateImage = true;
    updateCloud = true;
    changedVisualizable = true;
  }
  outDebug("switching to annotator: " << activeVisualizables[index]);
  return activeVisualizables[index];
}

std::string VisualizerAnnotatorManager::prevAnnotator()
{
  {
    std::lock_guard<std::mutex> lock_guard(lock);
    if(activeVisualizables.empty()) return "";
    index = (activeVisualizables.size() + index - 1) % activeVisualizables.size();
//    annotator = DrawingAnnotator::getAnnotator(activeAnnotators[index]);
    currentVisualizable = getAnnotator(activeVisualizables[index]);
    currentVisualizable->update = false;
    updateImage = true;
    updateCloud = true;
    changedVisualizable = true;
  }
  outDebug("switching to annotator: " << activeVisualizables[index]);
  return activeVisualizables[index];
}

std::string VisualizerAnnotatorManager::selectAnnotator(std::string anno)
{
  std::lock_guard<std::mutex> lock_guard(lock);
  ptrdiff_t pos = distance(activeVisualizables.begin(), find(activeVisualizables.begin(), activeVisualizables.end(), anno));
  index = pos;
  if(index >= activeVisualizables.size())
    return "";
//  annotator = DrawingAnnotator::getAnnotator(activeAnnotators[index]);
  currentVisualizable = getAnnotator(activeVisualizables[index]);
  currentVisualizable->update = false;
  updateImage = true;
  updateCloud = true;
  changedVisualizable = true;
  outDebug("switching to annotator: " << activeVisualizables[index]);
  return activeVisualizables[index];
}


void VisualizerAnnotatorManager::checkVisualizable()
{
  std::lock_guard<std::mutex> lock_guard(lock);
  if(currentVisualizable->update) {
    currentVisualizable->update = false;
    updateImage = true;
    updateCloud = true;
  }
}
bool VisualizerAnnotatorManager::visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
    robosherlock_msgs::RSVisControl::Response &res)
{
  std::string command = req.command;
  bool result = true;
  std::string activeAnnotator = "";

  if(command == "next")
    activeAnnotator = this->nextAnnotator();
  else if(command == "previous")
    activeAnnotator = this->prevAnnotator();
  else if(command != "")
    activeAnnotator = this->selectAnnotator(command);
  if(activeAnnotator == "")
    result = false;
  res.success = result;
  res.active_annotator = activeAnnotator;
  return result;
}

int VisualizerAnnotatorManager::consumeRecentVisualizables() {
  int copiedElements = Visualizable::copyAnnotatorList(visualizables);
  Visualizable::clearAnnotatorList();
  return copiedElements;
}

void VisualizerAnnotatorManager::getAnnotatorNames(std::vector<std::string> &names) {
  std::map<std::string, Visualizable *>::const_iterator it;
  names.clear();
  names.reserve(visualizables.size());

  for(it = visualizables.begin(); it != visualizables.end(); ++it)
  {
    names.push_back(it->first);
  }
}

Visualizable *VisualizerAnnotatorManager::getAnnotator(const std::string &name) {
  std::map<std::string, Visualizable *>::const_iterator it;
  it = visualizables.find(name);
  if(it != visualizables.end())
  {
    return it->second;
  }
  return NULL;
}

std::string VisualizerAnnotatorManager::getCurrentVisualizableName(){
  return currentVisualizable->name;
}

const std::string &VisualizerAnnotatorManager::getIdentifier() const {
  return identifier_;
}

Visualizable *VisualizerAnnotatorManager::getCurrentVisualizable() const {
  return currentVisualizable;
}

void VisualizerAnnotatorManager::publishOutputImage(cv::Mat &disp) {
  sensor_msgs::Image image_msg;
  cv_bridge::CvImage cv_image;
  cv_image.image = disp;
  cv_image.encoding = "bgr8";
  cv_image.toImageMsg(image_msg);
  outputImagePub.publish(image_msg);
}
