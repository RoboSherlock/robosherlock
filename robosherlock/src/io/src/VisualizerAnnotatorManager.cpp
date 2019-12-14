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
  currentDrawingAnnotator(NULL), names(), index(0), running(false),
  save(false), headless_(headless), saveFrameImage(0), saveFrameCloud(0), nh_("~"),
  updateImage(true), updateCloud(true), changedAnnotator(true)
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
  consumeRecentDrawingAnnotators(); // Claim the responsibility for all DrawingAnnotators in this VisualizerAnnotatorManager

  getAnnotatorNames(names);
  if(names.empty()) {
    outInfo("No annotators do visualize. Aborting Visualizer start.");
    return false;
  }
  //Initially, all annotators are active
  activeAnnotators = names;
//
  outputImagePub = nh_.advertise<sensor_msgs::Image>(identifier_ + "/output_image", 1, true);
  pubAnnotList = nh_.advertise<robosherlock_msgs::RSActiveAnnotatorList>(identifier_ +"/vis/active_annotators", 1, true);
  index = 0;

  currentDrawingAnnotator = getAnnotator(names[index]);

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
  std::vector<std::string> activeDrawingAnnotators(names.size());
  std::vector<std::string>::iterator it;
  std::sort(annotators.begin(), annotators.end());
  std::sort(names.begin(), names.end());
  it = std::set_intersection(annotators.begin(), annotators.end(), names.begin(), names.end(), activeDrawingAnnotators.begin());
  activeDrawingAnnotators.resize(it - activeDrawingAnnotators.begin());
  activeAnnotators = activeDrawingAnnotators;

  robosherlock_msgs::RSActiveAnnotatorList listMsg;
  listMsg.annotators = activeAnnotators;
  pubAnnotList.publish(listMsg);
  }

}


std::string VisualizerAnnotatorManager::nextAnnotator()
{
  {
    std::lock_guard<std::mutex> lock_guard(lock);
    if(activeAnnotators.empty()) return "";
    index = (index + 1) % activeAnnotators.size();
//    annotator = DrawingAnnotator::getAnnotator(activeAnnotators[index]);
    currentDrawingAnnotator = getAnnotator(activeAnnotators[index]);
    currentDrawingAnnotator->update = false;
    updateImage = true;
    updateCloud = true;
    changedAnnotator = true;
  }
  outDebug("switching to annotator: " << activeAnnotators[index]);
  return activeAnnotators[index];
}

std::string VisualizerAnnotatorManager::prevAnnotator()
{
  {
    std::lock_guard<std::mutex> lock_guard(lock);
    if(activeAnnotators.empty()) return "";
    index = (activeAnnotators.size() + index - 1) % activeAnnotators.size();
//    annotator = DrawingAnnotator::getAnnotator(activeAnnotators[index]);
    currentDrawingAnnotator = getAnnotator(activeAnnotators[index]);
    currentDrawingAnnotator->update = false;
    updateImage = true;
    updateCloud = true;
    changedAnnotator = true;
  }
  outDebug("switching to annotator: " << activeAnnotators[index]);
  return activeAnnotators[index];
}

std::string VisualizerAnnotatorManager::selectAnnotator(std::string anno)
{
  std::lock_guard<std::mutex> lock_guard(lock);
  ptrdiff_t pos = distance(activeAnnotators.begin(), find(activeAnnotators.begin(), activeAnnotators.end(), anno));
  index = pos;
  if(index >= activeAnnotators.size())
    return "";
//  annotator = DrawingAnnotator::getAnnotator(activeAnnotators[index]);
  currentDrawingAnnotator = getAnnotator(activeAnnotators[index]);
  currentDrawingAnnotator->update = false;
  updateImage = true;
  updateCloud = true;
  changedAnnotator = true;
  outDebug("switching to annotator: " << activeAnnotators[index]);
  return activeAnnotators[index];
}


void VisualizerAnnotatorManager::checkAnnotator()
{
  std::lock_guard<std::mutex> lock_guard(lock);
  if(currentDrawingAnnotator->update) {
    currentDrawingAnnotator->update = false;
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

int VisualizerAnnotatorManager::consumeRecentDrawingAnnotators() {
  int copiedElements = DrawingAnnotator::copyAnnotatorList(drawingAnnotators);
  DrawingAnnotator::clearAnnotatorList();
  return copiedElements;
}

void VisualizerAnnotatorManager::getAnnotatorNames(std::vector<std::string> &names) {
  std::map<std::string, DrawingAnnotator *>::const_iterator it;
  names.clear();
  names.reserve(drawingAnnotators.size());

  for(it = drawingAnnotators.begin(); it != drawingAnnotators.end(); ++it)
  {
    names.push_back(it->first);
  }
}

DrawingAnnotator *VisualizerAnnotatorManager::getAnnotator(const std::string &name) {
  std::map<std::string, DrawingAnnotator *>::const_iterator it;
  it = drawingAnnotators.find(name);
  if(it != drawingAnnotators.end())
  {
    return it->second;
  }
  return NULL;
}

std::string VisualizerAnnotatorManager::getCurrentAnnotatorName(){
  return currentDrawingAnnotator->name;
}

const std::string &VisualizerAnnotatorManager::getIdentifier() const {
  return identifier_;
}

DrawingAnnotator *VisualizerAnnotatorManager::getCurrentDrawingAnnotator() const {
  return currentDrawingAnnotator;
}

void VisualizerAnnotatorManager::publishOutputImage(cv::Mat &disp) {
  sensor_msgs::Image image_msg;
  cv_bridge::CvImage cv_image;
  cv_image.image = disp;
  cv_image.encoding = "bgr8";
  cv_image.toImageMsg(image_msg);
  outputImagePub.publish(image_msg);
}
