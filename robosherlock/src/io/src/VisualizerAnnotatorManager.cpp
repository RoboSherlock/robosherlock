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

VisualizerAnnotatorManager::VisualizerAnnotatorManager(bool headless, std::string aeName) : aeName_(aeName),
    windowImage(aeName + "/Image Viewer"), windowCloud(aeName +"/Cloud Viewer"),
    annotator(NULL), names(), index(0), running(false), updateImage(true), updateCloud(true), changedAnnotator(true),
    save(false), headless_(headless), saveFrameImage(0), saveFrameCloud(0), nh_("~")
{
  this->savePath = std::string(getenv("USER")) +"./ros/";
  if(this->savePath[this->savePath.size() - 1] != '/')
  {
    this->savePath += '/';
  }
  vis_service_ = nh_.advertiseService(aeName_ + "/vis_command", &VisualizerAnnotatorManager::visControlCallback, this);
}

VisualizerAnnotatorManager::~VisualizerAnnotatorManager()
{
    stop();
}

bool VisualizerAnnotatorManager::start()
{
  outInfo("start");
//  consumeRecentDrawingAnnotators(); // Claim the responsibility for all DrawingAnnotators in this VisualizerAnnotatorManager
//
//
//  saveParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
//  saveParams.push_back(9);
//
////  DrawingAnnotator::getAnnotatorNames(names);
//  getAnnotatorNames(names);
//  if(names.empty()) {
//    return false;
//  }
//  //Initially, all annotators are active
//  activeAnnotators = names;
//
//  pub = nh_.advertise<sensor_msgs::Image>(aeName_ + "/output_image", 1, true);
  pubAnnotList = nh_.advertise<robosherlock_msgs::RSActiveAnnotatorList>(aeName_ +"/vis/active_annotators", 1, true);
//
//  index = 0;
////  annotator = DrawingAnnotator::getAnnotator(names[index]);
//  annotator = getAnnotator(names[index]);
//
//
  running = true;
//  return true;
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

void VisualizerAnnotatorManager::callbackMouse(const int event, const int x, const int y, const int flags, void *object)
{
  ((VisualizerAnnotatorManager *)object)->callbackMouseHandler(event, x, y);
}

void VisualizerAnnotatorManager::callbackMouseHandler(const int event, const int x, const int y)
{
  try {
    bool needupdate_img = annotator->callbackMouse(event, x, y, DrawingAnnotator::IMAGE_VIEWER);
    updateImage = needupdate_img | updateImage;
    updateCloud = needupdate_img | updateCloud;
  }
  catch(...) {
    outError("Exception in " << annotator->name << "::callbackMouse!");
  }
}

void VisualizerAnnotatorManager::callbackKeyHandler(const char key, const DrawingAnnotator::Source source)
{
  // Catch space for triggering
  if(key == ' ') {
    if(trigger) {
      *trigger = true;
    }
    return;
  }
  try {
    bool needupdate_img = annotator->callbackKey(key, source);
    updateImage = needupdate_img | updateImage;
    updateCloud = needupdate_img | updateCloud;
  }
  catch(...) {
    outError("Exception in " << annotator->name << "::callbackKey!");
  }
}

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
    annotator = getAnnotator(activeAnnotators[index]);
    annotator->update = false;
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
    annotator = getAnnotator(activeAnnotators[index]);
    annotator->update = false;
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
  annotator = getAnnotator(activeAnnotators[index]);
  annotator->update = false;
  updateImage = true;
  updateCloud = true;
  changedAnnotator = true;
  outDebug("switching to annotator: " << activeAnnotators[index]);
  return activeAnnotators[index];
}


void VisualizerAnnotatorManager::checkAnnotator()
{
  std::lock_guard<std::mutex> lock_guard(lock);
  if(annotator->update) {
    annotator->update = false;
    updateImage = true;
    updateCloud = true;
  }
}

void VisualizerAnnotatorManager::keyboardEventImageViewer(const cv::Mat &disp)
{
    // TODO maybe forward the stuff in this class to the actual annotator
//
//  int key;
//#if CV_MAJOR_VERSION==3
//  key = cv::waitKeyEx(10);
//#else
//  key = cv::waitKey(10);
//#endif
//  // Not sure if key == 0 is there for legacy reasons, but according to
//  // https://docs.opencv.org/3.4/d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7
//  // -1 denotes 'no key was pressed'
//  if(key == 0 || key == -1) {
//    return;
//  }
//  switch(key) {
//  case 110: // next (n)
//    nextAnnotator();
//    break;
//  case 112: // previous (p)
//    prevAnnotator();
//    break;
//  case 99: // insert
//    saveImage(disp);
//    break;
//  }
//
//  if((key & 0xFF) == 27) { //Escape
//    shutdown();
//  }
//  else {
//    callbackKeyHandler(key & 0xFF, DrawingAnnotator::IMAGE_VIEWER);
//  }

}

void VisualizerAnnotatorManager::keyboardEventCloudViewer(const pcl::visualization::KeyboardEvent &event, void *)
{
    // TODO maybe forward the stuff in this class to the actual annotator
//  if(event.keyUp()) {
//    if(event.getKeySym() == "Left") {
//      nextAnnotator();
//    }
//    else if(event.getKeySym() == "Right") {
//      prevAnnotator();
//    }
//    else if(event.getKeySym() == "Escape") {
//      shutdown();
//    }
//    else if(event.getKeySym() == "Insert") {
//      save = true;
//    }
//    else if(event.getKeyCode() > 0) {
//      callbackKeyHandler(event.getKeyCode(), DrawingAnnotator::CLOUD_VIEWER);
//    }
//  }
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

