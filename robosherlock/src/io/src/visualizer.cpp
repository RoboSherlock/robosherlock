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
#include <rs/io/visualizer.h>

using namespace rs;

bool *Visualizer::trigger = NULL;

Visualizer::Visualizer(bool headless, bool multiAAEVisualizer) :
//    windowImage(aeName + "/Image Viewer"), windowCloud(aeName +"/Cloud Viewer"),
    running(false), multiAAEVisualizer_(multiAAEVisualizer),
    save(false), headless_(headless), saveFrameImage(0), saveFrameCloud(0), nh_("~")
{
  this->savePath = std::string(getenv("HOME")) +"/.ros/";
  if(this->savePath[this->savePath.size() - 1] != '/')
  {
    this->savePath += '/';
  }

  // TODO reenable the service in the VAMs
//  vis_service_ = nh_.advertiseService(aeName_ + "/vis_command", &Visualizer::visControlCallback, this);
}

Visualizer::~Visualizer()
{
    stop();
}

// TODO think of having headless in the VisualizerAnnotatorManager so some pipelines can be headless
void Visualizer::addVisualizerManager(std::string identifier)
{
  visualizerAnnotatorManagers_[identifier] = std::make_shared<VisualizerAnnotatorManager>(false, identifier);
  visualizerAnnotatorManagers_[identifier]->start();
}

bool Visualizer::start()
{
  outInfo("start");
  saveParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
  saveParams.push_back(9);

  if(!multiAAEVisualizer_){
    outInfo("Using Legacy Visualizer functionality");
    // This is the legacy-visualizer style and shouldn't break the older RoboSherlock code
    // Add the first visualizerAnnotatorManagers_ for the user
    // TODO get the AEName without breaking the API?
    addVisualizerManager("");

    // TODO move into VAM
//    pub = nh_.advertise<sensor_msgs::Image>(aeName_ + "/output_image", 1, true);
//    pubAnnotList = nh_.advertise<robosherlock_msgs::RSActiveAnnotatorList>(aeName_ +"/vis/active_annotators", 1, true);
  } else{
    outInfo("Using MultiAAE Visualizer functionality");

    // TODO this should be moved into the VAMs
//    pub = nh_.advertise<sensor_msgs::Image>(aeName_ + "/output_image", 1, true);
//    pubAnnotList = nh_.advertise<robosherlock_msgs::RSActiveAnnotatorList>(aeName_ +"/vis/active_annotators", 1, true);
  }


  // Creating threads for the visualizations
  // Please note that GUI handling is in general not very multi-thread friendly
  // This is also the reason why we only have ONE thread for
  // handling,for example, OpenCV Windows.
  imageViewerThread = std::thread(&Visualizer::imageViewer, this);
  if(!headless_)
    cloudViewerThread = std::thread(&Visualizer::cloudViewer, this);
  running = true;
  return true;
}

void Visualizer::stop()
{
  outInfo("stopping visualizer!");
  if(running) {
    running = false;
    imageViewerThread.join();
    if(!headless_)
      cloudViewerThread.join();

    if(!multiAAEVisualizer_) {
      pub.shutdown();
      pubAnnotList.shutdown();
    }
  }
  outInfo("visualizer stopped!");
}

void Visualizer::callbackMouse(const int event, const int x, const int y, const int flags, void *object)
{
  // TODO
  // Check which window is active and forward it to the right VisualizerAnnotatorManager
  // NOTE: Mouse callbacks are set up per Window. So it would make sense to bind a method
  // in such a way that we get the name of the window where the mouse action was fired
//  ((Visualizer *)object)->callbackMouseHandler(event, x, y);
  outInfo("TODO MouseHandler impl.");
}

//void Visualizer::callbackMouseHandler(const int event, const int x, const int y)
//{
//  try {
//    bool needupdate_img = annotator->callbackMouse(event, x, y, DrawingAnnotator::IMAGE_VIEWER);
//    updateImage = needupdate_img | updateImage;
//    updateCloud = needupdate_img | updateCloud;
//  }
//  catch(...) {
//    outError("Exception in " << annotator->name << "::callbackMouse!");
//  }
//}

// TODO In MultiAAE mode we should get a parameter which window/aae has been interacted with
// @assert activeVAM is not NULL
void Visualizer::callbackKeyHandler(const char key, const DrawingAnnotator::Source source, std::shared_ptr<VisualizerAnnotatorManager> activeVAM)
{
  // Catch space for triggering
  if(key == ' ') {
    if(trigger) {
      *trigger = true;
    }
    return;
  }
  try {
    bool needupdate_img;


    // TODO use right annotator
    needupdate_img = activeVAM->currentDrawingAnnotator->callbackKey(key, source);
    activeVAM->updateImage = needupdate_img | activeVAM->updateImage;
    activeVAM->updateCloud = needupdate_img | activeVAM->updateCloud;
  }
  catch(...) {
    outError("Exception in " << activeVAM->getCurrentAnnotatorName() << "::callbackKey!");
  }
}

void Visualizer::setActiveAnnotators(std::vector<std::string> annotators)
{
  assert(visualizerAnnotatorManagers_.size()>0);
  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;

  firstVizAnnoMgrAnnotator->setActiveAnnotators(annotators);
}

std::string Visualizer::nextAnnotator()
{
  assert(visualizerAnnotatorManagers_.size()>0);
  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;

  return firstVizAnnoMgrAnnotator->nextAnnotator();
}

std::string Visualizer::prevAnnotator()
{
  assert(visualizerAnnotatorManagers_.size()>0);
  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
  return firstVizAnnoMgrAnnotator->prevAnnotator();
}

std::string Visualizer::selectAnnotator(std::string anno)
{
  assert(visualizerAnnotatorManagers_.size()>0);
  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
  return firstVizAnnoMgrAnnotator->selectAnnotator(anno);
}

void Visualizer::shutdown()
{
  outInfo("ros shutdown!");
  ros::shutdown();
}

void Visualizer::imageViewer()
{
  cv::Mat disp;
  const cv::Point pos(5, 15);
  const cv::Scalar color = CV_RGB(255, 255, 255);
  const double sizeText = 0.5;
  const int lineText = 1;
  const int font = cv::FONT_HERSHEY_SIMPLEX;

  // Initialize Windows for every AAE
  if(!headless_)
  {
    for(auto vam : visualizerAnnotatorManagers_)
    {
      auto& VisualizationAnnotatorMgr = vam.second;
      cv::namedWindow( imageWindowName(*VisualizationAnnotatorMgr), CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
      //cv::moveWindow(windowImage, 0, 0);
//    TODO bring back cv::setMouseCallback(windowImage, &Visualizer::callbackMouse, this);
    }
  }

  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
  for(; ros::ok();) {
    for(auto vam : visualizerAnnotatorManagers_)
    {
      auto& VisualizationAnnotatorMgr = vam.second;
      VisualizationAnnotatorMgr->checkAnnotator();
      if(VisualizationAnnotatorMgr->updateImage) {
        VisualizationAnnotatorMgr->updateImage = false;
        VisualizationAnnotatorMgr->currentDrawingAnnotator->drawImage(disp);
        cv::putText(disp, "Annotator: " + VisualizationAnnotatorMgr->getCurrentAnnotatorName(), pos, font, sizeText, color, lineText, CV_AA);
        if(!headless_)
          cv::imshow(imageWindowName(*VisualizationAnnotatorMgr), disp);

        // TODO bring back ros publishing
//        sensor_msgs::Image image_msg;
//        cv_bridge::CvImage cv_image;
//        cv_image.image = disp;
//        cv_image.encoding = "bgr8";
//        cv_image.toImageMsg(image_msg);
//        pub.publish(image_msg);
      }
    } // end of visualizerAnnotatorManagers_ iteration
    if(!headless_)
      keyboardEventImageViewer(disp);
    usleep(100);
  }
  if(!headless_) {
    for (auto vam : visualizerAnnotatorManagers_) {
      auto &VisualizationAnnotatorMgr = vam.second;
      cv::destroyWindow(imageWindowName(*VisualizationAnnotatorMgr));
    }
  }
  cv::waitKey(100);
}

void Visualizer::cloudViewer()
{
  std::map<std::string, pcl::visualization::PCLVisualizer::Ptr> visualizers;

  for(auto vam : visualizerAnnotatorManagers_) {
    auto &VisualizationAnnotatorMgr = vam.second;
    const std::string annotatorName = "annotatorName-" + vam.first;
    visualizers[vam.first] = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(cloudWindowName(*VisualizationAnnotatorMgr)));

    auto &visualizer = visualizers[vam.first];
    visualizer->initCameraParameters();
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->setBackgroundColor(0, 0, 0);
    // TODO change to support multiple callback handlers
    visualizer->registerKeyboardCallback(&Visualizer::keyboardEventCloudViewer, *this);

    visualizer->addText(VisualizationAnnotatorMgr->getCurrentAnnotatorName(), 2, 20, 12, 1, 1, 1, annotatorName);

    visualizer->spinOnce();
    visualizer->setSize(1280, 960);
  }

  while(ros::ok()) {
    for(auto vam : visualizerAnnotatorManagers_)
    {
      auto &VisualizationAnnotatorMgr = vam.second;
      auto &visualizer = visualizers[vam.first];

      VisualizationAnnotatorMgr->checkAnnotator();

      if(VisualizationAnnotatorMgr->updateCloud) {
        if(VisualizationAnnotatorMgr->changedAnnotator) {
          visualizer->removeAllPointClouds();
          visualizer->removeAllShapes();
          const std::string annotatorName = "annotatorName-" + vam.first;
          visualizer->addText(VisualizationAnnotatorMgr->getCurrentAnnotatorName(), 2, 20, 12, 1, 1, 1, annotatorName);
        }
        if(VisualizationAnnotatorMgr->currentDrawingAnnotator->fillVisualizer(*visualizer, VisualizationAnnotatorMgr->changedAnnotator)) {
          VisualizationAnnotatorMgr->updateCloud = false;
          VisualizationAnnotatorMgr->changedAnnotator = false;
        }
      }
      visualizer->spinOnce(10);
    }

    if(save) {
      // TODO should be put a mutex here if you try to save multiple clouds very quickly?
      save = false;
      if(visualizerAnnotatorManagers_.count(saveVisualizerWithIdentifier)==0)
      {
        outError("Trying to save a cloud but we can't map the entered input from the window to a Visualizer.");
        break;
      }
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>()); // this was in the initialization before, i'm not sure if it's really needed there.
      saveCloud(cloud, visualizers[saveVisualizerWithIdentifier]);
      saveVisualizerWithIdentifier = "";
    }

  } // end of ros::ok while loop
  for(auto vam : visualizerAnnotatorManagers_) {
    auto &visualizer = visualizers[vam.first];
    visualizer->close();
    visualizer->spinOnce(10);
  }
}

void Visualizer::keyboardEventImageViewer(const cv::Mat &disp)
{


  int key;
#if CV_MAJOR_VERSION==3
  key = cv::waitKeyEx(10);
#else
  key = cv::waitKey(10);
#endif
  // Not sure if key == 0 is there for legacy reasons, but according to
  // https://docs.opencv.org/3.4/d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7
  // -1 denotes 'no key was pressed'
  if(key == 0 || key == -1) {
    return;
  }
  bool success=false;
  auto vamInteractedWith = getAnnotatorManagerForActiveWindow(success, DrawingAnnotator::IMAGE_VIEWER);
  if(!success){
    //We couldn't guess the active annotator from the window titles. We'll
    //use the first VAM as a fallback
    outError("Couldn't fetch the active Annotator from the window titles. Will forward to the first AAE.");
    vamInteractedWith = visualizerAnnotatorManagers_.begin()->second;
  }
  int lowerByteOfKey = key & 0xFF;
  switch(lowerByteOfKey) {
  case 110: // next (n)
    vamInteractedWith->nextAnnotator();
    break;
  case 112: // previous (p)
    vamInteractedWith->prevAnnotator();
    break;
  case 99: // insert
    saveImage(disp, vamInteractedWith);
    break;
  }
  if(lowerByteOfKey == 27) { //Escape
    shutdown();
  }
  else {
    callbackKeyHandler(lowerByteOfKey, DrawingAnnotator::IMAGE_VIEWER, vamInteractedWith);
  }
}

void Visualizer::keyboardEventCloudViewer(const pcl::visualization::KeyboardEvent &event, void *)
{
  bool success=false;
  auto vamInteractedWith = getAnnotatorManagerForActiveWindow(success, DrawingAnnotator::CLOUD_VIEWER);
  if(!success){
    //We couldn't guess the active annotator from the window titles. We'll
    //use the first VAM as a fallback
    outError("Couldn't fetch the active Annotator from the window titles. Will forward to the first AAE.");
    vamInteractedWith = visualizerAnnotatorManagers_.begin()->second;
  }

  if(event.keyUp()) {
    if(event.getKeySym() == "Left") {
      vamInteractedWith->nextAnnotator();
    }
    else if(event.getKeySym() == "Right") {
      vamInteractedWith->prevAnnotator();
    }
    else if(event.getKeySym() == "Escape") {
      shutdown();
    }
    else if(event.getKeySym() == "Insert") {
      save = true;
      saveVisualizerWithIdentifier = vamInteractedWith->getAEName();
    }
    else if(event.getKeyCode() > 0) {
      callbackKeyHandler(event.getKeyCode(), DrawingAnnotator::CLOUD_VIEWER, vamInteractedWith);
    }
  }
}

void Visualizer::saveImage(const cv::Mat &disp, std::shared_ptr<VisualizerAnnotatorManager> vam)
{
  std::lock_guard<std::mutex> lock_guard(lock);
  std::ostringstream oss;
  oss << savePath << std::setfill('0') << std::setw(5) << saveFrameImage << "_" << vam->getCurrentAnnotatorName() << ".png";

  outInfo("saving image: " << oss.str());
  cv::imwrite(oss.str(), disp, saveParams);
  ++saveFrameImage;
}

void Visualizer::saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,  pcl::visualization::PCLVisualizer::Ptr &visualizer)
{
  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
  std::lock_guard<std::mutex> lock_guard(lock);
  std::ostringstream oss, oss_cloud;
  oss_cloud << savePath << std::setfill('0') << std::setw(5) << saveFrameCloud << "_" << firstVizAnnoMgrAnnotator->getCurrentAnnotatorName() << ".pcd";
  oss << savePath << std::setfill('0') << std::setw(5) << saveFrameCloud << "_" << firstVizAnnoMgrAnnotator->getCurrentAnnotatorName() << ".png";

  auto x = oss_cloud.str();
  auto y = oss.str();
  outInfo("saving cloud: " << oss_cloud.str());
  //  pcl::io::savePCDFileASCII(oss.str(), *cloud);
  outInfo("saving screenshot: " << oss.str());
  visualizer->saveScreenshot(oss.str());
  ++saveFrameCloud;
}

bool Visualizer::visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
    robosherlock_msgs::RSVisControl::Response &res)
{
  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
  std::string command = req.command;
  bool result = true;
  std::string activeAnnotator = "";

  if(command == "next")
    activeAnnotator = firstVizAnnoMgrAnnotator->nextAnnotator();
  else if(command == "previous")
    activeAnnotator = firstVizAnnoMgrAnnotator->prevAnnotator();
  else if(command != "")
    activeAnnotator = firstVizAnnoMgrAnnotator->selectAnnotator(command);
  if(activeAnnotator == "")
    result = false;
  res.success = result;
  res.active_annotator = activeAnnotator;
  return result;
}

std::string Visualizer::getActiveWindowTitle()
{
  return exec("xprop -id $(xprop -root _NET_ACTIVE_WINDOW | cut -d ' ' -f 5) WM_NAME | awk -F '\"' '{print $2}' ");
}

// TODO maybe introduce another method that will just return the first AAE if there is no other AAE/VAM in visualizerAnnotatorManagers_
std::shared_ptr<VisualizerAnnotatorManager> Visualizer::getAnnotatorManagerForActiveWindow(bool &success, const DrawingAnnotator::Source windowType) {
  success = false;
  std::string active_window_title = getActiveWindowTitle();

  for(auto vam: visualizerAnnotatorManagers_)
  {
    // Check if the active window title starts with the name of the window names the different VAMs should have
    if(windowType == DrawingAnnotator::IMAGE_VIEWER && active_window_title.rfind(imageWindowName(*(vam.second)),0) == 0 )
    {
      success = true;
      return vam.second;
    }

    if(windowType == DrawingAnnotator::CLOUD_VIEWER && active_window_title.rfind(cloudWindowName(*(vam.second)),0) == 0 )
    {
      success = true;
      return vam.second;
    }
  }

  return NULL;
}
