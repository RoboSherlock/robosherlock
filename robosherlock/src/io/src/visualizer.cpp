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

Visualizer::Visualizer(bool headless, std::string aeName, bool multiAAEVisualizer) : aeName_(aeName),
//    windowImage(aeName + "/Image Viewer"), windowCloud(aeName +"/Cloud Viewer"),
    running(false), multiAAEVisualizer_(multiAAEVisualizer),
    save(false), headless_(headless), saveFrameImage(0), saveFrameCloud(0), nh_("~")
{
  this->savePath = std::string(getenv("USER")) +"./ros/";
  if(this->savePath[this->savePath.size() - 1] != '/')
  {
    this->savePath += '/';
  }
  vis_service_ = nh_.advertiseService(aeName_ + "/vis_command", &Visualizer::visControlCallback, this);
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
    addVisualizerManager(aeName_);

//    auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
//    firstVizAnnoMgrAnnotator->start();

    pub = nh_.advertise<sensor_msgs::Image>(aeName_ + "/output_image", 1, true);
    pubAnnotList = nh_.advertise<robosherlock_msgs::RSActiveAnnotatorList>(aeName_ +"/vis/active_annotators", 1, true);
  } else{
    outInfo("Using MultiAAE Visualizer functionality");
//    for(auto v : visualizerAnnotatorManagers_)


    // TODO this should be moved into the VAMs
    pub = nh_.advertise<sensor_msgs::Image>(aeName_ + "/output_image", 1, true);
    pubAnnotList = nh_.advertise<robosherlock_msgs::RSActiveAnnotatorList>(aeName_ +"/vis/active_annotators", 1, true);
  }


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
void Visualizer::callbackKeyHandler(const char key, const DrawingAnnotator::Source source)
{
  assert(visualizerAnnotatorManagers_.size()>0);
  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
  // Catch space for triggering
  if(key == ' ') {
    if(trigger) {
      *trigger = true;
    }
    return;
  }
  try {
    bool needupdate_img;



    needupdate_img = firstVizAnnoMgrAnnotator->currentDrawingAnnotator->callbackKey(key, source);
    firstVizAnnoMgrAnnotator->updateImage = needupdate_img | firstVizAnnoMgrAnnotator->updateImage;
    firstVizAnnoMgrAnnotator->updateCloud = needupdate_img | firstVizAnnoMgrAnnotator->updateCloud;

//    if(multiAAEVisualizer_)
//    {
//
//      // Just use the first window / annotator manager for now
//      assert(visualizerAnnotatorManagers_.size()>0);
//      auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
//      needupdate_img = firstVizAnnoMgrAnnotator->currentDrawingAnnotator->callbackKey(key, source);
//      visualizerAnnotatorManager_.updateImage = needupdate_img | firstVizAnnoMgrAnnotator->updateImage;
//      visualizerAnnotatorManager_.updateCloud = needupdate_img | firstVizAnnoMgrAnnotator->updateCloud;
//    }else {
//      needupdate_img = visualizerAnnotatorManager_.currentDrawingAnnotator->callbackKey(key, source);
//      visualizerAnnotatorManager_.updateImage = needupdate_img | visualizerAnnotatorManager_.updateImage;
//      visualizerAnnotatorManager_.updateCloud = needupdate_img | visualizerAnnotatorManager_.updateCloud;
//    }

  }
  catch(...) {
    outError("Exception in " << firstVizAnnoMgrAnnotator->getCurrentAnnotatorName() << "::callbackKey!");
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
//    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer(cloudWindowName(*VisualizationAnnotatorMgr)));

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
    // TODO check WHICH visualizer has been saved
//    if(save) {
//      save = false;
//      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>()); // this was in the initialization before, i'm not sure if it's really needed there.
//      saveCloud(cloud, visualizer);
//    }

  } // end of ros::ok while loop
  for(auto vam : visualizerAnnotatorManagers_) {
    auto &visualizer = visualizers[vam.first];
    visualizer->close();
    visualizer->spinOnce(10);
  }
}

void Visualizer::keyboardEventImageViewer(const cv::Mat &disp)
{

  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
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
  switch(key) {
  case 110: // next (n)
    firstVizAnnoMgrAnnotator->nextAnnotator();
    break;
  case 112: // previous (p)
    firstVizAnnoMgrAnnotator->prevAnnotator();
    break;
  case 99: // insert
    saveImage(disp);
    break;
  }

  if((key & 0xFF) == 27) { //Escape
    shutdown();
  }
  else {
    callbackKeyHandler(key & 0xFF, DrawingAnnotator::IMAGE_VIEWER);
  }

}

void Visualizer::keyboardEventCloudViewer(const pcl::visualization::KeyboardEvent &event, void *)
{
  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
  if(event.keyUp()) {
    if(event.getKeySym() == "Left") {
      firstVizAnnoMgrAnnotator->nextAnnotator();
    }
    else if(event.getKeySym() == "Right") {
      firstVizAnnoMgrAnnotator->prevAnnotator();
    }
    else if(event.getKeySym() == "Escape") {
      shutdown();
    }
    else if(event.getKeySym() == "Insert") {
      save = true;
    }
    else if(event.getKeyCode() > 0) {
      callbackKeyHandler(event.getKeyCode(), DrawingAnnotator::CLOUD_VIEWER);
    }
  }
}

// TODO this method has to know which AAE/VAM to pick
void Visualizer::saveImage(const cv::Mat &disp)
{
  auto firstVizAnnoMgrAnnotator = visualizerAnnotatorManagers_.begin()->second;
  std::lock_guard<std::mutex> lock_guard(lock);
  std::ostringstream oss;
  oss << savePath << std::setfill('0') << std::setw(5) << saveFrameImage << "_" << firstVizAnnoMgrAnnotator->getCurrentAnnotatorName() << ".png";

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