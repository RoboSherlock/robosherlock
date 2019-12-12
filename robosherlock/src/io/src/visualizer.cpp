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

Visualizer::Visualizer(bool headless, std::string aeName) : aeName_(aeName),
    windowImage(aeName + "/Image Viewer"), windowCloud(aeName +"/Cloud Viewer"),
    running(false),
    save(false), headless_(headless), saveFrameImage(0), saveFrameCloud(0), nh_("~"),
    visualizerAnnotatorManager_(headless, aeName)
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

bool Visualizer::start()
{
  outInfo("start");
  visualizerAnnotatorManager_.start();

  saveParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
  saveParams.push_back(9);

  pub = nh_.advertise<sensor_msgs::Image>(aeName_ + "/output_image", 1, true);
  pubAnnotList = nh_.advertise<robosherlock_msgs::RSActiveAnnotatorList>(aeName_ +"/vis/active_annotators", 1, true);

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
    pub.shutdown();
    pubAnnotList.shutdown();
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

void Visualizer::callbackKeyHandler(const char key, const DrawingAnnotator::Source source)
{
  // Catch space for triggering
  if(key == ' ') {
    if(trigger) {
      *trigger = true;
    }
    return;
  }
  try {
    bool needupdate_img = visualizerAnnotatorManager_.currentDrawingAnnotator->callbackKey(key, source);
    visualizerAnnotatorManager_.updateImage = needupdate_img | visualizerAnnotatorManager_.updateImage;
    visualizerAnnotatorManager_.updateCloud = needupdate_img | visualizerAnnotatorManager_.updateCloud;
  }
  catch(...) {
    outError("Exception in " << visualizerAnnotatorManager_.getCurrentAnnotatorName() << "::callbackKey!");
  }
}

void Visualizer::setActiveAnnotators(std::vector<std::string> annotators)
{
  visualizerAnnotatorManager_.setActiveAnnotators(annotators);
}

std::string Visualizer::nextAnnotator()
{
  return visualizerAnnotatorManager_.nextAnnotator();
}

std::string Visualizer::prevAnnotator()
{
  return visualizerAnnotatorManager_.prevAnnotator();
}

std::string Visualizer::selectAnnotator(std::string anno)
{
  return visualizerAnnotatorManager_.selectAnnotator(anno);
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

  if(!headless_) {
    cv::namedWindow(windowImage, CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
    //cv::moveWindow(windowImage, 0, 0);
    cv::setMouseCallback(windowImage, &Visualizer::callbackMouse, this);
  }
  for(; ros::ok();) {
    visualizerAnnotatorManager_.checkAnnotator();

    if(visualizerAnnotatorManager_.updateImage) {
      visualizerAnnotatorManager_.updateImage = false;
      visualizerAnnotatorManager_.currentDrawingAnnotator->drawImage(disp);
      cv::putText(disp, "Annotator: " + visualizerAnnotatorManager_.getCurrentAnnotatorName(), pos, font, sizeText, color, lineText, CV_AA);
      if(!headless_)
        cv::imshow(windowImage, disp);

      sensor_msgs::Image image_msg;
      cv_bridge::CvImage cv_image;
      cv_image.image = disp;
      cv_image.encoding = "bgr8";
      cv_image.toImageMsg(image_msg);
      pub.publish(image_msg);
    }
    if(!headless_)
      keyboardEventImageViewer(disp);
    usleep(100);
  }
  if(!headless_)
    cv::destroyWindow(windowImage);
  cv::waitKey(100);
}

void Visualizer::cloudViewer()
{
  const std::string annotatorName = "annotatorName";
  pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer(windowCloud));
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

  //visualizer->addCoordinateSystem(0.1, 0);
  visualizer->initCameraParameters();
  visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
  visualizer->setBackgroundColor(0, 0, 0);
  visualizer->registerKeyboardCallback(&Visualizer::keyboardEventCloudViewer, *this);

  visualizer->addText(visualizerAnnotatorManager_.getCurrentAnnotatorName(), 2, 20, 12, 1, 1, 1, annotatorName);

  visualizer->spinOnce();
  visualizer->setSize(1280, 960);

  while(ros::ok()) {
    visualizerAnnotatorManager_.checkAnnotator();

    if(visualizerAnnotatorManager_.updateCloud) {
      if(visualizerAnnotatorManager_.changedAnnotator) {
        visualizer->removeAllPointClouds();
        visualizer->removeAllShapes();
        visualizer->addText(visualizerAnnotatorManager_.getCurrentAnnotatorName(), 2, 20, 12, 1, 1, 1, annotatorName);
      }
      if(visualizerAnnotatorManager_.currentDrawingAnnotator->fillVisualizer(*visualizer, visualizerAnnotatorManager_.changedAnnotator)) {
        visualizerAnnotatorManager_.updateCloud = false;
        visualizerAnnotatorManager_.changedAnnotator = false;
      }
    }
    if(save) {
      save = false;
      saveCloud(cloud, visualizer);
    }

    visualizer->spinOnce(10);
  }
  visualizer->close();
  visualizer->spinOnce(10);
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
  switch(key) {
  case 110: // next (n)
    visualizerAnnotatorManager_.nextAnnotator();
    break;
  case 112: // previous (p)
    visualizerAnnotatorManager_.prevAnnotator();
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
  if(event.keyUp()) {
    if(event.getKeySym() == "Left") {
      visualizerAnnotatorManager_.nextAnnotator();
    }
    else if(event.getKeySym() == "Right") {
      visualizerAnnotatorManager_.prevAnnotator();
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

void Visualizer::saveImage(const cv::Mat &disp)
{
  std::lock_guard<std::mutex> lock_guard(lock);
  std::ostringstream oss;
  oss << savePath << std::setfill('0') << std::setw(5) << saveFrameImage << "_" << visualizerAnnotatorManager_.getCurrentAnnotatorName() << ".png";

  outInfo("saving image: " << oss.str());
  cv::imwrite(oss.str(), disp, saveParams);
  ++saveFrameImage;
}

void Visualizer::saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,  pcl::visualization::PCLVisualizer::Ptr &visualizer)
{
  std::lock_guard<std::mutex> lock_guard(lock);
  std::ostringstream oss, oss_cloud;
  oss_cloud << savePath << std::setfill('0') << std::setw(5) << saveFrameCloud << "_" << visualizerAnnotatorManager_.getCurrentAnnotatorName() << ".pcd";
  oss << savePath << std::setfill('0') << std::setw(5) << saveFrameCloud << "_" << visualizerAnnotatorManager_.getCurrentAnnotatorName() << ".png";

  outInfo("saving cloud: " << oss_cloud.str());
  //  pcl::io::savePCDFileASCII(oss.str(), *cloud);
  outInfo("saving screenshot: " << oss.str());
  visualizer->saveScreenshot(oss.str());
  ++saveFrameCloud;
}

bool Visualizer::visControlCallback(robosherlock_msgs::RSVisControl::Request &req,
    robosherlock_msgs::RSVisControl::Response &res)
{
  std::string command = req.command;
  bool result = true;
  std::string activeAnnotator = "";

  if(command == "next")
    activeAnnotator = visualizerAnnotatorManager_.nextAnnotator();
  else if(command == "previous")
    activeAnnotator = visualizerAnnotatorManager_.prevAnnotator();
  else if(command != "")
    activeAnnotator = visualizerAnnotatorManager_.selectAnnotator(command);
  if(activeAnnotator == "")
    result = false;
  res.success = result;
  res.active_annotator = activeAnnotator;
  return result;
}