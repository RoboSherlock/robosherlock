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

Visualizer::Visualizer(const std::string &savePath) : windowImage("Image Viewer"), windowCloud("Cloud Viewer"), annotator(NULL), names(), index(0),
  running(false), updateImage(true), updateCloud(true), changedAnnotator(true), save(false), saveFrameImage(0), saveFrameCloud(0), savePath(savePath), nh("~")
{
  this->savePath = savePath;
  if(this->savePath[this->savePath.size() - 1] != '/')
  {
    this->savePath += '/';
  }
}

Visualizer::~Visualizer()
{
}

bool Visualizer::start()
{
  outInfo("start");

  saveParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
  saveParams.push_back(9);

  DrawingAnnotator::getAnnotatorNames(names);
  if(names.empty())
  {
    return false;
  }

  pub = nh.advertise<sensor_msgs::Image>("output_image", 1, true);

  index = 0;
  annotator = DrawingAnnotator::getAnnotator(names[index]);

  imageViewerThread = std::thread(&Visualizer::imageViewer, this);
  cloudViewerThread = std::thread(&Visualizer::cloudViewer, this);
  running = true;
  return true;
}

void Visualizer::stop()
{
  outInfo("stopping visualizer!");
  if(running)
  {
    running = false;
    imageViewerThread.join();
    cloudViewerThread.join();
    pub.shutdown();
  }
  outInfo("visualizer stopped!");
}

void Visualizer::callbackMouse(const int event, const int x, const int y, const int flags, void *object)
{
  ((Visualizer *)object)->callbackMouseHandler(event, x, y);
}

void Visualizer::callbackMouseHandler(const int event, const int x, const int y)
{
  try
  {
    bool needupdate_img = annotator->callbackMouse(event, x, y, DrawingAnnotator::IMAGE_VIEWER);
    updateImage = needupdate_img | updateImage;
    updateCloud = needupdate_img | updateCloud;
  }
  catch(...)
  {
    outError("Exception in " << annotator->name << "::callbackMouse!");
  }
}

void Visualizer::callbackKeyHandler(const char key, const DrawingAnnotator::Source source)
{
  // Catch space for triggering
  if(key == ' ')
  {
    if(trigger)
    {
      *trigger = true;
    }
    return;
  }
  try
  {
    bool needupdate_img = annotator->callbackKey(key, source);
    updateImage = needupdate_img | updateImage;
    updateCloud = needupdate_img | updateCloud;
  }
  catch(...)
  {
    outError("Exception in " << annotator->name << "::callbackKey!");
  }
}

void Visualizer::nextAnnotator()
{
  lock.lock();
  DrawingAnnotator::getAnnotatorNames(names);
  index = (index + 1) % names.size();
  annotator = DrawingAnnotator::getAnnotator(names[index]);
  annotator->update = false;
  updateImage = true;
  updateCloud = true;
  changedAnnotator = true;
  lock.unlock();
  outDebug("switching to annotator: " << names[index]);
}

void Visualizer::prevAnnotator()
{
  lock.lock();
  DrawingAnnotator::getAnnotatorNames(names);
  index = (names.size() + index - 1) % names.size();
  annotator = DrawingAnnotator::getAnnotator(names[index]);
  annotator->update = false;
  updateImage = true;
  updateCloud = true;
  changedAnnotator = true;
  lock.unlock();
  outDebug("switching to annotator: " << names[index]);
}

void Visualizer::checkAnnotator()
{
  lock.lock();
  if(annotator->update)
  {
    annotator->update = false;
    updateImage = true;
    updateCloud = true;
  }
  lock.unlock();
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

  cv::namedWindow(windowImage);
  //cv::moveWindow(windowImage, 0, 0);
  cv::setMouseCallback(windowImage, &Visualizer::callbackMouse, this);

  for(; ros::ok();)
  {
    checkAnnotator();

    if(updateImage)
    {
      updateImage = false;
      annotator->drawImage(disp);
      cv::putText(disp, "Annotator: " + names[index], pos, font, sizeText, color, lineText, CV_AA);
      cv::imshow(windowImage, disp);

      sensor_msgs::Image image_msg;
      cv_bridge::CvImage cv_image;
      cv_image.image = disp;
      cv_image.encoding = "bgr8";
      cv_image.toImageMsg(image_msg);
      pub.publish(image_msg);
    }

    keyboardEventImageViewer(disp);
  }
  cv::destroyWindow(windowImage);
  cv::waitKey(100);
}

void Visualizer::cloudViewer()
{
  const std::string annotatorName = "annotatorName";
  pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

  //visualizer->addCoordinateSystem(0.1, 0);
  visualizer->initCameraParameters();
  visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
  visualizer->setBackgroundColor(0, 0, 0);
  visualizer->registerKeyboardCallback(&Visualizer::keyboardEventCloudViewer, *this);

  visualizer->addText(annotator->name, 2, 20, 12, 1, 1, 1, annotatorName);

  visualizer->spinOnce();
  visualizer->setSize(1280, 960);

  while(ros::ok())
  {
    checkAnnotator();

    if(updateCloud)
    {
      if(changedAnnotator)
      {
        visualizer->removeAllPointClouds();
        visualizer->removeAllShapes();
        visualizer->addText(annotator->name, 2, 20, 12, 1, 1, 1, annotatorName);
      }
      if(annotator->fillVisualizer(*visualizer, changedAnnotator))
      {
        updateCloud = false;
        changedAnnotator = false;
      }
    }
    if(save)
    {
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
  const int key = cv::waitKey(10);

  if(key == 0)
  {
    return;
  }

  if(key & 0xFF00) // Special keys
  {
    switch(key & 0xFF)
    {
    case 81: // right arrow
      nextAnnotator();
      break;
    case 83: // left arrow
      prevAnnotator();
      break;
    case 99: // insert
      saveImage(disp);
      break;
    }
  }
  else
  {
    if((key & 0xFF) == 27) //Escape
    {
      shutdown();
    }
    else
    {
      callbackKeyHandler(key & 0xFF, DrawingAnnotator::IMAGE_VIEWER);
    }
  }
}

void Visualizer::keyboardEventCloudViewer(const pcl::visualization::KeyboardEvent &event, void *)
{
  if(event.keyUp())
  {
    if(event.getKeySym() == "Left")
    {
      nextAnnotator();
    }
    else if(event.getKeySym() == "Right")
    {
      prevAnnotator();
    }
    else if(event.getKeySym() == "Escape")
    {
      shutdown();
    }
    else if(event.getKeySym() == "Insert")
    {
      save = true;
    }
    else if(event.getKeyCode() > 0)
    {
      callbackKeyHandler(event.getKeyCode(), DrawingAnnotator::CLOUD_VIEWER);
    }
  }
}

void Visualizer::saveImage(const cv::Mat &disp)
{
  lock.lock();
  std::ostringstream oss;
  oss << savePath << std::setfill('0') << std::setw(5) << saveFrameImage << "_" << names[index] << ".png";

  outInfo("saving image: " << oss.str());
  cv::imwrite(oss.str(), disp, saveParams);
  ++saveFrameImage;
  lock.unlock();
}

void Visualizer::saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,  pcl::visualization::PCLVisualizer::Ptr &visualizer)
{
  lock.lock();
  std::ostringstream oss, oss_cloud;
  oss_cloud << savePath << std::setfill('0') << std::setw(5) << saveFrameCloud << "_" << names[index] << ".pcd";
  oss << savePath << std::setfill('0') << std::setw(5) << saveFrameCloud << "_" << names[index] << ".png";

  outInfo("saving cloud: " << oss_cloud.str());
  //  pcl::io::savePCDFileASCII(oss.str(), *cloud);
  outInfo("saving screenshot: " << oss.str());
  visualizer->saveScreenshot(oss.str());
  ++saveFrameCloud;
  lock.unlock();
}

