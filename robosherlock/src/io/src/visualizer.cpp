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

bool* Visualizer::trigger = NULL;

Visualizer::Visualizer(bool headless, bool multiAAEVisualizer)
  : running(false)
  , multiAAEVisualizer_(multiAAEVisualizer)
  , save(false)
  , saveImageToDisk(false)
  , headless_(headless)
  , saveFrameImage(0)
  , saveFrameCloud(0)
  , nh_("~")
{
  this->savePath = std::string(getenv("HOME")) + "/.ros/";
  if (this->savePath[this->savePath.size() - 1] != '/')
  {
    this->savePath += '/';
  }
}

Visualizer::~Visualizer()
{
  stop();
}

void Visualizer::addVisualizableGroupManager(std::string identifier)
{
  if (visualizableGroupManagers_.count(identifier) > 0)
  {
    outWarn("The given identifier '" << identifier
                                     << "' in addVisualizableGroupManager is already existing! Visualizer Identifiers "
                                        "must be unique. Maybe you have loaded multiple analysis engines or cas "
                                        "consumers with the same name.");
  }
  visualizableGroupManagers_[identifier] = std::make_shared<VisualizableGroupManager>(identifier);
  visualizableGroupManagers_[identifier]->start();
}

bool Visualizer::start()
{
  outInfo("start");
  saveParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
  saveParams.push_back(9);

  if (!multiAAEVisualizer_)
  {
    outInfo("Using Legacy Visualizer functionality");
    // This is the legacy-visualizer style and shouldn't break the older RoboSherlock code
    // Add the first visualizableGroupManagers_ for the user

    // There is currently no way get the the AAE Name without breaking the API.
    // So we just pass an empty AAE Name here.
    addVisualizableGroupManager("");
  }
  else
  {
    outInfo("Using MultiAAE Visualizer functionality");
  }

  // Creating threads for the visualizations
  // Please note that GUI handling is in general not very multi-thread friendly
  // This is also the reason why we only have ONE thread for
  // handling,for example, OpenCV Windows.
  imageViewerThread = std::thread(&Visualizer::imageViewer, this);
  if (!headless_)
    cloudViewerThread = std::thread(&Visualizer::cloudViewer, this);
  running = true;
  return true;
}

void Visualizer::stop()
{
  outInfo("stopping visualizer!");
  if (running)
  {
    running = false;
    imageViewerThread.join();
    if (!headless_)
      cloudViewerThread.join();

    if (!multiAAEVisualizer_)
    {
      pub.shutdown();
      pubAnnotList.shutdown();
    }
  }
  outInfo("visualizer stopped!");
}

void Visualizer::callbackMouse(const int event, const int x, const int y, const int flags, void* object)
{
  VisualizableGroupManager* vgm = ((VisualizableGroupManager*)object);
  vgm->callbackMouseHandler(event, x, y);
}

void Visualizer::callbackKeyHandler(const char key, const Visualizable::VisualizableDataType source,
                                    std::shared_ptr<VisualizableGroupManager> activeVGM)
{
  // Catch space for triggering
  if (key == ' ')
  {
    if (trigger)
    {
      *trigger = true;
    }
    return;
  }
  try
  {
    bool needupdate_img;

    needupdate_img = activeVGM->getCurrentVisualizable()->callbackKey(key, source);
    activeVGM->updateImage = needupdate_img | activeVGM->updateImage;
    activeVGM->updateCloud = needupdate_img | activeVGM->updateCloud;
  }
  catch (...)
  {
    outError("Exception in " << activeVGM->getCurrentVisualizableName() << "::callbackKey!");
  }
}

void Visualizer::setActiveAnnotators(std::vector<std::string> annotators)
{
  // TODO change service and add a parameter which VGM/pipeline/AAE is meant
  assert(visualizableGroupManagers_.size() > 0);
  auto firstVGM = visualizableGroupManagers_.begin()->second;

  firstVGM->setActiveVisualizable(annotators);
}

std::string Visualizer::nextAnnotator()
{
  assert(visualizableGroupManagers_.size() > 0);
  auto firstVGM = visualizableGroupManagers_.begin()->second;

  return firstVGM->nextVisualizable();
}

std::string Visualizer::prevAnnotator()
{
  assert(visualizableGroupManagers_.size() > 0);
  auto firstVGM = visualizableGroupManagers_.begin()->second;
  return firstVGM->prevVisualizable();
}

std::string Visualizer::selectAnnotator(std::string anno)
{
  assert(visualizableGroupManagers_.size() > 0);
  auto firstVGM = visualizableGroupManagers_.begin()->second;
  return firstVGM->selectVisualizable(anno);
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
  if (!headless_)
  {
    for (auto vgm : visualizableGroupManagers_)
    {
      auto& VisualizationAnnotatorMgr = vgm.second;
      cv::namedWindow(imageWindowName(*VisualizationAnnotatorMgr), CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);

      // TODO It's not so nice to point to the raw data in the shared_ptr.
      cv::setMouseCallback(imageWindowName(*VisualizationAnnotatorMgr), &Visualizer::callbackMouse, &(*(vgm.second)));
    }
  }

  auto firstVizAnnoMgrAnnotator = visualizableGroupManagers_.begin()->second;
  for (; ros::ok();)
  {
    for (auto vgm : visualizableGroupManagers_)
    {
      auto& visualizationAnnotatorMgr = vgm.second;
      visualizationAnnotatorMgr->checkVisualizable();
      if (visualizationAnnotatorMgr->updateImage)
      {
        visualizationAnnotatorMgr->updateImage = false;
        visualizationAnnotatorMgr->getCurrentVisualizable()->drawImage(disp);
        cv::putText(disp, "Annotator: " + visualizationAnnotatorMgr->getCurrentVisualizableName(), pos, font, sizeText,
                    color, lineText, CV_AA);
        if (!headless_)
          cv::imshow(imageWindowName(*visualizationAnnotatorMgr), disp);

        visualizationAnnotatorMgr->publishOutputImage(disp);

        // When an image is to be saved from a previous iteration, check if we can fetch the current image now and write
        // it. If it's a problem that this is always one iteration behind, one could think of calling
        // the keyboard event image viewer for each iteration in this scope
        if (saveImageToDisk && imageVgmToBeSaved->getIdentifier() == visualizationAnnotatorMgr->getIdentifier())
        {
          saveImageToDisk = false;
          saveImage(disp, visualizationAnnotatorMgr);
        }
      }

    }  // end of visualizableGroupManagers_ iteration
    if (!headless_)
      keyboardEventImageViewer(disp);
    usleep(100);
  }
  if (!headless_)
  {
    for (auto vgm : visualizableGroupManagers_)
    {
      auto& VisualizationAnnotatorMgr = vgm.second;
      cv::destroyWindow(imageWindowName(*VisualizationAnnotatorMgr));
    }
  }
  cv::waitKey(100);
}

void Visualizer::cloudViewer()
{
  std::map<std::string, pcl::visualization::PCLVisualizer::Ptr> visualizers;

  for (auto vgm : visualizableGroupManagers_)
  {
    auto& VisualizationAnnotatorMgr = vgm.second;
    const std::string annotatorName = "annotatorName-" + vgm.first;
    visualizers[vgm.first] = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer(cloudWindowName(*VisualizationAnnotatorMgr)));

    auto& visualizer = visualizers[vgm.first];
    visualizer->initCameraParameters();
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->setBackgroundColor(0, 0, 0);
    // TODO change to support multiple callback handlers
    visualizer->registerKeyboardCallback(&Visualizer::keyboardEventCloudViewer, *this);

    visualizer->addText(VisualizationAnnotatorMgr->getCurrentVisualizableName(), 2, 20, 12, 1, 1, 1, annotatorName);

    visualizer->spinOnce();
    visualizer->setSize(1280, 960);
  }

  while (ros::ok())
  {
    for (auto vgm : visualizableGroupManagers_)
    {
      auto& VisualizationAnnotatorMgr = vgm.second;
      auto& visualizer = visualizers[vgm.first];

      VisualizationAnnotatorMgr->checkVisualizable();

      if (VisualizationAnnotatorMgr->updateCloud)
      {
        if (VisualizationAnnotatorMgr->changedVisualizable)
        {
          visualizer->removeAllPointClouds();
          visualizer->removeAllShapes();
          const std::string annotatorName = "annotatorName-" + vgm.first;
          visualizer->addText(VisualizationAnnotatorMgr->getCurrentVisualizableName(), 2, 20, 12, 1, 1, 1,
                              annotatorName);
        }
        if (VisualizationAnnotatorMgr->getCurrentVisualizable()->fillVisualizer(
                *visualizer, VisualizationAnnotatorMgr->changedVisualizable))
        {
          VisualizationAnnotatorMgr->updateCloud = false;
          VisualizationAnnotatorMgr->changedVisualizable = false;
        }
      }
      visualizer->spinOnce(10);
    }

    if (save)
    {
      save = false;
      if (visualizableGroupManagers_.count(saveVisualizerWithIdentifier) == 0)
      {
        outError("Trying to save a cloud but we can't map the entered input from the window to a Visualizer.");
        break;
      }
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZRGBA>());  // this was in the initialization before, i'm not sure if it's
                                                      // really needed there.
      saveCloud(cloud, visualizers[saveVisualizerWithIdentifier]);
      saveVisualizerWithIdentifier = "";
    }

  }  // end of ros::ok while loop
  for (auto vgm : visualizableGroupManagers_)
  {
    auto& visualizer = visualizers[vgm.first];
    visualizer->close();
    visualizer->spinOnce(10);
  }
}

void Visualizer::keyboardEventImageViewer(const cv::Mat& disp)
{
  int key;
#if CV_MAJOR_VERSION == 3
  key = cv::waitKeyEx(10);
#else
  key = cv::waitKey(10);
#endif
  // Not sure if key == 0 is there for legacy reasons, but according to
  // https://docs.opencv.org/3.4/d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7
  // -1 denotes 'no key was pressed'
  if (key == 0 || key == -1)
  {
    return;
  }
  bool success = false;
  auto vgmInteractedWith =
      getAnnotatorManagerForActiveWindow(success, Visualizable::VisualizableDataType::IMAGE_VIEWER);
  if (!success)
  {
    // We couldn't guess the active annotator from the window titles. We'll
    // use the first VAM as a fallback
    outError("Couldn't fetch the active Annotator from the window titles. Will forward to the first AAE.");
    vgmInteractedWith = visualizableGroupManagers_.begin()->second;
  }
  int lowerByteOfKey = key & 0xFF;
  switch (lowerByteOfKey)
  {
    case 110:  // next (n)
      vgmInteractedWith->nextVisualizable();
      break;
    case 112:  // previous (p)
      vgmInteractedWith->prevVisualizable();
      break;
    case 99:  // insert
      saveImageToDisk = true;
      imageVgmToBeSaved = vgmInteractedWith;
      break;
  }
  if (lowerByteOfKey == 27)
  {  // Escape
    shutdown();
  }
  else
  {
    callbackKeyHandler(lowerByteOfKey, DrawingAnnotator::IMAGE_VIEWER, vgmInteractedWith);
  }
}

void Visualizer::keyboardEventCloudViewer(const pcl::visualization::KeyboardEvent& event, void*)
{
  bool success = false;
  auto vgmInteractedWith = getAnnotatorManagerForActiveWindow(success, DrawingAnnotator::CLOUD_VIEWER);
  if (!success)
  {
    // We couldn't guess the active annotator from the window titles. We'll
    // use the first VAM as a fallback
    outError("Couldn't fetch the active Annotator from the window titles. Will forward to the first AAE.");
    vgmInteractedWith = visualizableGroupManagers_.begin()->second;
  }

  if (event.keyUp())
  {
    if (event.getKeySym() == "Left")
    {
      vgmInteractedWith->nextVisualizable();
    }
    else if (event.getKeySym() == "Right")
    {
      vgmInteractedWith->prevVisualizable();
    }
    else if (event.getKeySym() == "Escape")
    {
      shutdown();
    }
    else if (event.getKeySym() == "Insert")
    {
      save = true;
      saveVisualizerWithIdentifier = vgmInteractedWith->getIdentifier();
    }
    else if (event.getKeyCode() > 0)
    {
      callbackKeyHandler(event.getKeyCode(), DrawingAnnotator::CLOUD_VIEWER, vgmInteractedWith);
    }
  }
}

void Visualizer::saveImage(const cv::Mat& disp, std::shared_ptr<VisualizableGroupManager> vgm)
{
  std::lock_guard<std::mutex> lock_guard(lock);
  std::ostringstream oss;
  oss << savePath << std::setfill('0') << std::setw(5) << saveFrameImage << "_" << vgm->getCurrentVisualizableName()
      << ".png";

  outInfo("saving image: " << oss.str());
  cv::imwrite(oss.str(), disp, saveParams);
  ++saveFrameImage;
}

void Visualizer::saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                           pcl::visualization::PCLVisualizer::Ptr& visualizer)
{
  auto firstVizAnnoMgrAnnotator = visualizableGroupManagers_.begin()->second;
  std::lock_guard<std::mutex> lock_guard(lock);
  std::ostringstream oss, oss_cloud;
  oss_cloud << savePath << std::setfill('0') << std::setw(5) << saveFrameCloud << "_"
            << firstVizAnnoMgrAnnotator->getCurrentVisualizableName() << ".pcd";
  oss << savePath << std::setfill('0') << std::setw(5) << saveFrameCloud << "_"
      << firstVizAnnoMgrAnnotator->getCurrentVisualizableName() << ".png";

  outInfo("saving cloud: " << oss_cloud.str());
  //  pcl::io::savePCDFileASCII(oss.str(), *cloud);
  outInfo("saving screenshot: " << oss.str());
  visualizer->saveScreenshot(oss.str());
  ++saveFrameCloud;
}

/**
 * Source: https://stackoverflow.com/a/478960
 * Full example on https://gist.github.com/Sanic/10fb2517000985f53757b74e843004dc
 */
std::string Visualizer::exec(const char* cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe)
  {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
  {
    result += buffer.data();
  }
  return result;
}

std::string Visualizer::getActiveWindowTitle()
{
  return exec("xprop -id $(xprop -root _NET_ACTIVE_WINDOW | cut -d ' ' -f 5) WM_NAME | awk -F '\"' '{print $2}' ");
}

std::shared_ptr<VisualizableGroupManager>
Visualizer::getAnnotatorManagerForActiveWindow(bool& success, const Visualizable::VisualizableDataType windowType)
{
  success = false;
  std::string active_window_title = getActiveWindowTitle();

  for (auto vgm : visualizableGroupManagers_)
  {
    // Check if the active window title starts with the name of the window names the different VGMs should have
    if (windowType == Visualizable::VisualizableDataType::IMAGE_VIEWER &&
        active_window_title.rfind(imageWindowName(*(vgm.second)), 0) == 0)
    {
      success = true;
      return vgm.second;
    }

    if (windowType == Visualizable::VisualizableDataType::CLOUD_VIEWER &&
        active_window_title.rfind(cloudWindowName(*(vgm.second)), 0) == 0)
    {
      success = true;
      return vgm.second;
    }
  }

  return NULL;
}
