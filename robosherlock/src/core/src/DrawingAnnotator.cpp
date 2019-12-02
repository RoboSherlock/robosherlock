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

#include <rs/DrawingAnnotator.h>
#include <rs/utils/exception.h>

std::map<std::string, DrawingAnnotator *> DrawingAnnotator::annotators;

DrawingAnnotator::DrawingAnnotator(const std::string &name) : name(name), update(false), hasRun(false)
{
  outDebug("Added: " << name);
  annotators[name] = this;
}

DrawingAnnotator::~DrawingAnnotator()
{
  std::map<std::string, DrawingAnnotator *>::const_iterator it;
  for(it = annotators.begin(); it != annotators.end(); ++it)
  {
    if(it->second == this)
    {
      annotators.erase(it);
      break;
    }
  }
}

void DrawingAnnotator::getAnnotatorNames(std::vector<std::string> &names)
{
  std::map<std::string, DrawingAnnotator *>::const_iterator it;
  names.clear();
  names.reserve(annotators.size());

  for(it = annotators.begin(); it != annotators.end(); ++it)
  {
    names.push_back(it->first);
  }
}

DrawingAnnotator *DrawingAnnotator::getAnnotator(const std::string &name)
{
  std::map<std::string, DrawingAnnotator *>::const_iterator it;
  it = annotators.find(name);
  if(it != annotators.end())
  {
    return it->second;
  }
  return NULL;
}

uima::TyErrorId DrawingAnnotator::process(uima::CAS &tcas, uima::ResultSpecification const &res_spec)
{
  uima::TyErrorId ret = UIMA_ERR_UNKNOWN_TYPE;
  drawLock.lock();
  try
  {
    ret = processWithLock(tcas, res_spec);
    update = true;
    hasRun = true;
  }
  catch(const uima::Exception &e)
  {
    outError("Exception in " << name << ": " << e);
    drawLock.unlock();
    throw e;
  }
  catch(const rs::FrameFilterException &e)
  {
    update = true;
    hasRun = true;
    drawLock.unlock();
    throw e;
  }
  catch(const rs::Exception &e)
  {
    outError("Exception in " << name << ": " << e.what());
    drawLock.unlock();
    throw e;
  }
  catch(...)
  {
    outError("Exception in " << name << "!");
  }
  drawLock.unlock();
  return ret;
}

void DrawingAnnotator::drawImage(cv::Mat &disp)
{
  if(!hasRun)
  {
    disp = cv::Mat::zeros(480, 640, CV_8UC3);
    return;
  }
  drawLock.lock();
  try
  {
    drawImageWithLock(disp);
  }
  catch(...)
  {
    outError("Exception in " << name << "!");
    if(disp.empty())
    {
      disp = cv::Mat::zeros(480, 640, CV_8UC3);
    }
  }
  drawLock.unlock();
}

bool DrawingAnnotator::fillVisualizer(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
{
  if(!hasRun)
  {
    return false;
  }
  drawLock.lock();
  try
  {
    fillVisualizerWithLock(visualizer, firstRun);
  }
  catch(...)
  {
    outError("Exception in " << name << "!");
  }
  drawLock.unlock();
  return true;
}

bool DrawingAnnotator::callbackMouse(const int event, const int x, const int y, const Source source)
{
  return false;
}

bool DrawingAnnotator::callbackKey(const int key, const Source source)
{
  return false;
}

void DrawingAnnotator::fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
{
  //empty function declaration so C+ does not complain like the litle bitch it is
}

void DrawingAnnotator::drawImageWithLock(cv::Mat &disp)
{
  disp = cv::Mat::zeros(480, 640, CV_8UC3);
}

int DrawingAnnotator::copyAnnotatorList(std::map<std::string, DrawingAnnotator *> &inMap) {
    inMap.clear();
    inMap.insert(annotators.begin(), annotators.end());

    return inMap.size();
}

void DrawingAnnotator::clearAnnotatorList() {
    annotators.clear();
}
