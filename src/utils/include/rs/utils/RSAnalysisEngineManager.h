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

#ifndef RSANALYSISENGINEMANAGER_H
#define RSANALYSISENGINEMANAGER_H

#include <rs/utils/RSAnalysisEngine.h>
#include <rs/io/visualizer.h>

template <class AEType>
class RSAnalysisEngineManager
{
protected:
  std::vector<AEType> engines;

  const bool useVisualizer;
  rs::Visualizer visualizer;

public:
  RSAnalysisEngineManager(const bool useVisualizer, const std::string &savePath): useVisualizer(useVisualizer), visualizer(savePath)
  {
    // Create/link up to a UIMACPP resource manager instance (singleton)
    outInfo("Creating resource manager");
    uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");

    switch(OUT_LEVEL)
    {
    case OUT_LEVEL_NOOUT:
    case OUT_LEVEL_ERROR:
      resourceManager.setLoggingLevel(uima::LogStream::EnError);
      break;
    case OUT_LEVEL_INFO:
      resourceManager.setLoggingLevel(uima::LogStream::EnWarning);
      break;
    case OUT_LEVEL_DEBUG:
      resourceManager.setLoggingLevel(uima::LogStream::EnMessage);
      break;
    }
  }

  ~RSAnalysisEngineManager()
  {
    uima::ResourceManager::deleteInstance();
  }

  void init(const std::vector<std::string> &files)
  {
    engines.resize(files.size());
    for(size_t i = 0; i < engines.size(); ++i)
    {
      engines[i].init(files[i]);
    }
    if(useVisualizer)
    {
      visualizer.start();
    }
  }

  virtual void run()
  {
    for(; ros::ok();)
    {
      for(size_t i = 0; i < engines.size(); ++i)
      {
        engines[i].process();
        engines[i].resetCas();
      }
    }
  }

  void stop()
  {
    if(useVisualizer)
    {
      visualizer.stop();
    }
    for(size_t i = 0; i < engines.size(); ++i)
    {
      engines[i].resetCas();
      engines[i].stop();
    }
  }
};


#endif // RSANALYSISENGINEMANAGER_H
