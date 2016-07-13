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
    outInfo("Creating resource manager"); // TODO: DEBUG
    uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock"); // TODO: change topic?

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
