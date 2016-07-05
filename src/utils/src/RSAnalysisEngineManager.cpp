#include <rs/utils/RSAnalysisEngineManager.h>


template <class AEType>
RSAnalysisEngineManager<AEType>::RSAnalysisEngineManager(const bool useVisualizer, const std::string &savePath) : useVisualizer(useVisualizer), visualizer(savePath)
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

template <class AEType>
RSAnalysisEngineManager<AEType>::~RSAnalysisEngineManager()
{
  uima::ResourceManager::deleteInstance();
}

template <class AEType>
void RSAnalysisEngineManager<AEType>::init(const std::vector<std::string> &files)
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

template <class AEType>
void RSAnalysisEngineManager<AEType>::run()
{
  for(; ros::ok();)
  {
    for(size_t i = 0; i < engines.size(); ++i)
    {
      engines[i].process();
    }
  }
}

template <class AEType>
void RSAnalysisEngineManager<AEType>::stop()
{
  if(useVisualizer)
  {
    visualizer.stop();
  }
  for(size_t i = 0; i < engines.size(); ++i)
  {
    engines[i].stop();
  }
}



template class RSAnalysisEngineManager<RSAnalysisEngine>;
