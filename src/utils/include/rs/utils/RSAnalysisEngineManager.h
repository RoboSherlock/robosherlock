#ifndef RSANALYSISENGINEMANAGER_H
#define RSANALYSISENGINEMANAGER_H

#include <rs/utils/RSAnalysisEngine.h>
#include <rs/io/visualizer.h>

template <class AEType>
class RSAnalysisEngineManager
{
private:
  std::vector<AEType> engines;

  const bool useVisualizer;
  rs::Visualizer visualizer;

public:
  RSAnalysisEngineManager(const bool useVisualizer, const std::string &savePath);

  ~RSAnalysisEngineManager();


  void init(const std::vector<std::string> &files);

  void run();

  void stop();
};


#endif // RSANALYSISENGINEMANAGER_H
