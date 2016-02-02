#ifndef RSANALYSISENGINEMANAGER_H
#define RSANALYSISENGINEMANAGER_H

#include <rs/utils/RSAnalysisEngine.h>
#include <rs/io/visualizer.h>

class RSAnalysisEngineManager
{
private:
  std::vector<RSAnalysisEngine> engines;

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
