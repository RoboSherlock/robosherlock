#include <gtest/gtest.h>

#include <rs/utils/common.h>
#include "rs/flowcontrol/RSAnalysisEngineManager.h"
#include "rs/flowcontrol/RSAnalysisEngine.h"

#include <mongo/client/dbclient.h>

#include "main.h"


  uima::AnalysisEngine *engine;

int main(int argc, char **argv)
{
  char *userEnv = getenv("USER");
  std::string analysisEnginesArg, savePath;
  std::vector<std::string> analysisEngines;
  std::vector<std::string> analysisEngineFiles;
  std::ostringstream engineList;
  std::string engine_file;
  
  uima::ErrorInfo errorInfo;
   mongo::client::GlobalInstance instance;
  if(userEnv!=NULL)
  {
    ros::init(argc, argv, std::string("RoboSherlock_") +std::string(userEnv));

  }
  else
  {
    ros::init(argc, argv, std::string("RoboSherlock"));

  }
  ros::NodeHandle priv_nh;
  priv_nh = ros::NodeHandle("~");

  //ros::NodeHandle priv_nh = ros::NodeHandle("~");

  priv_nh.param("ae", analysisEnginesArg, std::string("u_test"));
  priv_nh.param("analysis_engines", analysisEnginesArg, analysisEnginesArg);
  
  analysisEngines.push_back(analysisEnginesArg);
   for(int i = 0; i < analysisEngines.size(); ++i)
   {
    const std::string &engine = analysisEngines[i];
    std::string engineFile;
    rs::common::getAEPaths(engine,engineFile);
    if(engineFile.empty())
    {
      outError("analysis engine \"" << engine << "\" not found.");
      return -1;
    }
    else
    {
      analysisEngineFiles.push_back(engineFile);
    }
    engineList << FG_CYAN << engine << (i + 1 < analysisEngines.size() ? NO_COLOR ", " : NO_COLOR);
  }
  
  
    engine_file = analysisEngineFiles[0];

  //To be asked what this does
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
  resourceManager.setLoggingLevel(uima::LogStream::EnError);
  
  engine = uima::Framework::createAnalysisEngine(engine_file.c_str(), errorInfo);

  
  
  
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}











