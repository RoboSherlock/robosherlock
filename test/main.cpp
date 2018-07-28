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

   std::string enginePath;
   rs::common::getAEPaths(std::string("u_test"), enginePath);	
   engine_file = enginePath;

   uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
   resourceManager.setLoggingLevel(uima::LogStream::EnError);
  
   engine = uima::Framework::createAnalysisEngine(engine_file.c_str(), errorInfo);

   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}











