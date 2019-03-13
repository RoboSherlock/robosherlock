#include <gtest/gtest.h>

#include <rs/utils/common.h>
#include <rs/flowcontrol/RSAggregateAnalysisEngine.h>

#include <mongo/client/dbclient.h>

#include "main.h"


mongo::client::GlobalInstance instance;
std::string engineFile;
RSAggregateAnalysisEngine *engine;
uima::CAS *cas;

int main(int argc, char **argv)
{
  char *userEnv = getenv("USER");
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
  resourceManager.setLoggingLevel(uima::LogStream::EnError);
  rs::common::getAEPaths("u_test", engineFile);
  engine = rs::createRSAggregateAnalysisEngine(engineFile,true, false);


  std::string analysisEnginesArg, savePath;
  std::vector<std::string> analysisEngines;
  std::vector<std::string> analysisEngineFiles;
  std::ostringstream engineList;
  std::string engine_file;

  uima::ErrorInfo errorInfo;
  mongo::client::GlobalInstance instance;

  if(userEnv != NULL)
  {
    ros::init(argc, argv, std::string("RoboSherlock_") + std::string(userEnv));

  }
  else
  {
    ros::init(argc, argv, std::string("RoboSherlock"));

  }
  cas = engine->newCAS();

  testing::InitGoogleTest(&argc, argv);
  RUN_ALL_TESTS();
  engine->destroy();

  delete cas;
  delete engine;

  return 1;
}











