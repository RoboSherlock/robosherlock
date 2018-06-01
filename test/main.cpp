#include <gtest/gtest.h>

#include <rs/utils/common.h>
#include "rs/flowcontrol/RSAnalysisEngineManager.h"
#include "rs/flowcontrol/RSAnalysisEngine.h"



int main(int argc, char **argv)
{
  outInfo("before");
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
