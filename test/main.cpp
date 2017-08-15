#include <gtest/gtest.h>

#include <rs/utils/common.h>
#include <rs/utils/RSAnalysisEngineManager.h>
#include <rs/utils/RSAnalysisEngine.h>


int main(int argc, char **argv)
{
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
