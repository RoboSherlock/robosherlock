#include <gtest/gtest.h>

#include <rs/utils/common.h>
<<<<<<< HEAD
#include <rs/utils/RSAnalysisEngineManager.h>
#include <rs/utils/RSAnalysisEngine.h>
=======
#include "rs/flowcontrol/RSAnalysisEngineManager.h"
#include "rs/flowcontrol/RSAnalysisEngine.h"
>>>>>>> 717a459bb12792f07de17b042739305eb08de5e6


int main(int argc, char **argv)
{
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
