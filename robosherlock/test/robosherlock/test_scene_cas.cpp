#include <string>
#include <gtest/gtest.h>


#include <robosherlock/scene_cas.h>
#include <robosherlock/CASConsumerContext.h>

#include <robosherlock/flowcontrol/RSAggregateAnalysisEngine.h>
#include "../main.h"

class SceneCasTest : public testing::Test
{
protected:
  SceneCasTest()
  {
  }
  virtual void SetUp()
  {
    // Do something
  }

  virtual void TearDown()
  {
      rs::CASConsumerContext::getInstance().clearCASes();
  }
};


TEST_F(SceneCasTest, getCASAsString)
{
  rs::SceneCas this_cas(*cas);

  std::string casAsStringE = this_cas.getCASAsString(*cas, false);
  std::string testString = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<CAS>\n</CAS>\n";
  EXPECT_EQ(testString, casAsStringE);
}

