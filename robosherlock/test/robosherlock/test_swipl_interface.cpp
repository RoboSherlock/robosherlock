#include <string>
#include <gtest/gtest.h>

#include <uima/api.hpp>

#include <rs/flowcontrol/RSAnalysisEngine.h>
#include <rs/flowcontrol/RSAggregateAnalysisEngine.h>
#include <rs/queryanswering/SWIPLInterface.h>
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>

#include <pcl/point_types.h>
#include <ros/ros.h>

#include <iostream>

class SWIPLInterfaceTest : public testing::Test
{
protected:

  std::shared_ptr<rs::SWIPLInterface> ke;
  virtual void SetUp()
  {
    ke = std::make_shared<rs::SWIPLInterface>();
  }

  virtual void TearDown()
  {
    ke->releaseEngine();
    ke.reset();
  }
};

//q_subClassOf
TEST_F(SWIPLInterfaceTest, SubclassOfTest)
{
  bool istrue = ke->q_subClassOf("CollectionReader", "RoboSherlockComponent");
  EXPECT_TRUE(istrue == true);
}


TEST_F(SWIPLInterfaceTest, PlanPipelinTest)
{
  bool success = ke->assertTestPipelnie();
  EXPECT_TRUE(success == true);
  std::vector<std::string> keys = {"shape"};
  std::vector<std::string> pipeline;
  ke->planPipelineQuery(keys, pipeline);
  std::for_each(pipeline.begin(), pipeline.end(), [](std::string & p)
  {
    outInfo(p);
  });
  EXPECT_TRUE(pipeline.end() != std::find(pipeline.begin(), pipeline.end(), "PrimitiveShapeAnnotator"));
}

TEST_F(SWIPLInterfaceTest, ReatractTest)
{
  bool retract_KvPs = ke->retractQueryKvPs();
  bool retract_annotators = ke->retractAllAnnotators();

  EXPECT_TRUE(retract_KvPs == true);
  EXPECT_TRUE(retract_annotators == true);
}
