#include <string>
#include <gtest/gtest.h>

#include <robosherlock/queryanswering/SWIPLInterface.h>
#include <robosherlock/queryanswering/QueryInterface.h>
#include <robosherlock/utils/common.h>
#include <robosherlock/types/all_types.h>
#include <robosherlock/scene_cas.h>

#include <pcl/point_types.h>
#include <ros/ros.h>

#include <iostream>

class QueryInterfaceTest : public testing::Test
{
protected:

  std::shared_ptr<rs::SWIPLInterface> ke;
  std::shared_ptr<QueryInterface> qi;

  QueryInterfaceTest()
  {
    ke = std::make_shared<rs::SWIPLInterface>();
    qi = std::make_shared<QueryInterface>(ke);
  }
  virtual void SetUp()
  {

  }

  virtual void TearDown()
  {
    ke->releaseEngine();
    ke.reset();
    qi.reset();
  }
};

//q_subClassOf
TEST_F(QueryInterfaceTest, HandleDetect)
{
  bool istrue = ke->assertTestPipelnie();
  std::string query1 = "{\"detect\":{\"shape\":\"box\"}}";

  std::string query2 = "{\"detect:\"shape\";\"box\"}}";
  EXPECT_TRUE(istrue == true);
  EXPECT_TRUE(true == qi->parseQuery(query1));
  EXPECT_TRUE(false == qi->parseQuery(query2));
  std::vector<std::string> newPipelineOrder;
  qi->handleDetect(newPipelineOrder);
  for_each(newPipelineOrder.begin(), newPipelineOrder.end(), [](const std::string & p)
  {
    outInfo(p);
  });
  EXPECT_TRUE(std::find(newPipelineOrder.begin(), newPipelineOrder.end(), "PrimitiveShapeAnnotator") != newPipelineOrder.end());
}

