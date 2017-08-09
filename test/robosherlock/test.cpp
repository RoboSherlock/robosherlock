#include <string>
#include <gtest/gtest.h>
#include "rs/utils/RSAnalysisEngineManager.h"
#include "rs/utils/RSPipelineManager.h"

#include "rs/utils/RSAnalysisEngine.h"
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>

#include <pcl/point_types.h>
#include <ros/ros.h>

class ExampleTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        rs::common::getAEPaths("test",engineFile);
        engine.init(engineFile);
        engine.initPipelineManager();
        engine.getPipelineManager()->setPipelineOrdering(engineList);
    }

    virtual void TearDown(){}
    std::string engineFile;
    RSAnalysisEngine engine;

    std::vector<std::string> engineList = {"CollectionReader","NormalEstimator"};

};


TEST_F(ExampleTest, ProcessTest){

  engine.process();
  uima::CAS* tcas = engine.getCas();
  rs::SceneCas cas(*tcas);
  pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);

  cas.get(VIEW_NORMALS, *normal_ptr);
  EXPECT_TRUE(normal_ptr->points.size()>0);
}
