#include <string>
#include <gtest/gtest.h>
#include "rs/flowcontrol/RSPipelineManager.h"

#include "rs/flowcontrol/RSAnalysisEngine.h"
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>

#include <pcl/point_types.h>
#include <ros/ros.h>

#include <mongo/client/dbclient.h>

#include <iostream>

class ExampleTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
      rs::common::getAEPaths("u_test",engineFile);
      engine.init(engineFile, false); // do not run parallel for now
      engine.initPipelineManager();
    }

    virtual void TearDown()
    {
       engine.stop();
    }
    mongo::client::GlobalInstance instance;
    std::string engineFile;
    RSAnalysisEngine engine;
};

TEST_F(ExampleTest, ProcessTest)
{
  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator"};
  engine.getPipelineManager()->setPipelineOrdering(engineList);
  engine.process();
  uima::CAS* tcas = engine.getCas();
  rs::SceneCas cas(*tcas);
  pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);
  cas.get(VIEW_NORMALS, *normal_ptr);
  engine.getPipelineManager()->resetPipelineOrdering();
  EXPECT_TRUE(normal_ptr->points.size()>0);
}

TEST_F(ExampleTest, PlaneEstimatorTest)
{
  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","PlaneAnnotator"};
  engine.getPipelineManager()->setPipelineOrdering(engineList);
  engine.process();
  rs::SceneCas cas(*engine.getCas());
  rs::Scene scene = cas.getScene();
  std::vector< rs::Plane > planes;
  scene.annotations.filter(planes);
  engine.getPipelineManager()->resetPipelineOrdering();
  EXPECT_TRUE(planes.size() >0);
}
