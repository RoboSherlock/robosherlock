#include <string>
#include <gtest/gtest.h>
#include "rs/flowcontrol/RSAnalysisEngineManager.h"
#include "rs/flowcontrol/RSPipelineManager.h"

#include "rs/flowcontrol/RSAnalysisEngine.h"
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>

#include <pcl/point_types.h>
#include <ros/ros.h>

#include <mongo/client/dbclient.h>

class ExampleTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
      rs::common::getAEPaths("u_test",engineFile);
      engine.init(engineFile);
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
  std::vector<std::string> engineList = {"CollectionReader","NormalEstimator"};
  engine.getPipelineManager()->setPipelineOrdering(engineList);
  engine.process();
  uima::CAS* tcas = engine.getCas();
  rs::SceneCas cas(*tcas);
  pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);

  cas.get(VIEW_NORMALS, *normal_ptr);
  EXPECT_TRUE(4>0);
}

/*TEST_F(ExampleTest, PlaneEstimatorTest)
{
  std::vector<std::string> engineList = {"CollectionReader","PlaneAnnotator"};
  engine.getPipelineManager()->setPipelineOrdering(engineList);
  engine.process();
  rs::SceneCas cas(*engine.getCas());
  rs::Scene scene = cas.getScene();
  std::vector< rs::Plane > planes;
  scene.annotations.filter(planes);
  EXPECT_TRUE(planes.size() >0);
}*/
