#include <gtest/gtest.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/flowcontrol/RSAggregateAnalysisEngine.h>

#include "../main.h"


void regionFilterTest()
{

  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor", "RegionFilter", "PlaneAnnotator","PointCloudClusterExtractor"};
  engine->setPipelineOrdering(engineList);
  engine->resetCas();
  engine->processOnce();
  cas = engine->getCas();
  
  if (cas == NULL) outError("The CAS is null");
  rs::SceneCas sceneCas(*cas);
  
  rs::Scene scene = sceneCas.getScene();
  std::vector<rs::ObjectHypothesis> obj_hyps;
  scene.identifiables.filter(obj_hyps);
  EXPECT_TRUE(obj_hyps.size()>0);

  std::vector<rs::Plane> planes;
  scene.annotations.filter(planes);
  EXPECT_TRUE(planes.size() > 0);
  EXPECT_TRUE(planes[0].inliers().size() > 0);
}

TEST(RegionFilterUnitTest,PlaneAndClusters)
{
  regionFilterTest();
}
