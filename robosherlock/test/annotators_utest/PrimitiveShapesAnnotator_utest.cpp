
#include <gtest/gtest.h>
#include <rs/scene_cas.h>
#include <rs/flowcontrol/RSAnalysisEngine.h>
#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"


void PrimitiveShapeAnnotator()
{

  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator","PointCloudFilter","PlaneAnnotator","PointCloudClusterExtractor","ClusterMerger","PrimitiveShapeAnnotator"};
  engine.setPipelineOrdering(engineList);
  engine.resetCas();
  engine.process();
  cas = engine.getCas();
  
  rs::SceneCas sceneCas(*cas);
  if (cas == NULL) outError("The CAS is null");
  rs::Scene scene = sceneCas.getScene();
  std::vector<rs::ObjectHypothesis> clusters;
  scene.identifiables.filter(clusters);
  EXPECT_TRUE(clusters.size()>0);
  

  
  for (int i = 0; i<clusters.size();i++)
  {
    rs::ObjectHypothesis &cluster = clusters[i];
    std::vector<rs::Shape> shape_annot;
    cluster.annotations.filter(shape_annot);
    EXPECT_TRUE(shape_annot.size()>0);
    for ( int j = 0; j<shape_annot.size();j++)
    {
       EXPECT_TRUE(shape_annot[j].shape.get()!="");
    }
  }
  
  
}

TEST(UnitTest,PrimitiveShapeAnnotator)
{
  PrimitiveShapeAnnotator();
}
