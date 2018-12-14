
#include <gtest/gtest.h>
#include <rs/scene_cas.h>
#include <rs/flowcontrol/RSAnalysisEngine.h>
#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"


void SacModelAnnotator()
{
  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator","PointCloudFilter","PlaneAnnotator","PointCloudClusterExtractor","SacModelAnnotator"};
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
  

  for ( std::vector<rs::ObjectHypothesis>::iterator it = clusters.begin(); it!=clusters.end();++it)
  {
      std::vector<rs::Shape> shape_annotation;
      it->annotations.filter(shape_annotation);
      for (int i = 0; i<shape_annotation.size();i++)
      {
	
	EXPECT_TRUE(shape_annotation[i].shape.get()!="");
      }
      
  }
  
  
}

TEST(UnitTest,SacModelAnnotator)
{
  SacModelAnnotator();
}
