
#include <gtest/gtest.h>
#include <rs/scene_cas.h>
#include <rs/flowcontrol/RSAnalysisEngine.h>
#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"


void MLNInferencerTest()
{
/*
  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator","PlaneAnnotator","MLNInferencer"};
  engine.getPipelineManager()->setPipelineOrdering(engineList);
  engine.resetCas();
  engine.process();
  cas = engine.getCas();
  
  rs::SceneCas sceneCas(*cas);
  if (cas == NULL) outError("The CAS is null");
  rs::Scene scene = sceneCas.getScene();
  std::vector<rs::ObjectHypothesis> clusters;
  scene.identifiables.filter(clusters);*/
  //EXPECT_TRUE(clusters.size()>0);
  /*

  for ( std::vector<rs::ObjectHypothesis>::iterator it = clusters.begin(); it!=clusters.end();++it)
  {
      std::vector<rs::MLNAtoms> mln_atoms;
      it->annotations.filter(mln_atoms);
      for (int i = 0; i<mln_atoms.size();i++)
      {
	outError(mln_atoms.size());	
	for (int j = 0; j<mln_atoms[i].atoms.size();j++)
	{
	 //outError(mln_atoms[i].atoms[j]);
	}
	//EXPECT_TRUE(mln_atoms[i].atoms.get()!="");
      }
      
  }*/
  
  
}

TEST(UnitTest,MLNInferencer)
{
  MLNInferencerTest();
}