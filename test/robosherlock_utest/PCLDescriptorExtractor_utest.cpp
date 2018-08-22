#include <gtest/gtest.h>
#include <rs/scene_cas.h>
#include <rs/flowcontrol/RSAnalysisEngine.h>
#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"

void processPCL(uima::CAS *cas)
{
  rs::SceneCas sceneCas(*cas);
  if (cas == NULL) outError("The CAS is null");
  rs::Scene scene = sceneCas.getScene();
  std::vector<rs::Cluster> clusters;
  scene.identifiables.filter(clusters);
  EXPECT_TRUE(clusters.size()>0);
  //Cluster3DGeometry
  for (int i = 0; i<clusters.size();i++)
  {
    std::vector<rs::PclFeature> annotations;
    clusters[i].annotations.filter(annotations);
    for (int j = 0; j<annotations.size();j++)
    {
      EXPECT_TRUE(annotations[j].feat_type.get() != NULL );
    }
  }
}

void PCLDescriptorExtractorTest()
{

  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator","PlaneAnnotator","PointCloudClusterExtractor","PCLDescriptorExtractor"};
  engine.setPipelineOrdering(engineList);
  
  engine.overwriteParam("PCLDescriptorExtractor","descriptorType", std::string("VFH"));
  engine.reconfigure();
  engine.process();
  cas = engine.getCas();
  processPCL(cas);	
  
  engine.overwriteParam("PCLDescriptorExtractor","descriptorType",std::string("CVFH"));
  engine.reconfigure();
  engine.process();
  cas = engine.getCas();
  processPCL(cas);
  
  engine.overwriteParam("PCLDescriptorExtractor","descriptorType",std::string("OUR-CVFH"));
  engine.reconfigure();
  engine.process();
  cas = engine.getCas();
  processPCL(cas);
  
  
}

TEST(UnitTest,PCLDescriptorExtractor)
{
  PCLDescriptorExtractorTest();
}
