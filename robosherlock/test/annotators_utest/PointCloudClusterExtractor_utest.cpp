
#include <gtest/gtest.h>
#include <rs/scene_cas.h>
#include <rs/flowcontrol/RSAnalysisEngine.h>
#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"

void processCluster(uima::CAS *cas)
{
  rs::SceneCas sceneCas(*cas);
  if (cas == NULL) outError("The CAS is null");
  rs::Scene scene = sceneCas.getScene();
  std::vector<rs::ObjectHypothesis> clusters;
  scene.identifiables.filter(clusters);
  EXPECT_TRUE(clusters.size()>0);
  //Cluster3DGeometry
  for (int i = 0; i<clusters.size();i++)
  {
    rs::ObjectHypothesis &cluster = clusters[i];
    pcl::PointIndicesPtr indices(new pcl::PointIndices());
    rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *indices);
    cv::Rect roi;
    rs::conversion::from(cluster.rois().roi(), roi);
    EXPECT_TRUE(roi.width>0);
    EXPECT_TRUE(roi.height>0);
  }
}

void pointCloudExtractorTest()
{

  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator","PointCloudFilter","PlaneAnnotator","PointCloudClusterExtractor"};
  engine.setPipelineOrdering(engineList);
  engine.resetCas();
  engine.overwriteParam("PointCloudClusterExtractor","mode", std::string("OEC"));
  engine.reconfigure();
  engine.process();
  cas = engine.getCas();
  processCluster(cas);	
  engine.resetCas();
  engine.overwriteParam("PointCloudClusterExtractor","mode",std::string("EC"));
  engine.reconfigure();
  engine.process();
  cas = engine.getCas();
  processCluster(cas);
  engine.resetCas();
  engine.overwriteParam("PointCloudClusterExtractor","mode",std::string("OEC_prism"));
  engine.reconfigure();
  engine.process();
  cas = engine.getCas();
  processCluster(cas);
  
  
}

TEST(UnitTest,PointCloudTest)
{
  pointCloudExtractorTest();
}

