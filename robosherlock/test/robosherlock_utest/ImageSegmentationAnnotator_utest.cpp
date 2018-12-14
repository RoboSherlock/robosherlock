#include <gtest/gtest.h>
#include <rs/scene_cas.h>
#include <rs/flowcontrol/RSAnalysisEngine.h>
#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"



void imageSegmentationAnnotatorTest()
{

  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator","PointCloudFilter","PlaneAnnotator","ImageSegmentationAnnotator"};
  engine.setPipelineOrdering(engineList);
  engine.resetCas();
  engine.process();
  cas = engine.getCas();
  rs::SceneCas sceneCas(*cas);
  if (cas == NULL) outError("The CAS is null");
  rs::Scene scene = sceneCas.getScene();
  std::vector<rs::ObjectHypothesis> clusters;
  scene.identifiables.filter(clusters);
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

TEST(UnitTest,ImageSegmentationAnnotator)
{
  imageSegmentationAnnotatorTest();
}
