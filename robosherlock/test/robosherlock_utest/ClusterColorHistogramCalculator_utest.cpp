
#include <gtest/gtest.h>
#include <rs/scene_cas.h>
#include <rs/flowcontrol/RSAnalysisEngine.h>
#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"


void clusterColorHistogramCalculator()
{

  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator","PointCloudFilter","PlaneAnnotator","PointCloudClusterExtractor","ClusterMerger","ClusterColorHistogramCalculator"};
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
    std::vector<rs::ColorHistogram> color_histogram;
   
    cluster.annotations.filter(color_histogram);
    EXPECT_TRUE(color_histogram.size()>0);
    for ( int j = 0; j<color_histogram.size();j++)
    {
       rs::Mat hist = color_histogram[j].hist.get();	
       EXPECT_TRUE(hist.cols.get()>0);
       EXPECT_TRUE(hist.rows.get()>0);
    }
  }
  
  
}

TEST(UnitTest,ClusterColorHistogramCalculator)
{
  clusterColorHistogramCalculator();
}
