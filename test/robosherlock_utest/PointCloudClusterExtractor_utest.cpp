
#include <string.h>
#include <gtest/gtest.h>
#include <errno.h>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <uima/fsfilterbuilder.hpp>

#include <ros/ros.h>
#include <rs/flowcontrol/RSProcessManager.h>
#include <rs/flowcontrol/RSAnalysisEngineManager.h>
#include <rs/utils/common.h>
#include <rs/io/Storage.h>

#include <rs/conversion/conversion.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <rs/scene_cas.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"



void pointCloudExtractorTest()
{

  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator","PlaneAnnotator","PointCloudClusterExtractor"};
  engine.getPipelineManager()->setPipelineOrdering(engineList);

  engine.process();
  cas = engine.getCas();
  rs::SceneCas sceneCas(*cas);
  if (cas == NULL) outError("The CAS is null");
  rs::Scene scene = sceneCas.getScene();
  std::vector<rs::Cluster> clusters;
  scene.identifiables.filter(clusters);
  EXPECT_TRUE(clusters.size()>0);
  //Cluster3DGeometry
  for (int i = 0; i<clusters.size();i++)
  {
    rs::Cluster &cluster = clusters[i];
    pcl::PointIndicesPtr indices(new pcl::PointIndices());
    rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *indices);
    cv::Rect roi;
    rs::conversion::from(cluster.rois().roi(), roi);
    EXPECT_TRUE(roi.width>0);
    EXPECT_TRUE(roi.height>0);
  }
  cv::Rect roi;
  
}

TEST(UnitTest,PointCloudTest)
{
  pointCloudExtractorTest();
}

