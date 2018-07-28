#include <string>
#include <gtest/gtest.h>
#include <rs/flowcontrol/RSAnalysisEngineManager.h>
#include <rs/flowcontrol/RSPipelineManager.h>

#include <rs/flowcontrol/RSAnalysisEngine.h>
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>

#include <rs/utils/array_utils.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

class SegmentationTest : public ::testing::Test
{
protected:

  std::string engineFile;
  RSAnalysisEngine engine;

  float segment_similarity_threshold;
  float overall_segment_threshold;

  std::vector<std::string> engineList = {"CollectionReader",
                                         "ImagePreprocessor",
                                         "PointCloudFilter",
				                                 "NormalEstimator",
                                         "PlaneAnnotator",
                                         "OverSegmentationAnnotator",
                                         "RotationalSymmetryAnnotator",
                                         "RotationalSymmetrySegmentation",
                                         "OverSegmentationAnnotator",
                                         "BilateralSymmetryAnnotator",
                                         "BilateralSymmetrySegmentation"};

  virtual void SetUp()
  {
    rs::common::getAEPaths("object_segmentation", engineFile);
    engine.init(engineFile);
    engine.initPipelineManager();
    engine.getPipelineManager()->setPipelineOrdering(engineList);

  }

  virtual void TearDown()
  {
    //clean up
    engine.getPipelineManager()->resetPipelineOrdering();
    engine.resetCas();
    engine.stop();
  }

  //there is no ground truth for this test, so for simply we just test if there are segments
  inline float test()
  {
    uima::CAS* tcas = engine.getCas();
    rs::SceneCas cas(*tcas);

    //main pipeline execution
    engine.process();

    //get segments data
    std::vector<pcl::PointIndices> rotational_segments;
    std::vector<pcl::PointIndices> bilateral_segments;
    std::vector< std::vector<int> > total_segments;
    cas.get(VIEW_ROTATIONAL_SEGMENTATION_IDS, rotational_segments);
    cas.get(VIEW_BILATERAL_SEGMENTATION_IDS, bilateral_segments);

    return rotational_segments.size() + bilateral_segments.size();
  }
};


TEST_F(SegmentationTest, SymmetrySegmentationTest1)
{

  int numSegments = test();
  ASSERT_TRUE(numSegments > 0);
}
