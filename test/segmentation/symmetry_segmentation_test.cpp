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
				         "NormalEstimator",
                                         "OverSegmentationAnnotator",
                                         "RotationalSymmetryAnnotator",
                                         "RotationalSymmetrySegmentation",
                                         "OverSegmentationAnnotator",
                                         "BilateralSymmetryAnnotator",
                                         "BilateralSymmetrySegmentation"};

  virtual void SetUp()
  {
    std::string ae_path = ros::package::getPath("robosherlock") + "/descriptors/analysis_engines/full_segmentation.xml";
    rs::common::getAEPaths(ae_path, engineFile);
    engine.init(engineFile);
    engine.initPipelineManager();
    engine.getPipelineManager()->setPipelineOrdering(engineList);

    segment_similarity_threshold = 0.7f; // meaning matching 8% ground truth points
    overall_segment_threshold = 0.75f; // meaning successfully segmenting 75% of presenting objects
  }

  virtual void TearDown()
  {
    //clean up
    engine.getPipelineManager()->resetPipelineOrdering();
    engine.resetCas();
    engine.stop();
  }

  inline float test(std::string cloudPath, std::string gtPath)
  {
    uima::CAS* tcas = engine.getCas();
    rs::SceneCas cas(*tcas);

    //read cloud dataset
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::string cloud_path = ros::package::getPath("robosherlock") + cloudPath;

    if(pcl::io::loadPCDFile (cloud_path, *cloud_ptr) == -1)
    {
      outError("Could not load cloud dataset for unit test. Check path again!");
    }

    cas.set(VIEW_CLOUD, *cloud_ptr);

    //read ground truth
    std::vector< std::vector<int> > ground_truth_ids;
    std::string ground_truth_path = ros::package::getPath("robosherlock") + gtPath;
    if(!readGroundTruth(ground_truth_path, ground_truth_ids))
    {
      outError("Could not read ground truth file!");
    }

    //main pipeline execution
    engine.process();

    //get segments data
    std::vector<pcl::PointIndices> rotational_segments;
    std::vector<pcl::PointIndices> bilateral_segments;
    std::vector< std::vector<int> > total_segments;
    cas.get(VIEW_ROTATIONAL_SEGMENTATION_IDS, rotational_segments);
    cas.get(VIEW_BILATERAL_SEGMENTATION_IDS, bilateral_segments);
    int numSegments = 0;

    for(size_t segmentId = 0; segmentId < rotational_segments.size(); segmentId++)
    {
      total_segments.push_back(rotational_segments[segmentId].indices);
    }

    for(size_t segmentId = 0; segmentId < bilateral_segments.size(); segmentId++)
    {
      total_segments.push_back(bilateral_segments[segmentId].indices);
    }

    for(size_t srcSegmentId = 0; srcSegmentId < total_segments.size(); srcSegmentId++)
    {
      for(size_t tgtSegmentId = 0; tgtSegmentId < ground_truth_ids.size(); tgtSegmentId++)
      {
        int intersectSize = Intersection(total_segments[srcSegmentId], ground_truth_ids[tgtSegmentId]).size();
        int unionSize = Union(total_segments[srcSegmentId], ground_truth_ids[tgtSegmentId]).size();
        float ratio = static_cast<float>(intersectSize) / unionSize;
        if(ratio > segment_similarity_threshold)
        {
          numSegments++;
        }
      }
    }

    float segmentation_ratio = static_cast<float>(numSegments) / ground_truth_ids.size();

    return segmentation_ratio;
  }
};


TEST_F(SegmentationTest, SymmetrySegmentationTest1)
{

  float seg_ratio = test("/samples/low_noise_dataset/clouds/cloud2.pcd",
                         "/samples/low_noise_dataset/ground_truth/ground_truth2.lbl");
  ASSERT_TRUE(seg_ratio > overall_segment_threshold);
}

TEST_F(SegmentationTest, SymmetrySegmentationTest2)
{

  float seg_ratio = test("/samples/low_noise_dataset/clouds/cloud3.pcd",
                         "/samples/low_noise_dataset/ground_truth/ground_truth3.lbl");

  ASSERT_TRUE(seg_ratio > overall_segment_threshold);
}
