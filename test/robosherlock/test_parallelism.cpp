#include <string>
#include <gtest/gtest.h>

#include <uima/api.hpp>

#include <rs/flowcontrol/RSAnalysisEngine.h>
#include <rs/flowcontrol/RSAggregateAnalysisEngine.h>
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>

#include <pcl/point_types.h>
#include <ros/ros.h>

#include <iostream>

class ParallelismTest : public testing::Test
{
    friend class RSAggregateAnalysisEngine;
protected:
    std::vector<std::string> engineList = {"CollectionReader",
                                           "ImagePreprocessor",
                                           "PointCloudFilter",
                                           "NormalEstimator",
                                           "PlaneAnnotator"};

    RSAggregateAnalysisEngine::AnnotatorOrderings orderings = {{"CollectionReader"},
                                                                {"ImagePreprocessor"},
                                                                {"PointCloudFilter"},
                                                                {"NormalEstimator", "PlaneAnnotator"}};
    RSAggregateAnalysisEngine::AnnotatorOrderingIndices orderingIndices = {{0}, {1}, {2}, {3, 4}};


    virtual void SetUp()
    {
      rs::common::getAEPaths("u_test",engineFile);
      engine.init(engineFile, false); // set false for not query from knowrob, we will manually set variables
      engine.initPipelineManager();

      engine.setPipelineOrdering(engineList);
      engine.setParallelOrderings(orderings,orderingIndices);
    }

    virtual void TearDown()
    {
       engine.stop();
    }

    std::string engineFile;
    RSAnalysisEngine engine;
};

TEST_F(ParallelismTest, ParallelExecutionTest)
{
  uima::TyErrorId error = engine.parallelProcess(*engine.getCas());

  EXPECT_TRUE(error == UIMA_ERR_NONE);
}
