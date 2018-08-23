#include <stdio.h>
#include <string.h>
#include <gtest/gtest.h>
#include <errno.h>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include <ros/ros.h>
#include <rs/flowcontrol/RSProcessManager.h>

#include <rs/utils/common.h>
#include <rs/io/Storage.h>

#include <rs/conversion/conversion.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <rs/scene_cas.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"

//Variables to be tested
  bool exist_cloud, exist_color_image, exist_color_image_HD, exist_depth_image, exist_depth_image_HD;
  bool exist_mask, exist_mask_HD, exist_thermal_image, exist_thermal_color_image;
  bool exist_thermal_depth_image,exist_thermal_fused_image, exist_thermal_cloud;

int preprocessingTest()
{

  UnicodeString ustrInputText;
  cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
  std::cerr<<"processing CAS"<<std::endl;

  //uima::AnnotatorContext &annotContext = engine.getAnnotatorContext();
  //uima::AnnotatorContext::TyMapDelegateAnCs delegates =  annotContext.getDelegates();


  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","NormalEstimator"};
  engine.setPipelineOrdering(engineList);

 try
    {
      //we process here
      //uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);
      engine.resetCas();
      engine.process();
      cas = engine.getCas();
      rs::SceneCas sceneCas(*cas);
//cloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_test(new pcl::PointCloud<pcl::PointXYZRGBA>());
      exist_cloud = sceneCas.get(VIEW_CLOUD, *cloud_test);
//color
      cv::Mat colorImg;
      exist_color_image = sceneCas.get(VIEW_COLOR_IMAGE, colorImg);
//colorHD
      cv::Mat colorImgHD;
      exist_color_image_HD = sceneCas.get(VIEW_COLOR_IMAGE_HD, colorImgHD);
//depth
      cv::Mat depthImg;
      exist_depth_image = sceneCas.get(VIEW_DEPTH_IMAGE, depthImg);
//depthHD
      cv::Mat depthImgHD;
      exist_depth_image_HD = sceneCas.get(VIEW_DEPTH_IMAGE_HD, depthImgHD);
//mask
      cv::Mat mask;
      exist_mask = sceneCas.get(VIEW_MASK, mask);
//maskHD
      cv::Mat maskHD;
      exist_mask_HD = sceneCas.get(VIEW_MASK_HD, maskHD);
//thermalImg
      cv::Mat thermalImg;
      exist_thermal_image = sceneCas.get(VIEW_THERMAL_IMAGE, thermalImg);
//thermalDepthImg
      cv::Mat thermalDepthImg;
      exist_thermal_depth_image = sceneCas.get(VIEW_THERMAL_DEPTH_IMAGE, thermalDepthImg);
//thermalColorImg
      cv::Mat thermalColorImg;
      exist_thermal_color_image = sceneCas.get(VIEW_THERMAL_COLOR_IMAGE, thermalColorImg);
//thermalFusedImg
      cv::Mat thermalFusedImg;
      exist_thermal_fused_image = sceneCas.get(VIEW_THERMAL_FUSED, thermalFusedImg);
//thermalCloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr thermal_cloud_test(new pcl::PointCloud<pcl::PointXYZRGBA>());
      exist_thermal_cloud = sceneCas.get(VIEW_THERMAL_CLOUD, *thermal_cloud_test);

      engine.getAnnotatorContext().releaseCAS(*cas);

    }
    catch(const rs::FrameFilterException &)
    {
      outError("There is an error in the rs");
    }
  //UnicodeString ucs_delegate("CollectionReader");
  //uima::AnnotatorContext *cr_context =  annotContext.extractDelegate(ucs_delegate);

 /* if(cr_context->isParameterDefined("camera_config_files"))
  {
    std::vector<std::string *> values;
    cr_context->extractValue("camera_config_files", values);
    for (auto v:values)
    {
       std::cerr<<*v<<std::endl;
	
    }
  }
  
  std::vector<UnicodeString> new_configs;
  new_configs.push_back(UnicodeString("config_mongodb_playback_utest.ini"));
   
  cr_context->assignValue(UnicodeString("camera_config_files"),new_configs); */
  engine.reconfigure();

  engine.collectionProcessComplete();

}

TEST(UnitTest,CheckExistingCloud)
{
   preprocessingTest();
   EXPECT_EQ(1,exist_cloud);

}
TEST(UnitTest, CheckExistingColorImage)
{
   EXPECT_EQ(1,exist_color_image);
}
TEST(UnitTest, CheckExistingColorImageHD)
{
   EXPECT_EQ(1, exist_color_image_HD);
}
TEST(UnitTest, CheckExistingDepthImage)
{
   EXPECT_EQ(1, exist_depth_image);
}
TEST(UnitTest, CheckExistingDepthImageHD)
{
   EXPECT_EQ(1, exist_depth_image_HD);
}
TEST(UnitTest, CheckExistingMask)
{
   EXPECT_EQ(1, exist_mask);
}
TEST(UnitTest, CheckExistingMaskHD)
{
   EXPECT_EQ(1, exist_mask_HD);
}
// TEST(UnitTest, CheckExistingThermalImage)
// {
//    EXPECT_EQ(1, exist_thermal_image);
// }
// TEST(UnitTest, CheckExistingThermalColorImage)
// {
//    EXPECT_EQ(1, exist_thermal_color_image);
// }
// TEST(UnitTest, CheckExistingThermalDepthImage)
// {
//    EXPECT_EQ(1, exist_thermal_depth_image);
// }
// TEST(UnitTest, CheckExistingThermalFused)
// {
//    EXPECT_EQ(1, exist_thermal_fused_image);
// }
// TEST(UnitTest, CheckExistingThermalCloud)
// {
//    EXPECT_EQ(1, exist_thermal_cloud);
// }
