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
#include <rs/flowcontrol/RSAnalysisEngineManager.h>
#include <rs/utils/common.h>
#include <rs/io/Storage.h>
#include </home/mirrorice/catkin_ws2/src/robosherlock/src/core/include/rs/conversion/conversion.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <rs/scene_cas.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"

//Variables to be tested
  bool exist_cloud;

int preprocessingTest()
{

  uima::CAS *cas;
  cas = engine.newCAS();

  UnicodeString ustrInputText;
  cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
  std::cerr<<"processing CAS"<<std::endl;

  //uima::AnnotatorContext &annotContext = engine.getAnnotatorContext();
  //uima::AnnotatorContext::TyMapDelegateAnCs delegates =  annotContext.getDelegates();

  //std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor"};
  //engine.getPipelineManager()->setPipelineOrdering(engineList);

 try
    {
      //we process here
      //uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);
      engine.process();
      cas = engine.getCas();
      rs::SceneCas sceneCas(*cas);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_test(new pcl::PointCloud<pcl::PointXYZRGBA>());
      exist_cloud = sceneCas.get(VIEW_CLOUD, *cloud_test);
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
