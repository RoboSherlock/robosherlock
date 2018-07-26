#include <string>
#include <sys/stat.h>
#include <stdio.h>
#include <gtest/gtest.h>
#include "rs/flowcontrol/RSAnalysisEngineManager.h"
#include "rs/flowcontrol/RSPipelineManager.h"

#include "rs/flowcontrol/RSAnalysisEngine.h"
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>

#include <pcl/point_types.h>
#include <ros/ros.h>



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

#include <ros/ros.h>
#include <ros/package.h>
#include <rs/scene_cas.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
#include "../main.h"

//Variables to be tested
int initial_image_width = 0;
int initial_image_height = 0;
int db_image_width = 0;
int db_image_height = 0;

int processEngine()
{
  const uima::AnalysisEngineMetaData &data = engine->getAnalysisEngineMetaData();
  std::string name;

  data.getName().toUTF8String(name);
  std::cerr<<"Enigne: " << name<<std::endl;
  uima::CAS *cas;

  cas = engine->newCAS();

  if(cas == NULL)
  {
    std::cerr<<"Creating new CAS failed."<<std::endl;
    engine->destroy();
    delete engine;
    return 0;
  }

  UnicodeString ustrInputText;
  ustrInputText.fromUTF8(name);
  cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
  std::cerr<<"processing CAS"<<std::endl;
 
  const uima::AnalysisEngineMetaData &aeMetaData = engine->getAnalysisEngineMetaData();
  std::string aeDescription;
  aeMetaData.getDescription().toUTF8String(aeDescription);
  uima::AnnotatorContext &annotContext = engine->getAnnotatorContext();
  uima::AnnotatorContext::TyMapDelegateAnCs delegates =  annotContext.getDelegates();
  	

 try
    {
      //we process here
      uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);
      rs::SceneCas sceneCas(*cas);
      cv::Mat colorImg;
      sceneCas.get(VIEW_COLOR_IMAGE_HD,colorImg);
      initial_image_width = colorImg.size().width;
      initial_image_height = colorImg.size().height;
      engine->getAnnotatorContext().releaseCAS(*cas);
    }
    catch(const rs::FrameFilterException &)
    {
      outError("There is an error in the rs");
    }
  UnicodeString ucs_delegate("CollectionReader");
  uima::AnnotatorContext *cr_context =  annotContext.extractDelegate(ucs_delegate);

  if(cr_context->isParameterDefined("camera_config_files"))
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
   
  cr_context->assignValue(UnicodeString("camera_config_files"),new_configs);
  engine->reconfigure();
  
  try
  {
  uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);
  rs::SceneCas sceneCas(*cas);
  cv::Mat colorImg;
  sceneCas.get(VIEW_COLOR_IMAGE_HD,colorImg);
  db_image_width = colorImg.size().width;
  db_image_height = colorImg.size().height;
  engine->getAnnotatorContext().releaseCAS(*cas);
  }
  catch(const rs::FrameFilterException &){}
  engine->collectionProcessComplete();
  engine->destroy();
}


TEST(UnitTest,CheckImageWidth)
{
   processEngine();
   EXPECT_EQ(initial_image_width,db_image_width);

}

TEST(UnitTest,CheckImageHeight)
{
   EXPECT_EQ(initial_image_height,db_image_height);

}

