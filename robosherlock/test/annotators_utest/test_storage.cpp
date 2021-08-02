#include <string>
#include <sys/stat.h>
#include <stdio.h>
#include <gtest/gtest.h>

#include <robosherlock/utils/common.h>
#include <robosherlock/types/all_types.h>
#include <robosherlock/scene_cas.h>

#include <pcl/point_types.h>
#include <ros/ros.h>



#include <string.h>
#include <ros/package.h>
#include <robosherlock/flowcontrol/RSAggregateAnalysisEngine.h>

#include "../main.h"

//Variables to be tested
int initial_image_width = 0;
int initial_image_height = 0;
int db_image_width = 0;
int db_image_height = 0;

int processEngine()
{
  icu::UnicodeString ustrInputText;

  cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
  std::cerr<<"processing CAS"<<std::endl;
  
  std::vector<std::string> engineList = {"CollectionReader","ImagePreprocessor","StorageWriter"};
  engine->setPipelineOrdering(engineList);

 try
    {
      engine->resetCas();
      engine->processOnce();
      cas = engine->getCas();
      rs::SceneCas sceneCas(*cas);
      cv::Mat colorImg;
      sceneCas.get(VIEW_COLOR_IMAGE_HD,colorImg);
      initial_image_width = colorImg.size().width;
      initial_image_height = colorImg.size().height;
      
      outInfo("Initial image_width = "<<initial_image_width);
      engine->getAnnotatorContext().releaseCAS(*cas);
    }
    catch(const rs::FrameFilterException &)
    {
      outError("There is an error in the rs");
    }

  engine->overwriteParam("CollectionReader","camera_config_files",std::vector<std::string>{"config_mongodb_playback_utest.ini"});
  engine->reconfigure();
  engine->resetCas(); 
  try
  {
    engine->processOnce();
    cas = engine->getCas();
    rs::SceneCas sceneCas(*cas);
    cv::Mat colorImg;
    sceneCas.get(VIEW_COLOR_IMAGE_HD,colorImg);
    db_image_width = colorImg.size().width;
    db_image_height = colorImg.size().height;
    outInfo("DB image_width = "<<db_image_width);
    engine->getAnnotatorContext().releaseCAS(*cas);
  }
  catch(const rs::FrameFilterException &){}
  engine->overwriteParam("CollectionReader","camera_config_files",std::vector<std::string>{"config_data_loader_utest.ini"});
  engine->reconfigure();
  engine->resetCas();
  engine->collectionProcessComplete();

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

