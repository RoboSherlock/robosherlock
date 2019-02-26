#include <string>
#include <sys/stat.h>
#include <stdio.h>
#include <gtest/gtest.h>

#include <rs/flowcontrol/RSAnalysisEngine.h>
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
  UnicodeString ustrInputText;

  cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
  std::cerr<<"processing CAS"<<std::endl;
  
  std::vector<std::string> engineList = {"CollectionReader","StorageWriter"};
  engine.setPipelineOrdering(engineList);

 try
    {
      engine.process();
      cas = engine.getCas();	
      rs::SceneCas sceneCas(*cas);
      cv::Mat colorImg;
      sceneCas.get(VIEW_COLOR_IMAGE_HD,colorImg);
      initial_image_width = colorImg.size().width;
      initial_image_height = colorImg.size().height;
      engine.getAnnotatorContext().releaseCAS(*cas);
    }
    catch(const rs::FrameFilterException &)
    {
      outError("There is an error in the rs");
    }

  engine.overwriteParam("CollectionReader","camera_config_files","config_mongodb_playback_utest.ini");
  engine.reconfigure();
  
  try
  {
  engine.process();
  cas = engine.getCas();
  rs::SceneCas sceneCas(*cas);
  cv::Mat colorImg;
  sceneCas.get(VIEW_COLOR_IMAGE_HD,colorImg);
  db_image_width = colorImg.size().width;
  db_image_height = colorImg.size().height;
  engine.getAnnotatorContext().releaseCAS(*cas);
  }
  catch(const rs::FrameFilterException &){}
  engine.collectionProcessComplete();
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

