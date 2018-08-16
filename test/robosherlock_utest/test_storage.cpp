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

#include <gtest/gtest.h>
#include <rs/scene_cas.h>
#include <rs/flowcontrol/RSAnalysisEngine.h>
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
  //const uima::AnalysisEngineMetaData &data = engine->getAnalysisEngineMetaData();
  //std::string name;

  //data.getName().toUTF8String(name);
  //std::cerr<<"Enigne: " << name<<std::endl;


  /*if(cas == NULL)
  {
    std::cerr<<"Creating new CAS failed."<<std::endl;
    engine->destroy();
    delete engine;
    return 0;
  }*/
  UnicodeString ustrInputText;
  //ustrInputText.fromUTF8(name);
  cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
  std::cerr<<"processing CAS"<<std::endl;
 
  //const uima::AnalysisEngineMetaData &aeMetaData = engine->getAnalysisEngineMetaData();
  //std::string aeDescription;
  //aeMetaData.getDescription().toUTF8String(aeDescription);
  
  std::vector<std::string> engineList = {"CollectionReader","StorageWriter"};
  engine.getPipelineManager()->setPipelineOrdering(engineList);	

 try
    {
      //we process here
      //uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);
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

