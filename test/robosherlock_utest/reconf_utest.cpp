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

float testFloat;
std::vector<std::string *> testVec;	

void processReconfig()
{

  UnicodeString ustrInputText;
  cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
  
  //const uima::AnalysisEngineMetaData &aeMetaData = engine.getAnalysisEngineMetaData();
  //std::string aeDescription;
  //aeMetaData.getDescription().toUTF8String(aeDescription);
  std::vector<std::string> engineList = {"CollectionReader","NormalEstimator"};
  engine.getPipelineManager()->setPipelineOrdering(engineList);	


 /* uima::AnnotatorContext::TyMapDelegateAnCs delegates =  annotContext.getDelegates();

  engine.process(); 
  std::cerr<<"========================================"<<std::endl;
  for(auto d:delegates)
  {
      std::string out;
      d.first.toUTF8String(out);
      
      std::cerr<<( out )<<std::endl;
      
  }
  std::cerr<<"========================================"<<std::endl;
    
*/
  

  //Overwrite a float
  engine.overwriteParam("NormalEstimator","radiusSearch",3.5);
  //Overwrite a vector
  std::vector<std::string > overWriteVector;
  std::string overWriteValue = "config_mongodb_playback_utest.ini";
  //std::string * overWritePointer  = &overWriteValue;
  overWriteVector.push_back("mockupstring");
  engine.overwriteParam("CollectionReader","camera_config_files",&overWriteVector);
  engine.reconfigure();

  UnicodeString ucs_delegate("NormalEstimator");
  uima::AnnotatorContext &annotContext = engine.getAnnotatorContext();
  uima::AnnotatorContext *cr_context = annotContext.getDelegate(ucs_delegate);
  if(cr_context->isParameterDefined("radiusSearch"))
  {
    cr_context->extractValue("radiusSearch", testFloat);
  } 
  else outError("It doesn't");
  ucs_delegate = "CollectionReader";
  cr_context = annotContext.getDelegate(ucs_delegate);
  if(cr_context->isParameterDefined("camera_config_files"))
  {
    cr_context->extractValue("camera_config_files", testVec);
  } 
  else outError("It doesn't");
  
  
  
}

TEST(ReconfTest,CheckFloat)
{
  processReconfig();
  EXPECT_EQ(3.5,testFloat);
}
/*
TEST(ReconfTest,CheckString)
{
}
TEST(ReconfTest,CheckBool)
{
}*/

TEST(ReconfTest,CheckVec)
{ 
  outError(*(testVec.front()));
  //EXPECT_EQ("mockupstring",testVec.at(0).c_str());
}

