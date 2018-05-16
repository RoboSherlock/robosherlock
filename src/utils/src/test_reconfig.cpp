/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include <ros/ros.h>

#include <rs/flowcontrol/RSAnalysisEngineManager.h>
#include <rs/utils/common.h>

#include <ros/ros.h>
#include <ros/package.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG

/**
 * Error output if program is called with wrong parameter.
 */
void help()
{
  std::cout << "Usage: rosrun robosherlock run [options] [analysis_engines]" << std::endl
            << "Options:" << std::endl
            << " _analysis_engines:=engine1[,...]  List of analysis engines for execution" << std::endl
            << "               _ae:=engine1[,...]  shorter version for _analysis_engines" << std::endl
            << std::endl
            << "Usage: roslaunch robosherlock rs.launch [options]" << std::endl
            << "Options:" << std::endl
            << "  analysis_engines:=engine1[,...]  List of analysis engines for execution" << std::endl
            << "                ae:=engine1[,...]  shorter version for analysis_engines" << std::endl;
}

/* ----------------------------------------------------------------------- */
/*       Main                                                              */
/* ----------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
  char *userEnv = getenv("USER");
  if(userEnv!=NULL)
  {
    ros::init(argc, argv, std::string("RoboSherlock_") +std::string(userEnv));
  }
  else
  {
    ros::init(argc, argv, std::string("RoboSherlock"));
  }

  std::string analysisEnginesArg, savePath;
  std::vector<std::string> analysisEngines;

  ros::NodeHandle priv_nh = ros::NodeHandle("~");

  for(int argI = 1; argI < argc; ++argI)
  {
    const std::string arg = argv[argI];
    if(arg == "-?" || arg == "-h" || arg == "--help")
    {
      help();
      return 0;
    }
  }

  priv_nh.param("ae", analysisEnginesArg, std::string("demo"));
  priv_nh.param("analysis_engines", analysisEnginesArg, analysisEnginesArg);

  analysisEngines.push_back(analysisEnginesArg);

  std::vector<std::string> analysisEngineFiles;
  std::ostringstream engineList;
  for(int i = 0; i < analysisEngines.size(); ++i)
  {
    const std::string &engine = analysisEngines[i];
    std::string engineFile;
    rs::common::getAEPaths(engine,engineFile);
    if(engineFile.empty())
    {
      outError("analysis engine \"" << engine << "\" not found.");
      return -1;
    }
    else
    {
      analysisEngineFiles.push_back(engineFile);
    }
    engineList << FG_CYAN << engine << (i + 1 < analysisEngines.size() ? NO_COLOR ", " : NO_COLOR);
  }

  if(analysisEngineFiles.empty())
  {
    outError("no analysis engine specified.");
    help();
    return -1;
  }
  std::string engine_file = analysisEngineFiles[0];
  outInfo("startup parameters:" << std::endl
          << "analysis_engines: " << engineList.str());
  
  uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
  resourceManager.setLoggingLevel(uima::LogStream::EnError);
  uima::ErrorInfo errorInfo;
  uima::AnalysisEngine *engine = uima::Framework::createAnalysisEngine(engine_file.c_str(), errorInfo);
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

  //prints all annotatores imported in the xml (not just in the fixed flow)
  std::cerr<<"========================================"<<std::endl;
  for(auto d:delegates)
  {
      std::string out;
      d.first.toUTF8String(out);
      std::cerr<<out<<std::endl;
  }
  std::cerr<<"========================================"<<std::endl;

  //we process here
  uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);
  for(int i = 0; casIter.hasNext(); ++i)
  {
   uima::CAS &outCas = casIter.next();
   std::cerr<<"release CAS " << i<<std::endl;
   engine->getAnnotatorContext().releaseCAS(outCas);
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
  new_configs.push_back(UnicodeString("config_kinect_robot.ini"));
   
  cr_context->assignValue(UnicodeString("camera_config_files"),new_configs);
  engine->reconfigure();

  casIter = engine->processAndOutputNewCASes(*cas);
  for(int i = 0; casIter.hasNext(); ++i)
  {
   uima::CAS &outCas = casIter.next();
   std::cerr<<"release CAS " << i<<std::endl;
   engine->getAnnotatorContext().releaseCAS(outCas);
  }
  engine->collectionProcessComplete();
  engine->destroy();

  return 0;
}
