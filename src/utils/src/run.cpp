/*------------------------------------------------------------------------
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 --------------------------------------------------------------------------
 Test driver that reads text files or XCASs or XMIs and calls the annotator
 -------------------------------------------------------------------------- */

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

#include <uima/api.hpp>

#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/exception.h>
#include <rs/io/visualizer.h>

#include <ros/ros.h>
#include <ros/package.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG

#define SEARCHPATH "/descriptors/analysis_engines/"

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

class RSAnalysisEngine
{
private:
  std::string name;

  uima::AnalysisEngine *engine;
  uima::CAS *cas;

public:
  RSAnalysisEngine() : engine(NULL), cas(NULL)
  {
  }

  ~RSAnalysisEngine()
  {
    if(cas)
    {
      delete cas;
      cas = NULL;
    }
    if(engine)
    {
      delete engine;
      engine = NULL;
    }
  }

  void init(const std::string &file)
  {
    uima::ErrorInfo errorInfo;

    size_t pos = file.rfind('/');
    outInfo("Creating analysis engine: " FG_BLUE << (pos == file.npos ? file : file.substr(pos)));

    engine = uima::Framework::createAnalysisEngine(file.c_str(), errorInfo);

    if(errorInfo.getErrorId() != UIMA_ERR_NONE)
    {
      outError("createAnalysisEngine failed.");
      throw uima::Exception(errorInfo);
    }
    const uima::AnalysisEngineMetaData &data = engine->getAnalysisEngineMetaData();
    data.getName().toUTF8String(name);

    // Get a new CAS
    outInfo("Creating a new CAS");
    cas = engine->newCAS();

    if(cas == NULL)
    {
      outError("Creating new CAS failed.");
      engine->destroy();
      delete engine;
      engine = NULL;
      throw uima::Exception(uima::ErrorMessage(UIMA_ERR_ENGINE_NO_CAS), UIMA_ERR_ENGINE_NO_CAS, uima::ErrorInfo::unrecoverable);
    }

    outInfo("initialization done: " << name << std::endl
            << std::endl << FG_YELLOW << "********************************************************************************" << std::endl);
  }

  void stop()
  {
    engine->collectionProcessComplete();
    engine->destroy();

    outInfo("Analysis engine stopped: " << name);
  }

  void process()
  {
    outInfo("executing analisys engine: " << name);
    try
    {
      UnicodeString ustrInputText;
      ustrInputText.fromUTF8(name);
      cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));

      rs::StopWatch clock;
      outInfo("processing CAS");
      uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);

      for(int i = 0; casIter.hasNext(); ++i)
      {
        uima::CAS &outCas = casIter.next();

        // release CAS
        outInfo("release CAS " << i);
        engine->getAnnotatorContext().releaseCAS(outCas);
      }

      outInfo("processing finished");
      outInfo(clock.getTime() << " ms." << std::endl << std::endl << FG_YELLOW
              << "********************************************************************************" << std::endl);
    }
    catch(const rs::Exception &e)
    {
      outError("Exception: " << std::endl << e.what());
    }
    catch(const uima::Exception &e)
    {
      outError("Exception: " << std::endl << e);
    }
    catch(const std::exception &e)
    {
      outError("Exception: " << std::endl << e.what());
    }
    catch(...)
    {
      outError("Unknown exception!");
    }
    cas->reset();
  }
};

class RSAnalysisEngineManager
{
private:
  std::vector<RSAnalysisEngine> engines;

  const bool useVisualizer;
  rs::Visualizer visualizer;

public:
  RSAnalysisEngineManager(const bool useVisualizer, const std::string &savePath) : useVisualizer(useVisualizer), visualizer(savePath)
  {
    // Create/link up to a UIMACPP resource manager instance (singleton)
    outInfo("Creating resource manager"); // TODO: DEBUG
    uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock"); // TODO: change topic?

    switch(OUT_LEVEL)
    {
    case OUT_LEVEL_NOOUT:
    case OUT_LEVEL_ERROR:
      resourceManager.setLoggingLevel(uima::LogStream::EnError);
      break;
    case OUT_LEVEL_INFO:
      resourceManager.setLoggingLevel(uima::LogStream::EnWarning);
      break;
    case OUT_LEVEL_DEBUG:
      resourceManager.setLoggingLevel(uima::LogStream::EnMessage);
      break;
    }
  }

  ~RSAnalysisEngineManager()
  {
    uima::ResourceManager::deleteInstance();
  }

  void init(const std::vector<std::string> &files)
  {
    engines.resize(files.size());
    for(size_t i = 0; i < engines.size(); ++i)
    {
      engines[i].init(files[i]);
    }
    if(useVisualizer)
    {
      visualizer.start();
    }
  }

  void run()
  {
    for(; ros::ok();)
    {
      for(size_t i = 0; i < engines.size(); ++i)
      {
        engines[i].process();
      }
    }
  }

  void stop()
  {
    if(useVisualizer)
    {
      visualizer.stop();
    }
    for(size_t i = 0; i < engines.size(); ++i)
    {
      engines[i].stop();
    }
  }
};

/* ----------------------------------------------------------------------- */
/*       Main                                                              */
/* ----------------------------------------------------------------------- */

/**
 * Error output if program is called with wrong parameter.
 */
void help()
{
  std::cout << "Usage: runAECppLoop [options] analysisEngine.xml [...]" << std::endl
            << "Options:" << std::endl
            << "  -visualizer  Enable visualization" << std::endl
            << "  -save PATH   Path for storing images" << std::endl;
}

int main(int argc, char *argv[])
{
  /* Access the command line arguments to get the name of the input text. */
  if(argc < 2)
  {
    help();
    return 1;
  }

  ros::init(argc,argv, std::string("RoboSherlock_") + getenv("USER"));

  std::vector<std::string> args;
  args.resize(argc - 1);
  for(int argI = 1; argI < argc; ++argI)
  {
    args[argI - 1] = argv[argI];
  }

  bool useVisualizer = false;
  std::string savePath = getenv("HOME");

  size_t argO = 0;
  for(size_t argI = 0; argI < args.size(); ++argI)
  {
    const std::string &arg = args[argI];

    if(arg == "-visualizer")
    {
      useVisualizer = true;
    }
    else if(arg == "-save")
    {
      if(++argI < args.size())
      {
        savePath = args[argI];
      }
      else
      {
        outError("No save path defined!");
        return -1;
      }
    }
    else
    {
      args[argO] = args[argI];
      ++argO;
    }
  }
  args.resize(argO);

  struct stat fileStat;
  if(stat(savePath.c_str(), &fileStat) || !S_ISDIR(fileStat.st_mode))
  {
    outError("Save path \"" << savePath << "\" does not exist.");
    return -1;
  }

  std::vector<std::string> analysisEngineFiles;

  //generate a vector of possible paths for the analysis engine
  std::vector<std::string> searchPaths;

  //empty path for full path given as argument
  searchPaths.push_back("");
  //add core package path
  searchPaths.push_back(ros::package::getPath("robosherlock") + std::string(SEARCHPATH));

  //look for packages dependent on core and find their full path
  std::vector<std::string> child_packages;
  ros::package::command("depends-on robosherlock", child_packages);
  for(int i = 0; i < child_packages.size(); ++i)
  {
    searchPaths.push_back(ros::package::getPath(child_packages[i]) + std::string(SEARCHPATH));
  }

  analysisEngineFiles.resize(args.size(), "");
  for(int argI = 0; argI < args.size(); ++argI)
  {
    const std::string &arg = args[argI];
    struct stat fileStat;

    for(size_t i = 0; i < searchPaths.size(); ++i)
    {
      const std::string file = searchPaths[i] + arg;
      const std::string fileXML = file + ".xml";

      if(!stat(file.c_str(), &fileStat) && S_ISREG(fileStat.st_mode))
      {
        analysisEngineFiles[argI] = file;
        break;
      }
      else if(!stat(fileXML.c_str(), &fileStat) && S_ISREG(fileStat.st_mode))
      {
        analysisEngineFiles[argI] = fileXML;
        break;
      }
    }

    if(analysisEngineFiles[argI].empty())
    {
      outError("analysis engine \"" << arg << "\" not found.");
      return -1;
    }
  }

  try
  {
    RSAnalysisEngineManager manager(useVisualizer, savePath);

    manager.init(analysisEngineFiles);

    manager.run();

    manager.stop();
  }
  catch(const rs::Exception &e)
  {
    outError("Exception: " << std::endl << e.what());
    return -1;
  }
  catch(const uima::Exception &e)
  {
    outError("Exception: " << std::endl << e);
    return -1;
  }
  catch(const std::exception &e)
  {
    outError("Exception: " << std::endl << e.what());
    return -1;
  }
  catch(...)
  {
    outError("Unknown exception!");
    return -1;
  }
  return 0;
}
