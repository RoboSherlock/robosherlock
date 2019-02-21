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
#include <chrono>
#include <condition_variable>
#include <sstream>

#include <uima/api.hpp>
#include "uima/internal_aggregate_engine.hpp"
#include "uima/annotator_mgr.hpp"

#include <rs/flowcontrol/RSProcessManager.h>

#include <ros/ros.h>
#include <ros/package.h>


/**
 * @brief help description of parameters
 */
void help()
{
  std::cout << "Usage: rosrun robosherlock run [options] [analysis_engines]" << std::endl
            << "Options:" << std::endl
            << "               _ae:=engine         Name of analysis enginee for execution" << std::endl
            << "              _vis:=true|false     shorter version for _visualization" << std::endl
            << "        _save_path:=PATH           Path to where images and point clouds should be stored" << std::endl
            << "             _wait:=true|false     Enable/Disable waiting for a query before the execution starts" << std::endl
            << "        _pervasive:=true|false     Enable/Disable running the pipeline defined in the analysis engine xml" << std::endl
            << "         _parallel:=true|false     Enable/Disable parallel execution of pipeline (json_prolog is required)" << std::endl
            << "        _withIDRes:=true|false     Enable/Disable running object identity resolution" << std::endl
            << "               _ke:=ke_type        Set the knowledge engine you want to use; Values are: [SWI_PROLOG, KNOWROB]. Default is SWI_PROLOG." << std::endl
            << std::endl;
}


/* ----------------------------------------------------------------------- */
/*       Main                                                              */
/* ----------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    help();
    return 1;
  }

  if(OUT_LEVEL == OUT_LEVEL_DEBUG)
  {
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  ros::init(argc, argv, std::string("RoboSherlock_") + getenv("USER"));
  ros::NodeHandle nh("~");

  std::string analysis_engine_names, analysis_engine_file, save_path, knowledge_engine;
  bool useVisualizer, waitForServiceCall, useObjIDRes, pervasive, parallel;

  nh.param("ae", analysis_engine_names, std::string(""));
  nh.param("analysis_engines", analysis_engine_names, analysis_engine_names);

  nh.param("wait", waitForServiceCall, false);
  nh.param("vis", useVisualizer, false);
  nh.param("visualization", useVisualizer, useVisualizer);

  nh.param("save_path", save_path, std::string(getenv("HOME")));
  nh.param("pervasive", pervasive, false);
  nh.param("parallel", parallel, false);
  nh.param("withIDRes", useObjIDRes, false);
  nh.param("ke", knowledge_engine, std::string("SWI_PROLOG"));

  nh.deleteParam("ae");
  nh.deleteParam("vis");
  nh.deleteParam("save_path");
  nh.deleteParam("wait");
  nh.deleteParam("pervasive");
  nh.deleteParam("parallel");
  nh.deleteParam("ke");


  //if only argument is an AE (nh.param reudces argc)
  if(argc == 2)
  {
    const std::string arg = argv[1];
    if(arg == "-?" || arg == "-h" || arg == "--help")
    {
      help();
      return 0;
    }
    analysis_engine_names = argv[1];
  }
  rs::common::getAEPaths(analysis_engine_names, analysis_engine_file);

  if(analysis_engine_file.empty())
  {
    outError("analysis engine \"" << analysis_engine_file << "\" not found.");
    return -1;
  }
  else
  {
    outInfo(analysis_engine_file);
  }

  std::string configFile = ros::package::getPath("robosherlock") + "/config/config.yaml";

  RSProcessManager::KnowledgeEngineType keType;
  if(knowledge_engine == "SWI_PROLOG")
  {
    keType = RSProcessManager::KnowledgeEngineType::SWI_PROLOG;
  }
  else if(knowledge_engine == "KNOWROB")
  {
    keType = RSProcessManager::KnowledgeEngineType::JSON_PROLOG;
  }
  else
  {
    outError("Unsupported Knowledge Engine type! Valid values are: [SWI_PROLOG, JSON_PROLOG]. Exiting.");
    return 0;
  }

  try
  {
    RSProcessManager manager(useVisualizer, waitForServiceCall, keType, nh, save_path);
    manager.setUseIdentityResolution(useObjIDRes);
    manager.init(analysis_engine_file, configFile, pervasive, parallel);
    manager.run();
    manager.stop();
  }
  catch(const rs::Exception &e)
  {
    outError("Exception: [" << e.what()<<"]");
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
