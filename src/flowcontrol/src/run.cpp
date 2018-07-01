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

#include <rs/flowcontrol/RSPipelineManager.h>
//#include <rs/queryanswering/KRDefinitions.h>
#include <rs/flowcontrol/RSProcessManager.h>

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
              << "    _visualization:=true|false     Enable/disable visualization" << std::endl
              << "              _vis:=true|false     shorter version for _visualization" << std::endl
              << "        _save_path:=PATH           Path to where images and point clouds should be stored" << std::endl
              << "             _wait:=true|false     Enable/Disable waiting for a query before the execution starts"<< std::endl
              << "        _pervasive:=true|false     Enable/Disable running the pipeline defined in the analysis engine xml"<< std::endl
              << "        _parallel:=true|false      Enable/Disable parallel execution of pipeline (json_prolog is required)"<< std::endl
              << "        _withIDRes:=true|false     Enable/Disable running object identity resolution"<< std::endl
              << std::endl
              << "Usage: roslaunch robosherlock rs.launch [options]" << std::endl
              << "Options:" << std::endl
              << "  analysis_engines:=engine1[,...]  List of analysis engines for execution" << std::endl
              << "                ae:=engine1[,...]  shorter version for analysis_engines" << std::endl
              << "     visualization:=true|false     Enable/disable visualization" << std::endl
              << "               vis:=true|false     shorter version for visualization" << std::endl
              << "         save_path:=PATH           Path to where images and point clouds should be stored" << std::endl
              << "             _wait:=true|false     Enable/Disable waiting for a query before the execution starts"<< std::endl
              << "        _pervasive:=true|false     Enable/Disable running the pipeline defined in the analysis engine xml"<< std::endl
              << "        _parallel:=true|false      Enable/Disable parallel execution of pipeline (json_prolog is required)"<< std::endl
              << "         withIDRes:=true|false     Enable/Disable running object identity resolution"<< std::endl;

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

  ros::init(argc, argv, std::string("RoboSherlock_") + getenv("USER"));
  ros::NodeHandle nh("~");

  std::string analysisEnginesName, analysisEngineFile, savePath;
  bool useVisualizer, waitForServiceCall, useObjIDRes, pervasive, parallel;

  nh.param("ae", analysisEnginesName, std::string(""));
  nh.param("analysis_engines", analysisEnginesName, analysisEnginesName);

  nh.param("wait",waitForServiceCall, false);
  nh.param("vis", useVisualizer, false);
  nh.param("visualization", useVisualizer, useVisualizer);

  nh.param("save_path", savePath, std::string(getenv("HOME")));
  nh.param("pervasive", pervasive, false);
  nh.param("parallel", parallel, true);
  nh.param("withIDRes", useObjIDRes,false);

  nh.deleteParam("ae");
  nh.deleteParam("analysis_engines");
  nh.deleteParam("vis");
  nh.deleteParam("visualization");
  nh.deleteParam("save_path");
  nh.deleteParam("wait");
  nh.deleteParam("pervasive");
  nh.deleteParam("parallel");

  //if only argument is an AE (nh.param reudces argc)
  if(argc == 2)
  {
    const std::string arg = argv[1];
    if(arg == "-?" || arg == "-h" || arg == "--help")
    {
      help();
      return 0;
    }
    analysisEnginesName = argv[1];
  }
  rs::common::getAEPaths(analysisEnginesName, analysisEngineFile);

  if(analysisEngineFile.empty())
  {
    outError("analysis   engine \"" << analysisEngineFile << "\" not found.");
    return -1;
  }
  else
  {
    outInfo(analysisEngineFile);
  }

  std::string configFile = ros::package::getPath("robosherlock") + "/config/config.yaml";

  try
  {
    RSProcessManager manager(useVisualizer, waitForServiceCall, nh, savePath);
    manager.setUseIdentityResolution(useObjIDRes);
    manager.pause();
    manager.init(analysisEngineFile, configFile, pervasive, parallel);
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
