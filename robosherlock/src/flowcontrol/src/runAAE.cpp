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

//#include <uima/api.hpp>
//#include "uima/internal_aggregate_engine.hpp"
//#include <uima/annotator_mgr.hpp>

#include <mongo/client/dbclient.h>

#include <rs/flowcontrol/RSAggregateAnalysisEngine.h>
#include <rs/io/visualizer.h>

#include <robosherlock_msgs/RSObjectDescriptions.h>

#include <ros/ros.h>
#include <ros/package.h>


/**
 * @brief help description of parameters
 */
void help()
{
  std::cout << "Continuously exectue the fixed flow defined in an analysis engine." << std::endl
            << "Usage: rosrun robosherlock runAAE [options] [analysis_engine]" << std::endl
            << "Options:" << std::endl
            << "               _ae:=engine         Name of analysis enginee for execution" << std::endl
            << "              _vis:=true|false     shorter version for _visualization" << std::endl
            << std::endl;
}


void signalHandler(int signum)
{
  outWarn("Interrupt signal " << signum << " recevied. Exiting!");
  exit(signum);
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
  bool useVisualizer;

  nh.param("ae", analysis_engine_names, std::string(""));
  nh.param("vis", useVisualizer, false);

  nh.deleteParam("ae");
  nh.deleteParam("vis");

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
    outInfo(analysis_engine_file);


//  rs::Visualizer vis2(!useVisualizer);

  RSAggregateAnalysisEngine *engine;
  RSAggregateAnalysisEngine *engine2;
  ros::Publisher result_pub_ = nh.advertise<robosherlock_msgs::RSObjectDescriptions>(std::string("result_advertiser"), 1);

  try
  {
    //singl
    uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");

    mongo::client::GlobalInstance instance; //this is a stupid thing we did now we suffer the consequences 
    ros::AsyncSpinner spinner(0);

    engine = rs::createRSAggregateAnalysisEngine(analysis_engine_file, false);
    rs::Visualizer vis(!useVisualizer, engine->getAAEName(), false);

    spinner.start();
//    vis.addVisualizerManager("demo");
    vis.start();

    ros::Rate rate(30.0);
    while(ros::ok())
    {
      signal(SIGINT, signalHandler);

      engine->resetCas();
      engine->processOnce();

      std::vector<std::string> obj_descriptions;
      rs::ObjectDesignatorFactory dw(engine->getCas(),rs::ObjectDesignatorFactory::Mode::CLUSTER);
      dw.getObjectDesignators(obj_descriptions);
      robosherlock_msgs::RSObjectDescriptions objDescr;
      objDescr.obj_descriptions = obj_descriptions;
      result_pub_.publish(objDescr);

      rate.sleep(); 
    }

  }
  catch(const rs::Exception &e)
  {
    outError("Exception: [" << e.what() << "]");
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
  ros::shutdown();
  delete engine;
  return 0;
}
