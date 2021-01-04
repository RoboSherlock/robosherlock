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

#include <robosherlock/flowcontrol/RSAggregateAnalysisEngine.h>
#include <robosherlock/io/visualizer.h>
#include <robosherlock/CASConsumerContext.h>
#include <robosherlock/scene_cas.h>


#include <robosherlock_msgs/RSObjectDescriptions.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

/**
 * @brief help description of parameters
 */
void help()
{
  std::cout << "Continuously exectue the fixed flow defined in an analysis engine." << std::endl
            << "Usage: rosrun robosherlock runAAE [options] [analysis_engine]" << std::endl
            << "Options:" << std::endl
            << "               _ae:=engine         Name of analysis engines for execution. Comma seperated list." << std::endl
            << "              _vis:=true|false     shorter version for _visualization." << std::endl
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

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    help();
    return 1;
  }

  ros::init(argc, argv, std::string("RoboSherlock_") + getenv("USER"));

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
     ros::console::notifyLoggerLevelsChanged();
  ros::NodeHandle nh("~");

  std::string analysis_engine_names, analysis_engine_file, save_path, knowledge_engine;
  bool useVisualizer;
  bool publishResults;

  nh.param("ae", analysis_engine_names, std::string(""));
  nh.param("vis", useVisualizer, false);
  nh.param("publish_results", publishResults, false);

  nh.deleteParam("ae");
  nh.deleteParam("vis");

  std::vector<std::string> analysis_engine_name_list;


  // if only argument is an AE (nh.param reudces argc)
  if (argc == 2)
  {
    const std::string arg = argv[1];
    if (arg == "-?" || arg == "-h" || arg == "--help")
    {
      help();
      return 0;
    }
    analysis_engine_names = argv[1];
  }

  std::stringstream ss(analysis_engine_names);
  while (ss.good())
  {
    std::string substring;
    std::getline(ss, substring, ',');
    if (substring != "")
    {
      analysis_engine_name_list.push_back(substring);
    }
  }

  std::vector<std::string> analysis_engine_file_paths;
  for (auto ae_name : analysis_engine_name_list)
  {
    rs::common::getAEPaths(ae_name, analysis_engine_file);

    if (analysis_engine_file.empty())
    {
      outError("analysis engine \"" << ae_name << "\" not found.");
      return -1;
    }
    else
    {
      outInfo(analysis_engine_file);
      analysis_engine_file_paths.push_back(analysis_engine_file);
    }
  }

  ros::Publisher result_pub;
  if (publishResults)
  {
    result_pub = nh.advertise<robosherlock_msgs::RSObjectDescriptions>(std::string("result_advertiser"), 1);
  }

  std::vector<RSAggregateAnalysisEngine*> loaded_rsaaes;
  std::vector<RSAggregateAnalysisEngine*> running_rsaaes;
  try
  {
    // singl
    uima::ResourceManager& resourceManager = uima::ResourceManager::createInstance("RoboSherlock");

    mongo::client::GlobalInstance instance;  // this is a stupid thing we did now we suffer the consequences
    ros::AsyncSpinner spinner(0);

    rs::Visualizer vis(!useVisualizer, true);
    spinner.start();
    for (auto ae_path : analysis_engine_file_paths)
    {
      RSAggregateAnalysisEngine* engine = rs::createRSAggregateAnalysisEngine(ae_path);
      loaded_rsaaes.push_back(engine);
      vis.addVisualizableGroupManager(engine->getAAEName());
    }
    vis.start();

    ros::Rate rate(30.0);
    while (ros::ok())
    {
      signal(SIGINT, signalHandler);

      // Analysis Engine processing
      // Copy the loaded RSAAE pointers into a runtime var to be able to modify it
      running_rsaaes = loaded_rsaaes;

      auto size = running_rsaaes.size();

      for (auto i = 0; i < size; i++)
      {
        RSAggregateAnalysisEngine* rsaae = running_rsaaes[i];
        rsaae->resetCas();
        rsaae->processOnce();

        rs::SceneCas scene_cas(*rsaae->getCas());
        cv::Mat dummy_mat;
        if(scene_cas.get(VIEW_AAE_RERUN,dummy_mat)){

//
//        // Check if this AAE should be rerun, for example when refinements are requested.
//        try
//        {
          // We check if there exists a VIEW_AAE_RERUN. We don't take about the particular value right now.
//          uima::CAS *view = rsaae->getCas()->getView(VIEW_AAE_RERUN);
          outInfo("RUNAAE : Got AAE RERUN view. Re-Adding AAE=" << rsaae->getAAEName() << " to running RSAAEs.");
          running_rsaaes.insert(running_rsaaes.begin() + i+1, rsaae);
          size++;
        }
        else
//        catch(const uima::CASException &e)
        {
            outInfo("RUNAAE : AAE_RERUN DOESN'T EXIST.");
          // Add the finished CAS ptr to the CASConsumerContext
          // We only do this when a AAE is finished, because otherwise we'll keep them in the CASConsumerContext
          // and couldn't directly use the previous CAS when we want to access the previous AAE CAS.
          rs::CASConsumerContext::getInstance().addCAS(rsaae->getAAEName(), rsaae->getCas());
          if (publishResults)
          {
            std::vector<std::string> obj_descriptions;
            rs::ObjectDesignatorFactory dw(rsaae->getCas(), rs::ObjectDesignatorFactory::Mode::CLUSTER);
            dw.getObjectDesignators(obj_descriptions);
            robosherlock_msgs::RSObjectDescriptions objDescr;
            objDescr.obj_descriptions = obj_descriptions;
            result_pub.publish(objDescr);
          }
        }
      } // end of AE processing
      outInfo("--------------------------------------- END OF RSAAE ITERATION");

      // Clear up CASes after consumption modules are done
      rs::CASConsumerContext::getInstance().clearCASes();

      rate.sleep();
    }
  }
  catch (const rs::Exception& e)
  {
    outError("Exception: [" << e.what() << "]");
  }
  catch (const uima::Exception& e)
  {
    outError("Exception: " << std::endl << e);
  }
  catch (const std::exception& e)
  {
    outError("Exception: " << std::endl << e.what());
  }
  catch (...)
  {
    outError("Unknown exception!");
  }
  ros::shutdown();
  for (auto rsaae : loaded_rsaaes)
    delete rsaae;

  return 0;
}
