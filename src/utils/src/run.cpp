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

#include <rs/utils/RSAnalysisEngineManager.h>
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
            << "    _visualization:=true|false     Enable/disable visualization" << std::endl
            << "              _vis:=true|false     shorter version for _visualization" << std::endl
            << "        _save_path:=PATH           Path to where images and point clouds should be stored" << std::endl
            << std::endl
            << "Usage: roslaunch robosherlock rs.launch [options]" << std::endl
            << "Options:" << std::endl
            << "  analysis_engines:=engine1[,...]  List of analysis engines for execution" << std::endl
            << "                ae:=engine1[,...]  shorter version for analysis_engines" << std::endl
            << "     visualization:=true|false     Enable/disable visualization" << std::endl
            << "               vis:=true|false     shorter version for visualization" << std::endl
            << "         save_path:=PATH           Path to where images and point clouds should be stored" << std::endl;
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
  std::vector<std::string> analysisEngines, analysisEnginesCL;
  bool visualization;

  ros::NodeHandle priv_nh = ros::NodeHandle("~");

  for(int argI = 1; argI < argc; ++argI)
  {
    const std::string arg = argv[argI];
    if(arg == "-?" || arg == "-h" || arg == "--help")
    {
      help();
      return 0;
    }
    analysisEnginesCL.push_back(arg);
  }

  priv_nh.param("ae", analysisEnginesArg, std::string(""));
  priv_nh.param("analysis_engines", analysisEnginesArg, analysisEnginesArg);

  priv_nh.param("vis", visualization, true);
  priv_nh.param("visualization", visualization, visualization);

  priv_nh.param("save_path", savePath, std::string(getenv("HOME")));

  // Do not cache parameters to prevent false behaviour with short parameter versions.
  priv_nh.deleteParam("ae");
  priv_nh.deleteParam("analysis_engines");
  priv_nh.deleteParam("vis");
  priv_nh.deleteParam("visualization");
  priv_nh.deleteParam("save_path");

  if(analysisEnginesArg.empty())
  {
    analysisEngines.swap(analysisEnginesCL);
  }
  else
  {
    for(size_t start = 0, end = 0; end != analysisEnginesArg.npos && start < analysisEnginesArg.length(); start = end + 1)
    {
      end = analysisEnginesArg.find(',', start);
      analysisEngines.push_back(analysisEnginesArg.substr(start, end));
    }
  }

  if(savePath.empty())
  {
    savePath = getenv("HOME");
  }

  struct stat fileStat;
  if(stat(savePath.c_str(), &fileStat) || !S_ISDIR(fileStat.st_mode))
  {
    outError("Save path \"" << savePath << "\" does not exist.");
    return -1;
  }

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

  outInfo("startup parameters:" << std::endl
          << "   visualization: " FG_CYAN << (visualization ? "enabled" : "disabled") << NO_COLOR << std::endl
          << "       save_path: " FG_CYAN << savePath << NO_COLOR << std::endl
          << "analysis_engines: " << engineList.str());

  try
  {
    RSAnalysisEngineManager<RSAnalysisEngine> manager(visualization, savePath);

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
