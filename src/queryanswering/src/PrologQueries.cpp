#include <rapidjson/document.h>
#include <designator_integration_msgs/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

#include <rs/flowcontrol/RSControledAnalysisEngine.h>
#include <rs/flowcontrol/RSProcessManager.h>
#include <rs/queryanswering/DesignatorWrapper.h>

#include <ros/package.h>

#include <uima/api.hpp>
#include <SWI-cpp.h>

#include <stdio.h>
#include <dlfcn.h>
#include <iostream>
#include <string>
#include <memory>

std::string *req_desig = NULL;

static RSProcessManager *pm;
static RSControledAnalysisEngine *ae;
uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
std::thread thread;

/***************************************************************************
 *                                  ADD DESIGNATOR
 * *************************************************************************/


//what ever happens to all these pointers? This smells like a huge memory leak

PREDICATE(cpp_make_designator, 2)
{
  std::string *desig = new std::string((char *) A1);

  outInfo("Sending back: " << *desig);
  return A2 = static_cast<void *>(desig);
}

PREDICATE(cpp_add_designator, 2)
{

  std::string desigType((char *)A1);
  outInfo("Desigtype: " << desigType);

  std::string *desig = new std::string("{\"detect\":{}}");

  outInfo("Sending back: " << *desig);
  return A2 = static_cast<void *>(desig);
}

PREDICATE(cpp_init_kvp, 3)
{
  void *obj = A1;
  std::string type((char *)A2);
  outInfo("Type: " << type);
  std::string *desig = (std::string *)(obj);
  outInfo("Type: " << *desig);
  return A3 = static_cast<void *>(desig);
}

PREDICATE(cpp_add_kvp, 3)
{
  std::string key = (std::string)A1;
  std::string value = (std::string)A2;
  void *obj = A3;
  std::string *desig = (std::string *)(obj);
  outInfo("Desig now: " << *desig);
  if(desig)
  {
    outInfo("Adding Kvp: (" << key << " : " << value);
    rapidjson::Document json;
    json.Parse(desig->c_str());
    rapidjson::Value &detectJson = json["detect"];
    rapidjson::Value v(key, json.GetAllocator());
    detectJson.AddMember(v, value, json.GetAllocator());

    *desig = rs::DesignatorWrapper::jsonToString(json);
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

PREDICATE(cpp_print_desig, 1)
{
  void *obj = A1;
  std::string *desig = (std::string *)(obj);
  if(desig)
  {
    std::cout << *desig << std::endl;
    return TRUE;
  }
  else
  {
    std::cerr << "Desigantor object not initialized. Can not print" << std::endl;
    return FALSE;
  }
}

PREDICATE(cpp_init_desig, 1)
{
  if(!req_desig)
  {
    std::cerr << "Initializing designator: " << std::endl;
    req_desig =  new std::string("{\"location\":{\"location\":\"on table\"}}");
    return A1 = (void *)req_desig;
  }
  else
  {
    std::cerr << "Designator already initialized" << std::endl;
    return FALSE;
  }
}

PREDICATE(cpp_delete_desig, 1)
{
  void *obj = A1;
  std::string *desig = (std::string *)obj;
  delete desig;
  return TRUE;
}

/***************************************************************************
 *                  Manipulate RS instances/pipelines
 * *************************************************************************/

/**
 * @brief initialize the AnalysisEngine object
 */
PREDICATE(cpp_init_rs, 2)
{
  if(!pm)
  {
    ros::init(ros::M_string(), std::string("/RoboSherlock"));
    ros::NodeHandle nh("~");
    dlopen("libpython2.7.so", RTLD_LAZY | RTLD_GLOBAL);
    outInfo((char *) A1);
    std::string pipelineName((char *)A1);

    std::string pipelinePath;
    outInfo(pipelineName);
    rs::common::getAEPaths(pipelineName, pipelinePath);
    std::vector<std::string> lowLvlPipeline;
    lowLvlPipeline.push_back("CollectionReader");
    std::string configPath =
      ros::package::getPath("rs_queryanswering").append(std::string("/config/config.yaml"));

    std::cerr << "Path to config file: " << configPath << std::endl;

    if(!pipelinePath.empty())
    {
      bool waitForService = false;
      pm = new RSProcessManager(false, waitForService, nh);
      //      pm->setLowLvlPipeline(lowLvlPipeline);
      pm->setUseIdentityResolution(false);
      pm->setUseJsonPrologInterface(true);
      pm->init(pipelinePath, configPath);
      thread = std::thread(&RSProcessManager::run, &(*pm));
      return A2 = (void *)pm;
    }
  }
  return FALSE;
}

PREDICATE(cpp_init_ae, 2)
{
  if(!ae)
  {
    ros::init(ros::M_string(), std::string("/RoboSherlock"));
    ros::NodeHandle nh("~");
    dlopen("libpython2.7.so", RTLD_LAZY | RTLD_GLOBAL);
    outInfo((char *) A1);
    std::string pipelineName((char *)A1);

    std::string pathToAE;
    outInfo(pipelineName);
    rs::common::getAEPaths(pipelineName, pathToAE);
    std::vector<std::string> lowLvlPipeline;
    lowLvlPipeline.push_back("CollectionReader");

    if(!pathToAE.empty())
    {
      ae = new RSControledAnalysisEngine(nh);
      ae->init(pathToAE, lowLvlPipeline);
      return A2 = (void *)ae;
    }
  }
  return FALSE;
}

/**
 * @brief run one iteration of the pipeline
 * in: A1: a list of AEs that should analize the image
 */
PREDICATE(cpp_rs_run_pipeline, 1)
{
  PlTail tail(A1);
  PlTerm e;
  std::vector<std::string> pipeline;
  while(tail.next(e))
  {
    std::string pipelineElement((char *)e);
    std::size_t pos = pipelineElement.find("owl#");
    if(pos != std::string::npos)
      pipeline.push_back(pipelineElement.substr(pos + 4, pipelineElement.length() - pos - 1));
  }
  for(auto e : pipeline)
    std::cerr << e << std::endl;
  if(ae)
  {
    ae->setNextPipeline(pipeline);
    ae->applyNextPipeline();
    ae->process();
  }
  else
    return FALSE;
  return TRUE;
}

PREDICATE(cpp_rs_pause, 1)
{
  if(pm)
  {
    pm->pause();
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

PREDICATE(cpp_remove_ae, 1)
{
  if(ae)
  {
    delete ae;
    ae = NULL;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

PREDICATE(cpp_stop_rs, 1)
{
  if(pm)
  {
    pm->stop();
    delete pm;
    pm = NULL;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}


/**
 * change AE file that is loaded, enables changing the context
 * e.g. from kitchen to cehmlab, where we need different parameterizations
 * */

PREDICATE(rs_render_view, 1)
{
  if(pm)
  {
    std::string objectName((char *)A1);
    pm->renderOffscreen(objectName);
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

PREDICATE(change_context, 1)
{
  if(pm)
  {
    std::string pipelineName((char *)A1);
    std::string newPipelinePath;
    rs::common::getAEPaths(pipelineName, newPipelinePath);

    if(!newPipelinePath.empty())
    {
      std::string currentPipeline = pm->getEngineName();
      if(currentPipeline != newPipelinePath)
      {
        pm->resetAE(newPipelinePath);
        return TRUE;
      }
      else
      {
        outInfo("Already set to: " << newPipelinePath);
        return FALSE;
      }
    }
    else
    {
      return FALSE;
    }
  }
  else
  {
    return FALSE;
  }
}

/**
 * @brief run the process function once
 */
PREDICATE(cpp_process_once, 1)
{
  outInfo("cpp_process_once");
  if(pm)
  {
    outInfo((char *) A1);
    void *myobj = A1;
    std::string *desig  = (std::string *)myobj;
    std::vector<std::string> resp;
    pm->handleQuery(*desig, resp);
    outInfo("handled query");
    return TRUE;
  }
  else
  {
    outInfo("No process manager.");
    return FALSE;
  }
}


PREDICATE(delete_desig, 1)
{
  if(req_desig)
  {
    delete req_desig;
    req_desig = NULL;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}


PREDICATE(write_list, 1)
{
  PlTail tail(A1);
  PlTerm e;

  while(tail.next(e))
  {
    std::cout << (char *)e << std::endl;
  }
  return TRUE;
}
