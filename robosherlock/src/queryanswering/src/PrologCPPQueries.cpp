#include "ros/ros.h"
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "robosherlock_msgs/RSQueryService.h"
#include "robosherlock_msgs/ExecutePipeline.h"
#include "robosherlock_msgs/ReconfigureAnnotator.h"
#include "robosherlock_msgs/OverwriteParam.h"

#include <ros/package.h>

#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <stdio.h>
#include <dlfcn.h>
#include <iostream>
#include <string>
#include <memory>

#include <boost/algorithm/algorithm.hpp>

std::string *req_desig = NULL;


/***************************************************************************
 *                                  ADD DESIGNATOR
 * *************************************************************************/


//what ever happens to all these pointers? This smells like a huge memory leak

PREDICATE(cpp_make_designator, 2)
{
  std::string *desig = new std::string((char *) PL_A1);

  std::cout << "Sending back: " << *desig << std::endl;
  return PL_A2 = static_cast<void *>(desig);
}


PREDICATE(cpp_query_rs, 1)
{
  void *query = PL_A1;
  std::string *queryString = (std::string *)(query);
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robosherlock_msgs::RSQueryService>("RoboSherlock/query");
  robosherlock_msgs::RSQueryService srv;
  std::cout << queryString->c_str() << std::endl;
  srv.request.query = queryString->c_str();
  if(client.call(srv)) {
    std::cout << "Call was successful" << std::endl;
    return TRUE;
  }
  else {
    std::cout << "Call was unsuccessful" << std::endl;
    return FALSE;
  }
}

PREDICATE(cpp_execute_pipeline, 1)
{
  PlTail tail(PL_A1);
  PlTerm e;
  std::vector<std::string> pipeline;
  while(tail.next(e)) {
    std::string element((char *)e);
    size_t hashLoc = element.find_last_of("#");
    size_t usLoc = element.find_last_of("_");
    if(usLoc > hashLoc) {
      element = element.substr(hashLoc + 1, usLoc - hashLoc - 1);
    }
    else {
      element = element.substr(hashLoc + 1, element.size() - hashLoc);
    }
    pipeline.push_back(element);

  }
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robosherlock_msgs::ExecutePipeline>("RoboSherlock/execute_pipeline");
  robosherlock_msgs::ExecutePipeline srv;
  srv.request.annotators = pipeline;

  if(client.call(srv)) {
    std::cout << "Calling RoboSherlock/execute_pipeline was successful" << std::endl;
  return TRUE;

  }
  else {
    std::cout << "Calling RoboSherlock/execute_pipeline was unsuccessful" << std::endl;
    return FALSE;
  }
}


PREDICATE(cpp_add_designator, 2)
{

  std::string desigType((char *)PL_A1);
  std::cout << "Desigtype: " << desigType << std::endl;

  std::string *desig = new std::string("{\"detect\":{}}");

  std::cout << "Sending back: " << *desig << std::endl;
  return PL_A2 = static_cast<void *>(desig);
}

PREDICATE(cpp_init_kvp, 3)
{
  void *obj = PL_A1;
  std::string type((char *)PL_A2);
  std::cout << "Type: " << type << std::endl;
  std::string *desig = (std::string *)(obj);
  std::cout << "Type: " << *desig << std::endl;
  return PL_A3 = static_cast<void *>(desig);
}

PREDICATE(cpp_add_kvp, 3)
{
  std::string key = (std::string)PL_A1;
  std::string value = (std::string)PL_A2;
  void *obj = PL_A3;
  std::string *desig = (std::string *)(obj);
  std::cout  << "Desig now: " << *desig << std::endl;
  if(desig) {
    std::cout << "Adding Kvp: (" << key << " : " << value << std::endl;
    rapidjson::Document json;
    json.Parse(desig->c_str());
    rapidjson::Value &detectJson = json["detect"];
    rapidjson::Value v(key, json.GetAllocator());
    detectJson.AddMember(v, value, json.GetAllocator());

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    json.Accept(writer);
    std::string jsonString = buffer.GetString();
    *desig = jsonString;
    return TRUE;
  }
  else {
    return FALSE;
  }
}

PREDICATE(cpp_print_desig, 1)
{
  void *obj = PL_A1;
  std::string *desig = (std::string *)(obj);
  if(desig) {
    std::cout << *desig << std::endl;
    return TRUE;
  }
  else {
    std::cerr << "Desigantor object not initialized. Can not print" << std::endl;
    return FALSE;
  }
}

PREDICATE(cpp_init_desig, 1)
{
  if(!req_desig) {
    std::cerr << "Initializing designator: " << std::endl;
    req_desig =  new std::string("{\"location\":{\"location\":\"on table\"}}");
    return PL_A1 = (void *)req_desig;
  }
  else {
    std::cerr << "Designator already initialized" << std::endl;
    return FALSE;
  }
}

PREDICATE(cpp_delete_desig, 1)
{
  void *obj = PL_A1;
  std::string *desig = (std::string *)obj;
  delete desig;
  return TRUE;
}


PREDICATE(delete_desig, 1)
{
  if(req_desig) {
    delete req_desig;
    req_desig = NULL;
    return TRUE;
  }
  else {
    return FALSE;
  }
}


PREDICATE(write_list, 1)
{
  PlTail tail(PL_A1);
  PlTerm e;

  while(tail.next(e)) {
    std::cout << (char *)e << std::endl;
  }
  return TRUE;
}


/**
 * Conversion methods, needed for overwrite_param service calls
 */

std::vector<int> getIntVector(PlTail prologList)
{
  std::vector<int> result;
  PlTerm e;

  while(prologList.next(e)) {
    result.push_back(e);
  }

  return result;
}

// ROS FLOAT = DOUBLE!
std::vector<double> getFloatVector(PlTail prologList)
{
  std::vector<double> result;
  PlTerm e;

  while(prologList.next(e)) {
    result.push_back(*(double *)&e);
  }

  return result;
}

std::vector<std::string> getStringVector(PlTail prologList)
{
  std::vector<std::string> result;
  PlTerm e;

  while(prologList.next(e)) {
    result.push_back(static_cast<std::string>((char *)e));
  }
  return result;
}


PREDICATE(cpp_reconfigure_annotator, 2)
{
  std::string *annotatorName, *setupName;
  annotatorName = static_cast<std::string *>((void *) PL_A1);
  setupName = static_cast<std::string *>((void *) PL_A2);

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robosherlock_msgs::ReconfigureAnnotator>("RoboSherlock/reconfigure_annotator");
  robosherlock_msgs::ReconfigureAnnotator srv;

  srv.request.annotatorName = *annotatorName;
  srv.request.setupName = *setupName;

  if(client.call(srv)) {
    std::cout << "Calling RoboSherlock/reconfigure_annotator was successful" << std::endl;
    return TRUE;
  }
  else {
    std::cout << "Calling RoboSherlock/reconfigure_annotator was unsuccessful" << std::endl;
    return FALSE;
  }
}


PREDICATE(cpp_overwrite_param, 4)
{
  std::string *annotatorName, *paramName;
  annotatorName = static_cast<std::string *>((void *) PL_A1);
  paramName = static_cast<std::string *>((void *) PL_A2);
  int paramType = PL_A3;
  void *value = PL_A4;

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robosherlock_msgs::OverwriteParam>("RoboSherlock/overwrite_param");
  robosherlock_msgs::OverwriteParam srv;

  srv.request.annotatorName = *annotatorName;
  srv.request.parameterName = *paramName;
  srv.request.parameterType = paramType;

  switch(paramType) {
    case robosherlock_msgs::OverwriteParam::Request::INT:
      srv.request.parameterInteger = PL_A4;
      break;
    case robosherlock_msgs::OverwriteParam::Request::INT_VECTOR:
      srv.request.parameterIntegerVector = getIntVector(PL_A4);
      break;
    case robosherlock_msgs::OverwriteParam::Request::BOOL:
      srv.request.parameterBool = *(bool *)value;
      break;
    case robosherlock_msgs::OverwriteParam::Request::FLOAT:
      srv.request.parameterFloat = PL_A4;
      break;
    case robosherlock_msgs::OverwriteParam::Request::FLOAT_VECTOR:
      srv.request.parameterFloatVector = getFloatVector(PL_A4);
      break;
    case robosherlock_msgs::OverwriteParam::Request::STRING:
      srv.request.parameterString = *(std::string *)value;
      break;
    case robosherlock_msgs::OverwriteParam::Request::STRING_VECTOR:
      srv.request.parameterStringVector = getStringVector(PL_A4);
      break;

    default:
      std::cout << "Invalid parameter data type, aborting." << std::endl;
      return FALSE;
  }

  if(client.call(srv)) {
    std::cout << "Calling RoboSherlock/overwrite_param was successful" << std::endl;
    return TRUE;
  }
  else {
    std::cout << "Calling RoboSherlock/overwrite_param was unsuccessful" << std::endl;
    return FALSE;
  }
}