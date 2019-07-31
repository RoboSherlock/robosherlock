#include "ros/ros.h"
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "robosherlock_msgs/RSQueryService.h"
#include "robosherlock_msgs/ExecutePipeline.h"

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
