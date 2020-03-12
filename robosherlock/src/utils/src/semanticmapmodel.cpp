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

#include <string>
#include <iostream>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <boost/algorithm/string.hpp>
#include <string>

#include <ros/ros.h>
#include <json_prolog/prolog.h>
#include <robosherlock/utils/output.h>

using namespace json_prolog;

struct SemanticMapItem
{
  std::string type;
  std::string name;
  cv::Mat transform;
  struct
  {
    double w;
    double h;
    double d;
  } dims;
};


void getTransformationFromSemanticMap(std::string type, std::vector<SemanticMapItem> &smis)
{

  Prolog pl;

  std::string q;
  q = "rdfs_individual_of(CounterTop, 'http://knowrob.org/kb/knowrob.owl#" + type + "'),";
  //  q+= "rdf_has(CounterTop, rdf:type, knowrob:'CounterTop'),";
  q += "current_object_pose(CounterTop, Pose),";
  q += "map_object_dimensions(CounterTop, W,D,H)";
  PrologQueryProxy bdgs = pl.query(q);

  for(PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++)
  {
    PrologBindings bdg = *it;
    SemanticMapItem smi;
    smi.type = type;
    std::vector<std::string> temp;
    std::string cName = bdg["CounterTop"].toString();
    boost::split(temp, cName, boost::is_any_of("#"), boost::token_compress_on);
    if(temp.size() > 0)
    {
      outError("Name of " << type << " : " << temp[temp.size() - 1]); //last element
      smi.name = temp[temp.size() - 1];
    }
    smi.transform = cv::Mat::zeros(4, 4, CV_64F);
    PrologValue pv = bdg["Pose"];
    std::vector<PrologValue> pose_values  = pv.as<std::vector<PrologValue>>();
    for(int i = 0; i < pose_values.size(); ++i)
    {
      smi.transform.at<double>(i / 4, i % 4) = pose_values[i].isDouble() ? pose_values[i].as<double>() : (double)pose_values[i].as<int64_t>();
    }

    smi.dims.w = bdg["W"].isDouble() ? bdg["W"].as<double>() : (double)bdg["W"].as<int64_t>();
    smi.dims.d = bdg["D"].isDouble() ? bdg["D"].as<double>() : (double)bdg["D"].as<int64_t>();
    smi.dims.h = bdg["H"].isDouble() ? bdg["H"].as<double>() : (double)bdg["H"].as<int64_t>();
    smis.push_back(smi);
  }
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "rs_semantic_map");
  std::vector<SemanticMapItem> smis;
  cv::FileStorage fs("semantic_map.yaml", cv::FileStorage::WRITE);

  getTransformationFromSemanticMap("Drawer", smis);
  getTransformationFromSemanticMap("CounterTop", smis);
  getTransformationFromSemanticMap("Cupboard", smis);
  getTransformationFromSemanticMap("Handle", smis);
  getTransformationFromSemanticMap("Wall", smis);
  getTransformationFromSemanticMap("Door", smis);
  getTransformationFromSemanticMap("Refrigerator", smis);
  //getTransformationFromSemanticMap("PrismaticJoint",smis);
  //getTransformationFromSemanticMap("HingedJoint",smis);

  fs << "names" << "[";
  for(int i = 0; i < smis.size(); ++i)
  {
    fs << smis[i].name;
  }
  fs << "]";
  for(int i = 0; i < smis.size(); ++i)
  {
    fs << smis[i].name << "{" << "type" << smis[i].type << "width" << smis[i].dims.w << "height" << smis[i].dims.h << "depth" << smis[i].dims.d <<
       "transform" << smis[i].transform << "}";
  }
  fs.release();
  return 0;
}
