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

#include <uima/api.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <rs/conversion/bson.h>

//#undef OUT_LEVEL
//#define OUT_LEVEL OUT_LEVEL_DEBUG


using namespace uima;

class ResultAdvertiser : public Annotator
{

private:
  ros::NodeHandle nh_;
  ros::Publisher resPub;

public:
  ResultAdvertiser() : nh_("~")
  {
    resPub = nh_.advertise<std_msgs::String>(std::string("result_advertiser"), 5);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    std::stringstream ss;
    ss<<"{\"results\" : [";
    int idx=0;
    for (auto annotation: scene.annotations())
    {
      ss<<(idx==0?"{\"scene_properties\" : [":"");
      mongo::BSONObj bson;
      rs::conversion::from(annotation, bson);
      //fields that just polute the resulting json. if reaally needed comment them out
      //TODO: recursive function to delete _od and _parent ->fields come from mongo and are not needed here
      bson = bson.removeField(mongo::StringData("inliers"));
      bson = bson.removeField(mongo::StringData("mask"));
      bson = bson.removeField(mongo::StringData("indices"));
      bson = bson.removeField(mongo::StringData("_id"));
      bson = bson.removeField(mongo::StringData("_parent"));
      ss << bson.jsonString(mongo::JsonStringFormat::Strict,1) << (idx++ < scene.annotations.size() - 1 ? "," : "");
    }
    ss<<(idx!=0 ? "]},":"");

    int i = 0;
    for(auto c : clusters)
    {
      ss << (i==0?"{\"object_hypotheses:\": [":"");
      ss << "{\"cluster\" : " << i << "," << std::endl << "\"annotations\" : [";
      rs::Cluster &cluster = c;
      int annotidx = 0;
      for(auto annotation : cluster.annotations())
      {
        mongo::BSONObj bson;
        rs::conversion::from(annotation, bson);
        bson = bson.removeField(mongo::StringData("_id"));
        bson = bson.removeField(mongo::StringData("_parent"));
        ss << bson.jsonString(mongo::JsonStringFormat::Strict,1) << (annotidx++ < cluster.annotations.size() - 1 ? "," : "");
      }
      ss << "]}" << (i++ < clusters.size() - 1 ? "," : ""); //end of annotations
    }
    ss<<(i!=0? "]}":"");
    ss<<"]}";


    std_msgs::String msg;
    msg.data = ss.str();
    resPub.publish(msg);
    return UIMA_ERR_NONE;
  }
};

MAKE_AE(ResultAdvertiser)
