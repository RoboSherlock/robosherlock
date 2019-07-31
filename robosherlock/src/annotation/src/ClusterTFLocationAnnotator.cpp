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
#include <vector>

#include <uima/api.hpp>
#include <uima/fsfilterbuilder.hpp>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>

using namespace uima;

class ClusterTFLocationAnnotator : public Annotator
{
private:
  struct Region
  {
    tf::Transform transform;
    float width, depth, height;
  };
  typedef std::map<std::string, std::vector<rs::SemanticMapObject>> SemanticMap;

public:

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

    // get clusters
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);

    // get segmentation masks
    SemanticMap semanticMap;
    getSemanticMap(cas, semanticMap);

    for(unsigned int i = 0; i < clusters.size(); ++i)
    {
      rs::ObjectHypothesis cluster(clusters[i]);

      int idx = -1;
      std::string name, relation;

      idx = idx < 0 ? checkOnTopOf(cluster, semanticMap["CounterTop"], name, relation) : idx;
      idx = idx < 0 ? checkInsideOf(cluster, semanticMap["Drawer"], name, relation) : idx;

      if(idx >= 0)
      {
        outInfo("adding tflocation annotation:  "<<relation<<" "<<name);
        rs::TFLocation annotation = rs::create<rs::TFLocation>(tcas);
        annotation.frame_id.set(name);
        annotation.reference_desc.set(relation);
        cluster.annotations.append(annotation);
      }
      else
      {
        outWarn("could not determine relative location");
      }
    }


    return UIMA_ERR_NONE;
  }

private:
  void getSemanticMap(rs::SceneCas &cas, SemanticMap &mapObjects)
  {
    std::vector<rs::SemanticMapObject> objects;
    cas.get(VIEW_SEMANTIC_MAP, objects);

    for(size_t i = 0; i < objects.size(); ++i)
    {
      mapObjects[objects[i].typeName()].push_back(objects[i]);
    }
  }

  int checkOnTopOf(rs::ObjectHypothesis &cluster, std::vector<rs::SemanticMapObject> &objects, std::string &name, std::string &relation)
  {
    std::vector<rs::PoseAnnotation> poses;
    cluster.annotations.filter(poses);

    if(poses.empty())
    {
      return -1;
    }

    tf::Stamped<tf::Pose> poseW;
    rs::conversion::from(poses[0].world(), poseW);
    const tf::Vector3 &vecW = poseW.getOrigin();
    for(size_t i = 0; i < objects.size(); ++i)
    {
      rs::SemanticMapObject &object = objects[i];
      tf::Transform transform;
      rs::conversion::from(object.transform(), transform);
      tf::Vector3 vecT = transform.inverse() * vecW;

      const float minX = -(object.width() / 2);
      const float maxX = (object.width() / 2);
      const float minY = -(object.height() / 2);
      const float maxY = (object.height() / 2);
      const float minZ = -(object.depth() / 2);

      if(vecT.x() > minX && vecT.x() < maxX && vecT.y() > minY && vecT.y() < maxY && vecT.z() > minZ)
      {
        name = object.name();
        relation = "on";
        return i;
      }
    }
    return -1;
  }

  int checkInsideOf(rs::ObjectHypothesis &cluster, std::vector<rs::SemanticMapObject> &objects, std::string &name, std::string &relation)
  {
    std::vector<rs::PoseAnnotation> poses;
    cluster.annotations.filter(poses);

    if(poses.empty())
    {
      return -1;
    }

    tf::Stamped<tf::Pose> poseW;
    rs::conversion::from(poses[0].world(), poseW);
    const tf::Vector3 &vecW = poseW.getOrigin();

    for(size_t i = 0; i < objects.size(); ++i)
    {
      rs::SemanticMapObject &object = objects[i];
      tf::Transform transform;
      rs::conversion::from(object.transform(), transform);
      const tf::Vector3 &vecT = transform.inverse() * vecW;

      const float minX = -(object.width() / 2);
      const float maxX = (object.width() / 2);
      const float minY = -(object.height() / 2);
      const float maxY = (object.height() / 2);
      const float minZ = -(object.depth() / 2);
      const float maxZ = -(object.depth() / 2);

      if(vecT.x() > minX && vecT.x() < maxX && vecT.y() > minY && vecT.y() < maxY && vecT.z() > minZ && vecT.z() < maxZ)
      {
        name = object.name();
        relation = "inside of";
        return i;
      }
    }
    return -1;
  }
};

MAKE_AE(ClusterTFLocationAnnotator)
