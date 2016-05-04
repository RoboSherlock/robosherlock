/*
 * Copyright (c) 2012, Nico Blodow <blodow@in.tum.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    // get segmentation masks
    SemanticMap semanticMap;
    getSemanticMap(cas, semanticMap);

    for(unsigned int i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster cluster(clusters[i]);

      int idx = -1;
      std::string name, relation;

      idx = idx < 0 ? checkOnTopOf(cluster, semanticMap["CounterTop"], name, relation) : idx;
      idx = idx < 0 ? checkInsideOf(cluster, semanticMap["Drawer"], name, relation) : idx;

      if(idx >= 0)
      {
        outInfo("adding tflocation annotation");
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

  int checkOnTopOf(rs::Cluster &cluster, std::vector<rs::SemanticMapObject> &objects, std::string &name, std::string &relation)
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

  int checkInsideOf(rs::Cluster &cluster, std::vector<rs::SemanticMapObject> &objects, std::string &name, std::string &relation)
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
