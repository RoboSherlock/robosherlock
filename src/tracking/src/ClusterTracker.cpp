/*
 * Copyright (c) 2012, Florian Seidel <seidel.florian@gmail.com>
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

#include <uima/api.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include <boost/ref.hpp>

#include <queue>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>

using namespace uima;
using namespace rs;

typedef struct WorldPose_
{
  //rs::Cluster cluster;
  long trackingID;
  tf::Vector3 worldPose;

  WorldPose_()
  {

  }

  //  WorldPose_(rs::Cluster cluster, btVector3 worldPose):cluster(cluster),worldPose(worldPose)
  WorldPose_(int trackingID, tf::Vector3 worldPose) :
    trackingID(trackingID), worldPose(worldPose)
  {

  }

  WorldPose_(const WorldPose_& pose)
  {
    //    cluster=pose.cluster;
    trackingID = pose.trackingID;
    worldPose = pose.worldPose;
  }

  WorldPose_ &operator=(const WorldPose_& pose)
  {
    //    cluster=pose.cluster;
    trackingID = pose.trackingID;
    worldPose = pose.worldPose;
    return *this;
  }

} WorldPose;

std::ostream &operator <<(std::ostream &output, WorldPose &pose)
{
  output << pose.worldPose;
  return output;
}

typedef struct IndexDistPair_
{
  float dist_;
  int index1_;
  int index2_;

  IndexDistPair_(float dist, float index1, float index2) :
    dist_(dist), index1_(index1), index2_(index2)
  {

  }

  IndexDistPair_(const IndexDistPair_& other) :
    dist_(other.dist_), index1_(other.index1_), index2_(other.index2_)
  {

  }

  IndexDistPair_ &operator =(const IndexDistPair_& other)
  {
    dist_ = other.dist_;
    index1_ = other.index1_;
    index2_ = other.index2_;
    return *this;
  }

  bool operator <(const IndexDistPair_& b) const
  {
    return dist_ < b.dist_;
  }

  bool operator >(const IndexDistPair_& b) const
  {
    return dist_ > b.dist_;
  }

} IndexDistPair;

typedef std::priority_queue<IndexDistPair, std::vector<IndexDistPair>, std::greater<IndexDistPair> > queue;

class ClusterTracker : public Annotator
{

private:
  Type cloud_type;
  std::vector<WorldPose> lastClustersPose;
  long nextID;
  float max_dist;
  double minDist;

public:

  ClusterTracker() :
    nextID(1)
  {

  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("max_dist"))
    {
      ctx.extractValue("max_dist", max_dist);
    }
    else
    {
      max_dist = 25.0;
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  std::vector<WorldPose> compute_world_poses(std::vector<rs::Cluster> thisClusters, tf::StampedTransform vp,
      CAS &tcas)
  {
    std::vector<WorldPose> thisClustersPose;
    for(int i = 0; i < thisClusters.size(); i++)
    {
      rs::SceneCas cas(tcas);
      rs::Cluster cluster = thisClusters.at(i);

      tf::Vector3 btCentroid;
      if(cluster.points.has() && cluster.points().type() == rs::type<ReferenceClusterPoints>(tcas))
      {
        pcl::PointCloud<pcl::PointXYZRGBA> pointCloud;
        pcl::PointIndices indices;

        ReferenceClusterPoints cp(cluster.points());

        cas.get(VIEW_CLOUD, pointCloud);
        indices.indices = cp.indices.get().indices.get();
        float x = 0, y = 0, z = 0;
        for(int i = 0; i < indices.indices.size() - 1; i++)
        {
          x += pointCloud.points[indices.indices[i]].x;
          y += pointCloud.points[indices.indices[i]].y;
          z += pointCloud.points[indices.indices[i]].z;
        }
        btCentroid[0] = x / indices.indices.size();
        btCentroid[1] = y / indices.indices.size();
        btCentroid[2] = z / indices.indices.size();
      }
      else if(cluster.points.has() && cluster.points().type() == rs::type<StandaloneClusterPoints>(tcas))
      {
        pcl::PointCloud<pcl::PointXYZRGBA> pointCloud;

        StandaloneClusterPoints cp(cluster.points());

        cas.get(VIEW_CLOUD, pointCloud);
        float x = 0, y = 0, z = 0;
        for(int i = 0; i < pointCloud.points.size(); i++)
        {
          x += pointCloud.points[i].x;
          y += pointCloud.points[i].y;
          z += pointCloud.points[i].z;
        }

        btCentroid[0] = x / pointCloud.points.size();
        btCentroid[1] = y / pointCloud.points.size();
        btCentroid[2] = z / pointCloud.points.size();
      }

      btCentroid = vp.getBasis() * btCentroid + vp.getOrigin();
      WorldPose pose(0, btCentroid);
      thisClustersPose.push_back(pose);

      outDebug("Cluster " << i << " " << pose);
    }
    return thisClustersPose;
  }

  void track(std::vector<rs::Cluster> &thisClusters, std::vector<WorldPose> &thisClustersPose, CAS &tcas)
  {
    outDebug("Tracking");
    if(lastClustersPose.empty())
    {
      outDebug("First iteration, assigning new cluster ids to all clusters");
      for(int i = 0; i < thisClustersPose.size(); i++)
      {
        thisClustersPose[i].trackingID = nextID++;
        rs::Tracking annotation = rs::create<rs::Tracking>(tcas);
        annotation.trackingID.set(thisClustersPose[i].trackingID);
        annotation.annotatorID.set("TFTracker");
        thisClusters[i].annotations.append(annotation);
      }
    }
    else
    {
      int thisSize = thisClustersPose.size();
      int lastSize = lastClustersPose.size();

      Eigen::MatrixXf distances(thisSize, lastSize); //For debug purposes
      queue q;
      for(int i = 0; i < thisSize; i++)
      {
        for(int j = 0; j < lastSize; j++)
        {
          float d = (thisClustersPose[i].worldPose - lastClustersPose[j].worldPose).length2();
          outDebug(d);
          distances(i, j) = d;
          IndexDistPair idp(d, i, j);
          q.push(idp);
        }
      }
      outDebug("Distance matrix:");
      outDebug(distances);

      std::vector<bool> thisAssigned(thisSize);
      std::vector<bool> lastAssigned(lastSize);
      for(int i = 0; i < thisSize; i++)
      {
        thisAssigned[i] = false;
      }
      for(int i = 0; i < lastSize; i++)
      {
        lastAssigned[i] = false;
      }
      int no_assigned = 0;
      while(!q.empty() && no_assigned < thisSize)
      {
        outDebug("Assigning Clusters ");
        outDebug("====================================");
        IndexDistPair p = q.top();
        q.pop();
        if(p.dist_ > max_dist)
        {
          outDebug("Distance from " << p.index1_ << " to " << p.index2_ << " exceeds threshold; breaking");
          break;
        }
        if(!thisAssigned[p.index1_] && !lastAssigned[p.index2_])
        {
          long lastID = lastClustersPose[p.index2_].trackingID;
          if(lastID == 0)
          {
            outError("LastID = 0!");
            continue;
          }

          no_assigned++;
          outDebug("Assigning current cluster with index " << p.index1_ << " to past cluster with index "
                   << p.index2_ << " (" << thisAssigned.size() << ", " << lastAssigned.size() << ")");

          thisAssigned[p.index1_] = true;
          lastAssigned[p.index2_] = true;

          outDebug("last ID " << lastID);
          thisClustersPose[p.index1_].trackingID = lastID;
          outDebug("Distance: " << p.dist_);
          outDebug("Distance to others: " << distances.row(p.index1_));
        }
      }
      outDebug("Assigning new id to new clusters");
      if(no_assigned <= thisSize)
        for(int i = 0; i < thisSize; i++)
        {
          rs::Tracking annotation = rs::create<rs::Tracking>(tcas);
          if(!thisAssigned[i])
          {
            annotation.trackingID.set(nextID++);
            annotation.annotatorID.set("TFTracker");
          }
          else
          {
            annotation.trackingID.set(thisClustersPose[i].trackingID);
            annotation.annotatorID.set("TFTracker");
          }
          thisClusters[i].annotations.append(annotation);
        }
    }
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    if(scene.viewPoint.has())
    {
      outDebug("Scene has viewpoint");
      tf::StampedTransform vp;
      conversion::from(scene.viewPoint.get(), vp);

      if(!scene.identifiables.empty())
      {
        std::vector<rs::Cluster> thisClusters;
        scene.identifiables.filter(thisClusters);

        outDebug("Processing " << thisClusters.size() << " clusters");
        if(!thisClusters.empty())
        {
          std::vector<WorldPose> thisClustersPose = compute_world_poses(thisClusters, vp, tcas);

          //Track
          track(thisClusters, thisClustersPose, tcas);

          lastClustersPose = thisClustersPose;
        }
        outDebug("====================================");
      }
    }

    for(int i = 0; i < lastClustersPose.size(); i++)
    {
      outDebug("last cluster pose " << i << " : " << lastClustersPose[i].trackingID);
    }

    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ClusterTracker)
