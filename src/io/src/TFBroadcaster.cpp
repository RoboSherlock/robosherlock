
/*
 * Copyright (c) 2011, Ferenc Balint-Benczedi <balintbe@tzi.de>
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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <thread>
#include <mutex>
#include <chrono>

/**
*Gets the clusters that have poses and broadcasts them to tf
*/
class BroadcasterWrapper
{

private:
  std::vector<tf::StampedTransform> transforms;
  bool updated;
  std::chrono::milliseconds sleepTime;
public:

  std::mutex mutex;

  BroadcasterWrapper(): transforms(), updated(false)
  {
    sleepTime = std::chrono::milliseconds(1000 / 30);
  }

  void run()
  {
    tf::TransformBroadcaster br;
    while(ros::ok())
    {
      mutex.lock();
      if(!transforms.empty())
      {
        ros::Time t = ros::Time::now();
        for(int i = 0; i < transforms.size(); ++i)
        {
          transforms[i].stamp_ = t;
        }
        br.sendTransform(transforms);
      }
      mutex.unlock();
      std::this_thread::sleep_for(sleepTime);
    }
  }

  void addTransforms(const std::vector<tf::StampedTransform> &ts)
  {
    mutex.lock();
    transforms.clear();
    transforms.insert(transforms.end(), ts.begin(), ts.end());
    updated = true;
    mutex.unlock();
  }
};


typedef pcl::PointXYZRGBA PointT;
using namespace uima;

class TFBroadcaster : public Annotator
{

private:
  ros::NodeHandle nh_;
  ros::Publisher pub;

  std::thread thread;
  BroadcasterWrapper broadCasterObject;
  sensor_msgs::CameraInfo cam_info_;

public:

  TFBroadcaster() :
    nh_("~"), broadCasterObject()
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    thread = std::thread(&BroadcasterWrapper::run, &broadCasterObject);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    thread.join();
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }


  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("processing start");
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_CAMERA_INFO, cam_info_);


    std::vector<rs::Cluster> uimaClusters;

    if(!scene.identifiables.empty())
    {
      scene.identifiables.filter(uimaClusters);
      if(!uimaClusters.empty())
      {
        outInfo("Processing " << uimaClusters.size() << " clusters");
        std::vector<tf::StampedTransform> stamped_transforms;

        for(std::vector<rs::Cluster>::iterator clust_it = uimaClusters.begin(); clust_it != uimaClusters.end();
            ++clust_it)
        {

          std::vector<rs::PoseAnnotation> pose;

          clust_it->annotations.filter(pose);
          //skip cluster if it has no pose
          tf::Stamped<tf::Pose> tf_stamped_pose;
          if(!pose.empty())
          {
            rs::conversion::from(pose[0].camera(), tf_stamped_pose);
          }
          else
          {
            continue;
          }

          std::ostringstream oss;
          oss << "cluster" << clust_it - uimaClusters.begin() << "_frame";
          stamped_transforms.push_back(tf::StampedTransform(tf_stamped_pose, ros::Time::now(), cam_info_.header.frame_id, oss.str()));
        }
        broadCasterObject.addTransforms(stamped_transforms);
      }
    }
    usleep(100);
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(TFBroadcaster)

