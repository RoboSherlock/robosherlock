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

//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>

//Boost
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/io/TFBroadcasterWrapper.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <thread>
#include <mutex>
#include <chrono>


typedef pcl::PointXYZRGBA PointT;
using namespace uima;

class TFBroadcaster : public Annotator
{

private:
  ros::NodeHandle nh_;

  std::thread thread;
  TFBroadcasterWrapper broadCasterObject;
  sensor_msgs::CameraInfo cam_info_;

public:

  TFBroadcaster() :
    nh_("~"), broadCasterObject()
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    thread = std::thread(&TFBroadcasterWrapper::run, &broadCasterObject);
    thread.detach();

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    broadCasterObject.terminate();
    
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


    std::vector<rs::ObjectHypothesis> clusters;

    if(!scene.identifiables.empty())
    {
      scene.identifiables.filter(clusters);
      if(!clusters.empty())
      {
        outInfo("Processing " << clusters.size() << " clusters");
        std::vector<tf::StampedTransform> stamped_transforms;

        for(std::vector<rs::ObjectHypothesis>::iterator clust_it = clusters.begin(); clust_it != clusters.end();
            ++clust_it)
        {

          std::vector<rs::PoseAnnotation> pose;
          std::vector<rs::ClusterPart> clusterParts;
          clust_it->annotations.filter(pose);
          clust_it->annotations.filter(clusterParts);

          //skip cluster if it has no pose

          for(rs::PoseAnnotation p : pose)
          {
            tf::Stamped<tf::Pose> tf_stamped_pose;
            rs::conversion::from(p.world(), tf_stamped_pose);
            std::ostringstream oss;
            oss << "rs_cluster" << clust_it - clusters.begin() << "_" << p.source() << "_frame";
            stamped_transforms.push_back(tf::StampedTransform(tf_stamped_pose, ros::Time::now(), tf_stamped_pose.frame_id_, oss.str()));
          }
          int idx = 0;
          for(rs::ClusterPart clPart : clusterParts)
          {
            tf::Stamped<tf::Pose> tf_stamped_pose;
            rs::conversion::from(clPart.pose(), tf_stamped_pose);
            std::stringstream ss;
            ss << "rs_cluster_part_" << clust_it - clusters.begin() << "_" << idx << "_frame";
            stamped_transforms.push_back(tf::StampedTransform(tf_stamped_pose, ros::Time::now(), tf_stamped_pose.frame_id_, ss.str()));
            idx++;
          }

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
