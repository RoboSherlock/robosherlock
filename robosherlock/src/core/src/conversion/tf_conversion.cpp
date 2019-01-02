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

// ROS
#include <tf/transform_datatypes.h>

// RS
#include <rs/conversion/conversion.h>
#include <rs/types/tf_types.h>

namespace rs
{
namespace conversion
{

template<>
void from(const uima::FeatureStructure &fs, tf::Pose &output)
{
  rs::Pose pose(fs);
  const std::vector<double> &rotVec = pose.rotation.get();
  const std::vector<double> &transVec = pose.translation.get();

  tf::Vector3 trans;


  //rot matrix or quaternion (for backwards compatibility with old logs)
  assert((rotVec.size() == 9 || rotVec.size() == 4) && transVec.size() == 3);

  trans.setValue(transVec[0], transVec[1], transVec[2]);
  if(rotVec.size() == 9) {
    tf::Matrix3x3 rot;
    rot.setValue(rotVec[0], rotVec[1], rotVec[2], rotVec[3], rotVec[4], rotVec[5], rotVec[6], rotVec[7], rotVec[8]);
    tf::Quaternion quat;
    rot.getRotation(quat);
    output = tf::Pose(quat, trans);
  }
  else {
    tf::Quaternion quat;
    quat.setValue(rotVec[0], rotVec[1], rotVec[2], rotVec[3]); //x y z w
    output = tf::Pose(quat, trans);
  }


}

template<>
uima::FeatureStructure to(uima::CAS &cas, const tf::Pose &input)
{
  rs::Pose pose = rs::create<rs::Pose>(cas);

  std::vector<double> quatVec(4), transVec(3);

  const tf::Quaternion quat = input.getRotation();
  const tf::Vector3 &trans = input.getOrigin();

  quatVec.push_back(quat.x());
  quatVec.push_back(quat.y());
  quatVec.push_back(quat.z());
  quatVec.push_back(quat.w());

  transVec[0] = trans[0];
  transVec[1] = trans[1];
  transVec[2] = trans[2];

  pose.rotation.set(quatVec);
  pose.translation.set(transVec);

  return pose;
}

template<>
void from(const uima::FeatureStructure &fs, tf::Stamped<tf::Pose> &output)
{
  rs::StampedPose pose(fs);
  const std::vector<double> &rotVec = pose.rotation.get();
  const std::vector<double> &transVec = pose.translation.get();

  tf::Vector3 trans;

  assert((rotVec.size() == 9 || rotVec.size() == 4) && transVec.size() == 3);

  trans.setValue(transVec[0], transVec[1], transVec[2]);
  if(rotVec.size() == 9) {
    tf::Matrix3x3 rot;
    rot.setValue(rotVec[0], rotVec[1], rotVec[2], rotVec[3], rotVec[4], rotVec[5], rotVec[6], rotVec[7], rotVec[8]);
    tf::Quaternion quat;
    rot.getRotation(quat);
    output = tf::Stamped<tf::Pose>(tf::Pose(quat, trans), ros::Time(pose.timestamp.get() / 1000000000, pose.timestamp.get() % 1000000000), pose.frame.get());
  }
  else {
    tf::Quaternion quat;
    quat.setValue(rotVec[0], rotVec[1], rotVec[2], rotVec[3]); //x y z w
    output = tf::Stamped<tf::Pose>(tf::Pose(quat, trans), ros::Time(pose.timestamp.get() / 1000000000, pose.timestamp.get() % 1000000000), pose.frame.get());
  }
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const tf::Stamped<tf::Pose> &input)
{
  rs::StampedPose tfpose = rs::create<rs::StampedPose>(cas);

  std::vector<double> quatVec(4), transVec(3);

  const tf::Quaternion &quat = input.getRotation();
  const tf::Vector3 &trans = input.getOrigin();

  quatVec[0] = quat.x();
  quatVec[1] = quat.y();
  quatVec[2] = quat.z();
  quatVec[3] = quat.w();

  transVec[0] = trans[0];
  transVec[1] = trans[1];
  transVec[2] = trans[2];

  tfpose.rotation.set(quatVec);
  tfpose.translation.set(transVec);
  tfpose.frame.set(input.frame_id_);
  tfpose.timestamp.set(input.stamp_.toNSec());

  return tfpose;
}

template<>
void from(const uima::FeatureStructure &fs, tf::StampedTransform &output)
{
  rs::StampedTransform transform(fs);
  const std::vector<double> &rotVec = transform.rotation.get();
  const std::vector<double> &transVec = transform.translation.get();

  assert((rotVec.size() == 9 || rotVec.size() == 4) && transVec.size() == 3);

  tf::Vector3 trans;
  trans.setValue(transVec[0], transVec[1], transVec[2]);

  if(rotVec.size() == 9) {
      tf::Matrix3x3 rot;
      rot.setValue(rotVec[0], rotVec[1], rotVec[2], rotVec[3], rotVec[4], rotVec[5], rotVec[6], rotVec[7], rotVec[8]);
      tf::Quaternion quat;
      rot.getRotation(quat);
      output = tf::StampedTransform(tf::Transform(quat, trans), ros::Time(transform.timestamp.get() / 1000000000, transform.timestamp.get() % 1000000000), transform.frame.get(), transform.childFrame.get());
    }
    else {
      tf::Quaternion quat;
      quat.setValue(rotVec[0], rotVec[1], rotVec[2], rotVec[3]); //x y z w
      output = tf::StampedTransform(tf::Transform(quat, trans), ros::Time(transform.timestamp.get() / 1000000000, transform.timestamp.get() % 1000000000), transform.frame.get(), transform.childFrame.get());
    }
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const tf::StampedTransform &input)
{
  rs::StampedTransform transform = rs::create<rs::StampedTransform>(cas);

  std::vector<double> quatVec(4), transVec(3);

  const tf::Quaternion &quat = input.getRotation();
  const tf::Vector3 &trans = input.getOrigin();

  quatVec[0] = quat.x();
  quatVec[1] = quat.y();
  quatVec[2] = quat.z();
  quatVec[3] = quat.w();

  transVec[0] = trans[0];
  transVec[1] = trans[1];
  transVec[2] = trans[2];

  transform.rotation.set(quatVec);
  transform.translation.set(transVec);
  transform.frame.set(input.frame_id_);
  transform.childFrame.set(input.child_frame_id_);
  transform.timestamp.set(input.stamp_.toNSec());

  return transform;
}

}
}
