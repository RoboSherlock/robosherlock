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

  tf::Matrix3x3 rot;
  tf::Vector3 trans;

  assert(rotVec.size() == 9 && transVec.size() == 3);

  rot.setValue(rotVec[0], rotVec[1], rotVec[2], rotVec[3], rotVec[4], rotVec[5], rotVec[6], rotVec[7], rotVec[8]);
  trans.setValue(transVec[0], transVec[1], transVec[2]);

  output = tf::Pose(rot, trans);
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const tf::Pose &input)
{
  rs::Pose pose = rs::create<rs::Pose>(cas);

  std::vector<double> rotVec(9), transVec(3);

  const tf::Matrix3x3 &rot = input.getBasis();
  const tf::Vector3 &trans = input.getOrigin();

  for(int r = 0, i = 0; r < 3; ++r)
  {
    const tf::Vector3 &row = rot[r];
    for(int c = 0; c < 3; ++c, ++i)
    {
      rotVec[i] = row[c];
    }
  }
  transVec[0] = trans[0];
  transVec[1] = trans[1];
  transVec[2] = trans[2];

  pose.rotation.set(rotVec);
  pose.translation.set(transVec);

  return pose;
}

template<>
void from(const uima::FeatureStructure &fs, tf::Stamped<tf::Pose> &output)
{
  rs::StampedPose pose(fs);
  const std::vector<double> &rotVec = pose.rotation.get();
  const std::vector<double> &transVec = pose.translation.get();

  tf::Matrix3x3 rot;
  tf::Vector3 trans;

  assert(rotVec.size() == 9 && transVec.size() == 3);

  rot.setValue(rotVec[0], rotVec[1], rotVec[2], rotVec[3], rotVec[4], rotVec[5], rotVec[6], rotVec[7], rotVec[8]);
  trans.setValue(transVec[0], transVec[1], transVec[2]);

  output = tf::Stamped<tf::Pose>(tf::Pose(rot, trans), ros::Time(pose.timestamp.get() / 1000000000, pose.timestamp.get() % 1000000000), pose.frame.get());
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const tf::Stamped<tf::Pose> &input)
{
  rs::StampedPose tfpose = rs::create<rs::StampedPose>(cas);

  std::vector<double> rotVec(9), transVec(3);

  const tf::Matrix3x3 &rot = input.getBasis();
  const tf::Vector3 &trans = input.getOrigin();

  for(int r = 0, i = 0; r < 3; ++r)
  {
    const tf::Vector3 &row = rot[r];
    for(int c = 0; c < 3; ++c, ++i)
    {
      rotVec[i] = row[c];
    }
  }
  transVec[0] = trans[0];
  transVec[1] = trans[1];
  transVec[2] = trans[2];

  tfpose.rotation.set(rotVec);
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

  tf::Matrix3x3 rot;
  tf::Vector3 trans;

  assert(rotVec.size() == 9 && transVec.size() == 3);

  rot.setValue(rotVec[0], rotVec[1], rotVec[2], rotVec[3], rotVec[4], rotVec[5], rotVec[6], rotVec[7], rotVec[8]);
  trans.setValue(transVec[0], transVec[1], transVec[2]);

  output = tf::StampedTransform(tf::Transform(rot, trans), ros::Time(transform.timestamp.get() / 1000000000, transform.timestamp.get() % 1000000000), transform.frame.get(), transform.childFrame.get());
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const tf::StampedTransform &input)
{
  rs::StampedTransform transform = rs::create<rs::StampedTransform>(cas);
  rs::Pose pose = rs::create<rs::Pose>(cas);

  std::vector<double>
  rotVec(9),
         transVec(3);

  const tf::Matrix3x3 &rot = input.getBasis();
  const tf::Vector3 &trans = input.getOrigin();

  for(int r = 0, i = 0; r < 3; ++r)
  {
    const tf::Vector3 &row = rot[r];
    for(int c = 0; c < 3; ++c, ++i)
    {
      rotVec[i] = row[c];
    }
  }
  transVec[0] = trans[0];
  transVec[1] = trans[1];
  transVec[2] = trans[2];

  transform.rotation.set(rotVec);
  transform.translation.set(transVec);
  transform.frame.set(input.frame_id_);
  transform.childFrame.set(input.child_frame_id_);
  transform.timestamp.set(input.stamp_.toNSec());

  return transform;
}

}
}
