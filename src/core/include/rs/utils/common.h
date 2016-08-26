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

#ifndef COMMON_H
#define COMMON_H

#include <ros/package.h>
#include <iostream>
#include <stdlib.h>
#include <sys/stat.h>
#include <errno.h>
#include <opencv/highgui.h>

#include <tf/tf.h>
#include <rs/utils/output.h>

namespace rs
{

#define SEARCHPATH "/descriptors/analysis_engines/"

namespace common
{

#define COLOR_TO_SCALAR(i) CV_RGB((colors[i] >> 16) & 0xFF, (colors[i] >> 8) & 0xFF, colors[i] & 0xFF)
#define COLOR_TO_VEC3B(i) cv::Vec3b(colors[i] & 0xFF, (colors[i] >> 8) & 0xFF, (colors[i] >> 16) & 0xFF)

static const uint32_t colors[] =
{
  0xFF0000,
  0x00FF00,
  0x0000FF,
  0xFFFF00,
  0xFF00FF,
  0x00FFFF,
  0xC00000,
  0x00C000,
  0x0000C0,
  0xC0C000,
  0xC000C0,
  0x00C0C0,
  0x800000,
  0x008000,
  0x000080,
  0x808000,
  0x800080,
  0x008080
};

static const cv::Scalar cvScalarColors[] =
{
  COLOR_TO_SCALAR(0),
  COLOR_TO_SCALAR(1),
  COLOR_TO_SCALAR(2),
  COLOR_TO_SCALAR(3),
  COLOR_TO_SCALAR(4),
  COLOR_TO_SCALAR(5),
  COLOR_TO_SCALAR(6),
  COLOR_TO_SCALAR(7),
  COLOR_TO_SCALAR(8),
  COLOR_TO_SCALAR(9),
  COLOR_TO_SCALAR(10),
  COLOR_TO_SCALAR(11),
  COLOR_TO_SCALAR(12),
  COLOR_TO_SCALAR(13),
  COLOR_TO_SCALAR(14),
  COLOR_TO_SCALAR(15),
  COLOR_TO_SCALAR(16),
  COLOR_TO_SCALAR(17)
};

static const cv::Vec3b cvVec3bColors[] =
{
  COLOR_TO_VEC3B(0),
  COLOR_TO_VEC3B(1),
  COLOR_TO_VEC3B(2),
  COLOR_TO_VEC3B(3),
  COLOR_TO_VEC3B(4),
  COLOR_TO_VEC3B(5),
  COLOR_TO_VEC3B(6),
  COLOR_TO_VEC3B(7),
  COLOR_TO_VEC3B(8),
  COLOR_TO_VEC3B(9),
  COLOR_TO_VEC3B(10),
  COLOR_TO_VEC3B(11),
  COLOR_TO_VEC3B(12),
  COLOR_TO_VEC3B(13),
  COLOR_TO_VEC3B(14),
  COLOR_TO_VEC3B(15),
  COLOR_TO_VEC3B(16),
  COLOR_TO_VEC3B(17)
};

static const size_t numberOfColors = sizeof(colors) / sizeof(colors[0]);

static const std::string base64_chars =
             "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

inline std::string base64_encode(unsigned char const *bytes_to_encode, unsigned int in_len)
{
  std::string ret;
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];

  while(in_len--)
  {
    char_array_3[i++] = *(bytes_to_encode++);
    if(i == 3)
    {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for(i = 0; (i < 4) ; i++)
      {
        ret += base64_chars[char_array_4[i]];
      }
      i = 0;
    }
  }

  if(i)
  {
    for(j = i; j < 3; j++)
    {
      char_array_3[j] = '\0';
    }

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for(j = 0; (j < i + 1); j++)
    {
      ret += base64_chars[char_array_4[j]];
    }

    while((i++ < 3))
    {
      ret += '=';
    }

  }

  return ret;
}

inline bool getAEPaths(const std::string ae, std::string &aePath)
{
  std::vector<std::string> searchPaths;
  searchPaths.push_back("");
  //add core package path
  searchPaths.push_back(ros::package::getPath("robosherlock") + std::string(SEARCHPATH));

  //look for packages dependent on core and find their full path
  std::vector<std::string> child_packages;
  ros::package::command("depends-on robosherlock", child_packages);
  for(size_t i = 0; i < child_packages.size(); ++i)
  {
    searchPaths.push_back(ros::package::getPath(child_packages[i]) + std::string(SEARCHPATH));
  }

  struct stat fileStat;
  for(size_t j = 0; j < searchPaths.size(); ++j)
  {
    const std::string file = searchPaths[j] + ae;
    const std::string fileXML = file + ".xml";
    //AEs are accepted with or without the xml extension
    if(!stat(file.c_str(), &fileStat) && S_ISREG(fileStat.st_mode))
    {
      aePath = file;
      break;
    }
    else if(!stat(fileXML.c_str(), &fileStat) && S_ISREG(fileStat.st_mode))
    {
      aePath = fileXML;
      break;
    }
  }
  if(aePath.empty())
  {
    outWarn("no AE found with that name");
    return false;
  }
  else
  {
    return true;
  }
}

inline void projectPointOnPlane(tf::Stamped<tf::Pose> &pose, std::vector<float> plane_model)
{
  assert(plane_model.size()==4);
  cv::Point3f normal(plane_model[0], plane_model[1], plane_model[2]);
  float planeDist = plane_model[3];
  cv::Point3f point(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
  float pointDist = point.dot(normal);
  float t = planeDist + pointDist;
  cv::Point3f projected = point - normal * t;
  pose.setOrigin(tf::Vector3(projected.x, projected.y, projected.z));
}

}//end common namespace

}//end rs namespace

#endif
