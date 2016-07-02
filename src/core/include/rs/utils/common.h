#ifndef COMMON_H
#define COMMON_H

#include <ros/package.h>
#include <iostream>
#include <stdlib.h>
#include <sys/stat.h>
#include <errno.h>
#include <opencv/highgui.h>

#include <rs/utils/output.h>

namespace rs
{

#define COLOR_SIZE 12
#define SEARCHPATH "/descriptors/analysis_engines/"

namespace common
{

static const uint32_t colors[] =
{
  0xFF0000, /*red*/
  0x00FF00, /*lime*/
  0x0000FF, /*blue*/
  0xFFFF00, /*yellow*/
  0xFF00FF, /*pink*/
  0x00FFFF, /*aqua*/
  0x800000, /*maroon*/
  0x008000, /*green*/
  0x000080, /**/
  0x808000, /*olive*/
  0x800080,  /**/
  0x008080  /**/
};

static const std::vector<cv::Scalar> cvScalarColorsVec =
{
  CV_RGB(255, 0, 0),
  CV_RGB(255, 255, 0),
  CV_RGB(0, 255, 0),
  CV_RGB(0, 255, 255),
  CV_RGB(0, 0, 255),
  CV_RGB(255, 0, 255),
  CV_RGB(255, 255, 255),
  CV_RGB(0, 0, 0),
  CV_RGB(127, 127, 127)
};

static const std::vector<cv::Vec3b> cvVec3bcolorsVec =
{
  cv::Vec3b(255, 0, 0),
  cv::Vec3b(255, 255, 0),
  cv::Vec3b(0, 255, 0),
  cv::Vec3b(0, 255, 255),
  cv::Vec3b(0, 0, 255),
  cv::Vec3b(255, 0, 255),
  cv::Vec3b(255, 255, 255),
  cv::Vec3b(0, 0, 0),
  cv::Vec3b(127, 127, 127)
};


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

}//end common namespace

}//end rs namespace

#endif
