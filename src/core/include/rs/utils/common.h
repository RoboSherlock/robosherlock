
//header file for common

#include <ros/package.h>
#include <iostream>
#include <stdlib.h>
#include <sys/stat.h>
#include <errno.h>


namespace rs
{

#define COLOR_SIZE 12
#define SEARCHPATH "/descriptors/analysis_engines/"

namespace common
{
static const uint32_t colors[] = {0xFF0000, /*red*/
                                  0x00FF00, /*lime*/
                                  0x0000FF, /*blue*/
                                  0xFFFF00, /*yellow*/
                                  0xFF00FF, /*pink*/
                                  0x00FFFF, /*aqua*/
                                  0x800000, /*maroon*/
                                  0x008000, /*green*/
                                  0x000080, /**/
                                  0x808000, /*olive*/
                                  0x800080, /**/
                                  0x008080  /**/
                                 };

void getAEPaths(const std::string ae, std::string &aePath)
{
  std::vector<std::string> searchPaths;
  searchPaths.push_back("");
  //add core package path
  searchPaths.push_back(ros::package::getPath("robosherlock") + std::string(SEARCHPATH));

  //look for packages dependent on core and find their full path
  std::vector<std::string> child_packages;
  ros::package::command("depends-on robosherlock", child_packages);
  for(int i = 0; i < child_packages.size(); ++i)
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
    outInfo("no AE found with that name");
  }
}

}//end common namespace

}//end rs namespace
