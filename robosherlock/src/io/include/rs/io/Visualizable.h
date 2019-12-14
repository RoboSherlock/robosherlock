//
// Created by pmania on 14.12.19.
//

#ifndef ROBOSHERLOCK_VISUALIZABLE
#define ROBOSHERLOCK_VISUALIZABLE

#include <vector>
#include <map>
#include <string>
#include <mutex>

#include <uima/api.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rs/utils/output.h>

class Visualizable
{
public:
  enum VisualizableDataType
  {
    IMAGE_VIEWER = 0,
    CLOUD_VIEWER
  };

  const std::string name;
  bool update;

private:
  static std::map<std::string, Visualizable *> visualizables; // TODO RENAME

public:
  Visualizable(const std::string &name);
  virtual ~Visualizable();

  static void clearAnnotatorList();
  // clear inMap and copy the list of current DrawingAnnotators into inMap
  static int copyAnnotatorList(std::map<std::string, Visualizable *> &inMap);

  virtual void drawImage(cv::Mat &disp);
  virtual bool fillVisualizer(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun);

  virtual bool callbackMouse(const int event, const int x, const int y, const VisualizableDataType source);
  virtual bool callbackKey(const int key, const VisualizableDataType source);
};

#endif //ROBOSHERLOCK_VISUALIZABLE