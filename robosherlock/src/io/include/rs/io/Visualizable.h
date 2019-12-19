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

/**
 * Interface-like class to define the base for something to be used by rs::Visualizer.
 * This class is usually inherited from by DrawingAnnotators or CAS Consumers that need to visualize something.
 *
 * During the Construction of a Visualizable, a pointer to the newly created object will be
 * pushed into Visualizable::visualizables to keep track of new Visualizable instances.
 * These instances will then be picked up by VisualizGroupManagers
 */
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
  static std::map<std::string, Visualizable*> visualizables;

public:
  Visualizable(const std::string& name);
  virtual ~Visualizable();

  static void clearVisualizableList();

  /**
   *  clear inMap and copy the list of current Visualizables into inMap
   */
  static int copyVisualizableList(std::map<std::string, Visualizable*>& inMap);

  virtual void drawImage(cv::Mat& disp);
  virtual bool fillVisualizer(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun);

  virtual bool callbackMouse(const int event, const int x, const int y, const VisualizableDataType source);
  virtual bool callbackKey(const int key, const VisualizableDataType source);
};

#endif  // ROBOSHERLOCK_VISUALIZABLE