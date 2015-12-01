#ifndef __DRAWING_ANNOTATOR_H__
#define __DRAWING_ANNOTATOR_H__

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

class DrawingAnnotator : public uima::Annotator
{
public:
  enum Source
  {
    IMAGE_VIEWER = 0,
    CLOUD_VIEWER
  };

  const std::string name;
  bool update;
  bool hasRun;

private:
  static std::map<std::string, DrawingAnnotator *> annotators;

  std::mutex drawLock;

public:
  DrawingAnnotator(const std::string &name);
  virtual ~DrawingAnnotator();

  static void getAnnotatorNames(std::vector<std::string> &names);
  static DrawingAnnotator *getAnnotator(const std::string &name);

  uima::TyErrorId process(uima::CAS &tcas, uima::ResultSpecification const &res_spec);

  void drawImage(cv::Mat &disp);
  bool fillVisualizer(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun);

  virtual bool callbackMouse(const int event, const int x, const int y, const Source source);
  virtual bool callbackKey(const int key, const Source source);

protected:
  virtual uima::TyErrorId processWithLock(uima::CAS &tcas, uima::ResultSpecification const &res_spec) = 0;

  virtual void drawImageWithLock(cv::Mat &disp);
  virtual void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun);
};

#endif //__DRAWING_ANNOTATOR_H__
