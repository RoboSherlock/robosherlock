#include <uima/api.hpp>
#include <vector>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/progress.hpp>

#include <rs/types/all_types.h>
//RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

using namespace uima;

namespace fs = boost::filesystem;


class DataLoader : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  std::string path_to_data;
  std::vector<std::string> filenames;

  bool isLoop;
  bool isFile;

  int iterator;

  double pointSize;

public:
  DataLoader () : DrawingAnnotator(__func__), pointSize(1.0) {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    iterator = 0;
    isFile = true;
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    //get parameters
    if(ctx.isParameterDefined("path_to_data"))
    {
      ctx.extractValue("path_to_data", path_to_data);
    }
    if(ctx.isParameterDefined("isLoop"))
    {
      ctx.extractValue("isLoop", isLoop);
    }

    fs::path full_path(fs::initial_path<fs::path>());
    full_path = fs::system_complete(fs::path(path_to_data));

    //check if path exists
    if(!fs::exists(full_path)){
      outError("Could not found path provided! Please check again");
    }

    //if it is a directory, find all relevant pcd file
    if(fs::is_directory(full_path)){
      fs::directory_iterator end_iter;
      for(fs::directory_iterator dir_it(full_path); dir_it != end_iter; dir_it++){
        std::string currEntry = dir_it->path().string();
        size_t isPCD = currEntry.find(".pcd");
        if(isPCD != std::string::npos)
          filenames.push_back(currEntry);
      }

      isFile = false;
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;

    outInfo("process begins");
    rs::SceneCas cas(tcas);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

    std::string path;
    if(!isFile){
      path = filenames[iterator];

      iterator++;
      if(iterator > filenames.size() - 1){
        if(isLoop)
          iterator = 0;
        else
          iterator = filenames.size() - 1;
      }
    }
    else{
      path = path_to_data;
    }

    if(pcl::io::loadPCDFile (path, *cloud_ptr) == -1){
      outError("Could not load point cloud file as PCD type. Check path again!");
    }

    cloud = cloud_ptr;
    outInfo("Cloud size: " << cloud->points.size());

    cas.set(VIEW_CLOUD, *cloud);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(firstRun){
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else{
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(DataLoader)
