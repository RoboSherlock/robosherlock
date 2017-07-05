#include <uima/api.hpp>
#include <vector>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>


#include <rs/types/all_types.h>
//RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

using namespace uima;


class LoadPointCloud : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  std::string folder;
  std::vector<std::string* > filenames;

  int iterator;

  double pointSize;

public:
  LoadPointCloud () : DrawingAnnotator(__func__), pointSize(1.0) {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    iterator = 0;
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("folder"))
    {
      ctx.extractValue("folder", folder);
    }
    if(ctx.isParameterDefined("filenames"))
    {
      ctx.extractValue("filenames", filenames);
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

    std::string path = folder + *filenames[iterator];

    if(pcl::io::loadPCDFile (path, *cloud_ptr) == -1){
      outError("Could not load point cloud file as PCD type. Check path again!");
    }

    cloud = cloud_ptr;
    outInfo("Cloud size: " << cloud->points.size());

    iterator++;
    if(iterator > filenames.size() - 1)
      iterator = 0;

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
MAKE_AE(LoadPointCloud)
