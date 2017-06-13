#include <uima/api.hpp>
#include <vector>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/boundary.h>
#include <pcl/common/io.h>
#include <pcl/search/impl/kdtree.hpp>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

#include <rs/segmentation/array_utils.hpp>
#include <rs/segmentation/BoundarySegmentation.hpp>



using namespace uima;


class EdgeCurveAnnotator : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr boundary_cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr non_boundary_cloud;

  float differentAngleThreshold;
  float radiusSearch;

  double pointSize;

public:
  EdgeCurveAnnotator () : DrawingAnnotator(__func__), pointSize(1.0) {
    boundary_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    non_boundary_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("differentAngleThreshold"))
    {
      ctx.extractValue("differentAngleThreshold", differentAngleThreshold);
    }
    if(ctx.isParameterDefined("radiusSearch"))
    {
      ctx.extractValue("radiusSearch", radiusSearch);
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

    //get RGB cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);

    //get normal cloud
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_NORMALS, *normals);

    //discard color information
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz (new  pcl::PointCloud<pcl::PointXYZ>);
    //pcl::copyPointCloud(*cloud_ptr, *cloudxyz);

    //main execution
    if(!extractBoundaryCloud<pcl::PointXYZRGBA, pcl::Normal>(cloud_ptr, normals, boundary_cloud, non_boundary_cloud, radiusSearch, differentAngleThreshold))
      outError("Extract Boundary points failed!");

    outInfo("Boundary cloud size = " << boundary_cloud->points.size());

    //publish cloud to CAS
    cas.set(VIEW_CLOUD_BOUNDARY, *boundary_cloud);
    cas.set(VIEW_CLOUD_NON_BOUNDARY, *non_boundary_cloud);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(firstRun){
      visualizer.addPointCloud(boundary_cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else{
      visualizer.updatePointCloud(boundary_cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(EdgeCurveAnnotator)
