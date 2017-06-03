#include <uima/api.hpp>
#include <vector>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

#include <rs/types/all_types.h>
//RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>



using namespace uima;


class OverSegmentationAnnotator : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;

  float normalAngleThreshold;
  float curvatureThreshold;

  int minClusterSize;
  int maxClusterSize;

  int neighborNumber;

  double pointSize;

public:
  OverSegmentationAnnotator () : DrawingAnnotator(__func__), pointSize(1.0) {}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("normalAngleThreshold"))
    {
      ctx.extractValue("normalAngleThreshold", normalAngleThreshold);
    }
    if(ctx.isParameterDefined("curvatureThreshold"))
    {
      ctx.extractValue("curvatureThreshold", curvatureThreshold);
    }
    if(ctx.isParameterDefined("minClusterSize"))
    {
      ctx.extractValue("minClusterSize", minClusterSize);
    }
    if(ctx.isParameterDefined("maxClusterSize"))
    {
      ctx.extractValue("maxClusterSize", maxClusterSize);
    }
    if(ctx.isParameterDefined("neighborNumber"))
    {
      ctx.extractValue("neighborNumber", neighborNumber);
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

    //get cloud from CAS
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD,*cloud_ptr);
    outInfo("Cloud size: " << cloud_ptr->points.size());

    //get point cloud normals from CAS
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_NORMALS, *normals);
    outInfo("Normals size: " << normals->points.size());

    //discard color information
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_ptr, *cloud);


    //init search method
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());


    //update cloud for RegionGrowing instance
    rg.setMinClusterSize(minClusterSize);
    rg.setMaxClusterSize(maxClusterSize);
    rg.setNumberOfNeighbours(neighborNumber);
    rg.setSmoothnessThreshold(normalAngleThreshold);
    rg.setCurvatureThreshold(curvatureThreshold);
    rg.setSearchMethod(tree);
    rg.setInputCloud(cloud);
    rg.setInputNormals(normals);

    outInfo("Normal Threshold =  " << normalAngleThreshold);
    outInfo("Curvature Threshold =  " << curvatureThreshold);

    //container for segmentation Result
    std::vector<pcl::PointIndices> clusters;

    //main execution
    rg.extract(clusters);
    colored_cloud = rg.getColoredCloud();

    outInfo("Total clusters = " << clusters.size());

    //publish clusters to CAS
    //cas.set(VIEW_OVERSEG_CLUSTERS, *clusters);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(firstRun){
      visualizer.addPointCloud(colored_cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else{
      visualizer.updatePointCloud(colored_cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
  void drawImageWithLock(cv::Mat& disp) {}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(OverSegmentationAnnotator)
