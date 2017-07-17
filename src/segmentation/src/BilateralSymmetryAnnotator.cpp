#include <uima/api.hpp>
#include <vector>
#include <omp.h>

//PCL include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/types/all_types.h>

#include <rs/segmentation/array_utils.hpp>
#include <rs/segmentation/RotationalSymmetry.hpp>
#include <rs/segmentation/RotationalSymmetryScoring.hpp>
#include <rs/segmentation/BoundarySegmentation.hpp>
#include <rs/occupancy_map/DistanceMap.hpp>
#include <rs/NonLinearOptimization/Functor.hpp>
#include <rs/graph/Graph.hpp>
#include <rs/graph/GraphAlgorithms.hpp>

using namespace uima;

class BilateralSymmetryAnnotator : public DrawingAnnotator
{
private:
  //container for inital symmetries usign PCA solver
  std::vector< std::vector<RotationalSymmetry> > segmentInitialSymmetries;
  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segmentClouds;
  std::vector< std::vector<float> > symSupportSizes;
  std::vector< pcl::PointCloud<pcl::Normal>::Ptr > segmentNormals;
  std::vector<pcl::PointIndices> segments;
  std::vector<Eigen::Vector3f> segment_centroids;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  int numSegments;

  double pointSize;

public:
  BilateralSymmetryAnnotator () : DrawingAnnotator(__func__), pointSize(1.0) {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

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
    cloud = cloud_ptr;

    //get normal cloud
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_NORMALS, *normals);

    //get segments
    cas.get(VIEW_SEGMENT_IDS, segments);

    //clearing for consecutive frame
    segmentInitialSymmetries.clear();
    segmentClouds.clear();
    symSupportSizes.clear();
    segmentNormals.clear();
    segment_centroids.clear();

    //allocating containers
    numSegments = segments.size();
    segmentInitialSymmetries.resize(numSegments);
    segmentClouds.resize(numSegments);
    symSupportSizes.resize(numSegments);
    segmentNormals.resize(numSegments);
    segment_centroids.resize(numSegments);


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
MAKE_AE(BilateralSymmetryAnnotator)
