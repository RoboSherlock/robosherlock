#include <uima/api.hpp>
#include <vector>
#include <omp.h>

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
#include <rs/types/all_types.h>

#include <rs/segmentation/array_utils.hpp>
#include <rs/segmentation/BoundarySegmentation.hpp>
#include <rs/segmentation/RotationalSymmetry.hpp>
#include <rs/segmentation/RotationalSymmetryScoring.hpp>
#include <rs/segmentation/SymmetrySegmentation.hpp>

#include <rs/occupancy_map/DistanceMap.hpp>
#include <rs/occupancy_map/DownsampleMap.hpp>

#include <rs/graph/WeightedGraph.hpp>



using namespace uima;


class RotationalSymmetrySegmentation : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr sceneNormals;

  boost::shared_ptr< DistanceMap<pcl::PointXYZRGBA> > dist_map;
  Eigen::Vector4f boundingPlane;

  WeightedGraph sceneGraph;

  std::vector<RotationalSymmetry> symmetries;
  int numSymmetries;

  std::vector< float > symmetryScores;
  std::vector< float > occlusionScores;
  std::vector< float > smoothnessScores;

  std::vector< std::vector< float > > pointSymScores;
  std::vector< std::vector< float > > pointOcclusionScores;
  std::vector< std::vector< float > > pointPerpendicularScores;

  std::vector< std::vector< float > > fgWeights;
  std::vector< std::vector< float > > bgWeights;

  //parameters
  bool isDownsampled;
  float downsample_leaf_size;

  float dist_map_resolution;

  float adjacency_radius;
  int num_adjacency_neighbors;

  float adjacency_sigma_convex;
  float adjacency_sigma_concave;
  float adjacency_weight_factor;

  float min_fit_angle;
  float max_fit_angle;
  float min_occlusion_dist;
  float max_occlusion_dist;
  float max_perpendicular_angle;

  float fg_weight_factor;
  float bg_weight_factor;

  double pointSize;

public:
  RotationalSymmetrySegmentation () : DrawingAnnotator(__func__), pointSize(1.0) {
    sceneCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    sceneNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("isDownsampled", isDownsampled);
    ctx.extractValue("downsample_leaf_size", downsample_leaf_size);
    ctx.extractValue("dist_map_resolution", dist_map_resolution);
    ctx.extractValue("adjacency_radius", adjacency_radius);
    ctx.extractValue("num_adjacency_neighbors", num_adjacency_neighbors);
    ctx.extractValue("adjacency_sigma_convex", adjacency_sigma_convex);
    ctx.extractValue("adjacency_sigma_concave", adjacency_sigma_concave);
    ctx.extractValue("adjacency_weight_factor", adjacency_weight_factor);
    ctx.extractValue("min_fit_angle", min_fit_angle);
    ctx.extractValue("max_fit_angle", max_fit_angle);
    ctx.extractValue("min_occlusion_dist", min_occlusion_dist);
    ctx.extractValue("max_occlusion_dist", max_occlusion_dist);
    ctx.extractValue("max_perpendicular_angle", max_perpendicular_angle);
    ctx.extractValue("fg_weight_factor", fg_weight_factor);
    ctx.extractValue("bg_weight_factor", bg_weight_factor);

    boundingPlane << 0.104788, -0.720677, -0.685305, 0.693016; // plane parameters from example cloud

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

    //clearing previous data
    symmetryScores.clear();
    occlusionScores.clear();
    smoothnessScores.clear();
    pointSymScores.clear();
    pointOcclusionScores.clear();
    pointPerpendicularScores.clear();
    fgWeights.clear();
    bgWeights.clear();
    symmetries.clear();

    //get RGB cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);

    //get normal cloud
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_NORMALS, *normals);

    //get Rotational Symmteries
    std::vector<rs::RotationalSymmetry> casSymmetries;
    cas.get(VIEW_ROTATIONAL_SYMMETRIES, casSymmetries);
    numSymmetries = casSymmetries.size();
    symmetries.resize(numSymmetries);

    for(size_t it = 0; it < casSymmetries.size(); it++){
      Eigen::Vector3f currOrigin(casSymmetries[it].origin().x(), casSymmetries[it].origin().y(), casSymmetries[it].origin().z());
      Eigen::Vector3f currOrientation(casSymmetries[it].orientation().x(), casSymmetries[it].orientation().y(), casSymmetries[it].orientation().z());
      symmetries[it] = RotationalSymmetry(currOrigin, currOrientation);
    }

    //discard color information
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz (new  pcl::PointCloud<pcl::PointXYZ>);
    //pcl::copyPointCloud(*cloud_ptr, *cloudxyz);

    //allocating containers
    symmetryScores.resize(numSymmetries);
    occlusionScores.resize(numSymmetries);
    smoothnessScores.resize(numSymmetries);
    pointSymScores.resize(numSymmetries);
    pointOcclusionScores.resize(numSymmetries);
    pointPerpendicularScores.resize(numSymmetries);
    fgWeights.resize(numSymmetries);
    bgWeights.resize(numSymmetries);

    //main execution

    //downsample the cloud and normal cloud to speed up segmentation
    if(isDownsampled){
      std::vector< std::vector<int> > dsMap;
      std::vector<int> nearestMap;
      DownsampleMap<pcl::PointXYZRGBA> dc;
      dc.setInputCloud(cloud_ptr);
      dc.setLeafSize(downsample_leaf_size);
      dc.filter(*sceneCloud);
      dc.getDownsampleMap(dsMap);
      dc.getNearestNeighborMap(nearestMap);

      computeDownsampleNormals(normals, dsMap, nearestMap, AVERAGE, sceneNormals);
    }
    else{
      sceneCloud = cloud_ptr;
      sceneNormals = normals;
    }

    //initialize distance map
    std::vector<Eigen::Vector4f> planes;
    planes.push_back(boundingPlane);

    dist_map = boost::shared_ptr< DistanceMap< pcl::PointXYZRGBA > >(new DistanceMap <pcl::PointXYZRGBA> (dist_map_resolution));
    dist_map->setBoundingPlanes(planes);
    dist_map->setInputCloud(sceneCloud);

    //compute adjacency weigth for smoothness term
    if(!computeCloudAdjacencyWeight<pcl::PointXYZRGBA>(sceneCloud, sceneNormals, adjacency_radius, num_adjacency_neighbors, sceneGraph, adjacency_weight_factor))
      outError("Could not construct adjacency graph!");

    #pragma omp parallel for
    for(size_t symId = 0; symId < numSymmetries; symId++){
      //compute point scores for each symmetry
      getCloudSymmetryScore<pcl::PointXYZRGBA>(sceneCloud, sceneNormals, symmetries[symId], pointSymScores[symId], min_fit_angle, max_fit_angle);
      getCloudOcclusionScore<pcl::PointXYZRGBA>(sceneCloud, *dist_map, symmetries[symId], pointOcclusionScores[symId], min_occlusion_dist, max_occlusion_dist);
      getCloudPerpendicularScore(sceneNormals, symmetries[symId], pointPerpendicularScores[symId], max_perpendicular_angle);

      //compute unary weight from scores
      fgWeights[symId].resize(sceneCloud->points.size());
      bgWeights[symId].resize(sceneCloud->points.size());

      for(size_t pId = 0; pId < sceneCloud->points.size(); pId++){
        fgWeights[symId][pId] = (1.0f - pointSymScores[symId][pId]) * (1.0f - pointOcclusionScores[symId][pId]) * (1.0f - pointPerpendicularScores[symId][pId]) * fg_weight_factor;

        bgWeights[symId][pId] = (pointSymScores[symId][pId] * (1.0f - pointPerpendicularScores[symId][pId]) + pointOcclusionScores[symId][pId]) * bg_weight_factor;
      }
    }

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    const std::string normalsname = this->name + "_normals";

    if(firstRun){
      visualizer.addPointCloud(sceneCloud, cloudname);
      visualizer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(sceneCloud, sceneNormals, 50, 0.02f, normalsname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, normalsname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else{
      visualizer.updatePointCloud(sceneCloud, cloudname);
      visualizer.removePointCloud(normalsname);
      visualizer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(sceneCloud, sceneNormals, 50, 0.02f, normalsname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, normalsname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RotationalSymmetrySegmentation)
