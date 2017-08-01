#include <uima/api.hpp>
#include <vector>
#include <omp.h>
#include <mutex>

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
#include <rs/segmentation/BilateralSymmetry.hpp>
#include <rs/segmentation/BilateralSymmetryScoring.hpp>
#include <rs/segmentation/SymmetrySegmentation.hpp>

#include <rs/occupancy_map/DistanceMap.hpp>
#include <rs/occupancy_map/DownsampleMap.hpp>

#include <rs/graph/WeightedGraph.hpp>



using namespace uima;


class BilateralSymmetrySegmentation : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr sceneNormals;

  boost::shared_ptr< DistanceMap<pcl::PointXYZRGBA> > dist_map;
  Eigen::Vector4f boundingPlane;

  WeightedGraph sceneGraph;

  std::vector<BilateralSymmetry> symmetries;
  std::vector<BilateralSymmetry> finalSymmetries;
  int numSymmetries;

  std::vector< std::vector<int> > dsMap;

  std::vector< float > symmetryScores;
  std::vector< float > occlusionScores;
  std::vector< float > cutScores;

  std::vector< std::vector< float > > pointSymScores;
  std::vector< std::vector< float > > pointOcclusionScores;
  std::vector< std::vector< float > > pointPerpendicularScores;

  std::vector< std::vector< float > > fgWeights;
  std::vector< std::vector< float > > bgWeights;

  std::vector< std::vector<int> > dsSegmentIds;
  std::vector< std::vector<int> > segmentIds;

  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segments;
  std::vector<int> filteredSegmentIds;

  //parameters
  bool isDownsampled;
  float downsample_voxel_size;

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
  float min_perpendicular_angle;
  float max_perpendicular_angle;
  float correspondence_max_sym_reflected_dist;

  float fg_weight_factor;
  float bg_weight_factor;

  float max_sym_score;
  float max_occlusion_score;
  float max_cut_score;
  float min_sym_sypport_overlap;
  int min_segment_size;

  float overlap_threshold;

  std::mutex sym_mutex;

  double pointSize;
  int segVisIt;

public:
  BilateralSymmetrySegmentation () : DrawingAnnotator(__func__), pointSize(1.0), segVisIt(0){
    sceneCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    sceneNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("isDownsampled", isDownsampled);
    ctx.extractValue("downsample_voxel_size", downsample_voxel_size);
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
    ctx.extractValue("min_perpendicular_angle", min_perpendicular_angle);
    ctx.extractValue("max_perpendicular_angle", max_perpendicular_angle);
    ctx.extractValue("correspondence_max_sym_reflected_dist", correspondence_max_sym_reflected_dist);

    ctx.extractValue("fg_weight_factor", fg_weight_factor);
    ctx.extractValue("bg_weight_factor", bg_weight_factor);

    ctx.extractValue("max_sym_score", max_sym_score);
    ctx.extractValue("max_occlusion_score", max_occlusion_score);
    ctx.extractValue("max_cut_score", max_cut_score);
    ctx.extractValue("min_sym_sypport_overlap", min_sym_sypport_overlap);
    ctx.extractValue("min_segment_size", min_segment_size);

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
    cutScores.clear();
    pointSymScores.clear();
    pointOcclusionScores.clear();
    pointPerpendicularScores.clear();
    dsMap.clear();
    fgWeights.clear();
    bgWeights.clear();
    symmetries.clear();
    finalSymmetries.clear();
    segmentIds.clear();
    dsSegmentIds.clear();
    segments.clear();
    filteredSegmentIds.clear();

    //get RGB cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);

    //get normal cloud
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_NORMALS, *normals);

    //get Rotational Symmteries
    std::vector<rs::BilateralSymmetry> casSymmetries;
    cas.get(VIEW_BILATERAL_SYMMETRIES, casSymmetries);
    numSymmetries = casSymmetries.size();

    if(numSymmetries < 1){
      outWarn("No input bilateral symmteries! Segmentation abort!");
      return UIMA_ERR_NONE;
    }


    symmetries.resize(numSymmetries);

    for(size_t it = 0; it < casSymmetries.size(); it++){
      Eigen::Vector3f currOrigin(casSymmetries[it].origin().x(), casSymmetries[it].origin().y(), casSymmetries[it].origin().z());
      Eigen::Vector3f currNormal(casSymmetries[it].normal().x(), casSymmetries[it].normal().y(), casSymmetries[it].normal().z());
      symmetries[it] = BilateralSymmetry(currOrigin, currNormal);
    }

    //discard color information
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz (new  pcl::PointCloud<pcl::PointXYZ>);
    //pcl::copyPointCloud(*cloud_ptr, *cloudxyz);

    //allocating containers
    fgWeights.resize(numSymmetries);
    bgWeights.resize(numSymmetries);
    segmentIds.resize(numSymmetries);
    dsSegmentIds.resize(numSymmetries);

    //main execution

    //downsample the cloud and normal cloud to speed up segmentation
    if(isDownsampled){
      std::vector<int> nearestMap;
      DownsampleMap<pcl::PointXYZRGBA> dc;
      dc.setInputCloud(cloud_ptr);
      dc.setLeafSize(downsample_voxel_size);
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

    //main execution

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(numSymmetries > 0){
      if(firstRun){
        visualizer.addPointCloud(sceneCloud, cloudname);
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
        addSymmetryPlanes(visualizer, symmetries, 0.05f, 0.05f);
        //visualizer.addText("Segment " + std::to_string(segVisIt+1) + " / " + std::to_string(segments.size()), 15, 125, 24, 1.0, 1.0, 1.0);
      }
      else{
        visualizer.removeAllShapes();
        visualizer.updatePointCloud(sceneCloud, cloudname);
        visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
        addSymmetryPlanes(visualizer, symmetries, 0.05f, 0.05f);
        //visualizer.addText("Segment " + std::to_string(segVisIt+1) + " / " + std::to_string(segments.size()), 15, 125, 24, 1.0, 1.0, 1.0);
      }
    }
  }

private:

  inline void addSymmetryPlane(pcl::visualization::PCLVisualizer &visualizer, BilateralSymmetry &symmetry, std::string &id, float width, float height)
  {
    Eigen::Affine3f pose;
    pose.translation() = symmetry.getOrigin();
    pose.linear() = getAlignMatrix<float>(Eigen::Vector3f::UnitZ(), symmetry.getNormal());

    float halfWidth = width / 2.0f;
    float halfHeight = height / 2.0f;

    pcl::PointCloud<pcl::PointXYZ>::Ptr rect(new pcl::PointCloud<pcl::PointXYZ>);
    rect->resize(4);
    rect->points[0] = pcl::PointXYZ(-halfWidth, -halfHeight, 0);
    rect->points[1] = pcl::PointXYZ(halfWidth, -halfHeight, 0);
    rect->points[2] = pcl::PointXYZ(halfWidth, halfHeight, 0);
    rect->points[3] = pcl::PointXYZ(-halfWidth, halfHeight, 0);

    pcl::transformPointCloud<pcl::PointXYZ>(*rect, *rect, pose);

    visualizer.addPolygon<pcl::PointXYZ>(rect, id);
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, id);
    visualizer.addPolygon<pcl::PointXYZ>(rect, id + "_border");
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id + "_border");
  }

  inline void addSymmetryPlanes(pcl::visualization::PCLVisualizer &visualizer, std::vector< std::vector<BilateralSymmetry> > &symmetries, float width, float height)
  {
    for(size_t segmentId = 0; segmentId < symmetries.size(); segmentId++)
    {
      for(size_t symId = 0; symId < symmetries[segmentId].size(); symId++)
      {
        std::string id = "BilSym" + std::to_string(segmentId * symmetries[segmentId].size() + symId);
        addSymmetryPlane(visualizer, symmetries[segmentId][symId], id, width, height);
      }
    }
  }

  inline void addSymmetryPlanes(pcl::visualization::PCLVisualizer &visualizer, std::vector<BilateralSymmetry> &symmetries, float width, float height)
  {
    for(size_t symId = 0; symId < symmetries.size(); symId++)
    {
      std::string id = "BilSym" + std::to_string(symId);
      addSymmetryPlane(visualizer, symmetries[symId], id, width, height);
    }
  }

  /*bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case 'a':
      segVisIt--;
      if(segVisIt < 0)
        segVisIt = numSymmetries - 1;
      break;
    case 'd':
      segVisIt++;
      if(segVisIt >= numSymmetries)
        segVisIt = 0;
      break;
    default:
      segVisIt = 0;
      break;
    }
    return true;
  }*/
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BilateralSymmetrySegmentation)
