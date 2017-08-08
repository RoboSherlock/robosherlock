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
  std::vector<Eigen::Vector4f> boundingPlanes;

  WeightedGraph sceneGraph;

  std::vector<RotationalSymmetry> symmetries;
  std::vector<RotationalSymmetry> finalSymmetries;
  int numSymmetries;

  std::vector< float > symmetryScores;
  std::vector< float > occlusionScores;
  std::vector< float > cutScores;

  std::vector< std::vector< float > > pointSymScores;
  std::vector< std::vector< float > > pointOcclusionScores;
  std::vector< std::vector< float > > pointPerpendicularScores;

  std::vector< std::vector<int> > dsMap;

  std::vector< std::vector< float > > fgWeights;
  std::vector< std::vector< float > > bgWeights;

  std::vector< std::vector<int> > dsSegmentIds;
  std::vector< std::vector<int> > segmentIds;

  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segments;
  std::vector<int> filteredSegmentIds;

  //parameters
  bool isDownsampled;
  float downsample_leaf_size;

  float dist_map_resolution;

  float rotSymSeg_adjacency_radius;
  int rotSymSeg_num_adjacency_neighbors;

  float adjacency_sigma_convex;
  float adjacency_sigma_concave;
  float rotSymSeg_adjacency_weight_factor;

  float rotSymSeg_min_fit_angle;
  float rotSymSeg_max_fit_angle;
  float rotSymSeg_min_occlusion_dist;
  float rotSymSeg_max_occlusion_dist;
  float rotSymSeg_max_perpendicular_angle;

  float rotSymSeg_fg_weight_factor;
  float rotSymSeg_bg_weight_factor;

  float rotSymSeg_max_sym_score;
  float rotSymSeg_max_occlusion_score;
  float rotSymSeg_max_cut_score;
  int min_segment_size;

  double pointSize;
  int segVisIt;

  std::mutex sym_mutex;

public:
  RotationalSymmetrySegmentation () : DrawingAnnotator(__func__), pointSize(1.0), segVisIt(0) {
    sceneCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    sceneNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("isDownsampled", isDownsampled);
    ctx.extractValue("downsample_leaf_size", downsample_leaf_size);
    ctx.extractValue("dist_map_resolution", dist_map_resolution);
    ctx.extractValue("rotSymSeg_adjacency_radius", rotSymSeg_adjacency_radius);
    ctx.extractValue("rotSymSeg_num_adjacency_neighbors", rotSymSeg_num_adjacency_neighbors);
    ctx.extractValue("adjacency_sigma_convex", adjacency_sigma_convex);
    ctx.extractValue("adjacency_sigma_concave", adjacency_sigma_concave);
    ctx.extractValue("rotSymSeg_adjacency_weight_factor", rotSymSeg_adjacency_weight_factor);
    ctx.extractValue("rotSymSeg_min_fit_angle", rotSymSeg_min_fit_angle);
    ctx.extractValue("rotSymSeg_max_fit_angle", rotSymSeg_max_fit_angle);
    ctx.extractValue("rotSymSeg_min_occlusion_dist", rotSymSeg_min_occlusion_dist);
    ctx.extractValue("rotSymSeg_max_occlusion_dist", rotSymSeg_max_occlusion_dist);
    ctx.extractValue("rotSymSeg_max_perpendicular_angle", rotSymSeg_max_perpendicular_angle);
    ctx.extractValue("rotSymSeg_fg_weight_factor", rotSymSeg_fg_weight_factor);
    ctx.extractValue("rotSymSeg_bg_weight_factor", rotSymSeg_bg_weight_factor);
    ctx.extractValue("rotSymSeg_max_sym_score", rotSymSeg_max_sym_score);
    ctx.extractValue("rotSymSeg_max_occlusion_score", rotSymSeg_max_occlusion_score);
    ctx.extractValue("rotSymSeg_max_cut_score", rotSymSeg_max_cut_score);
    ctx.extractValue("min_segment_size", min_segment_size);

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
    rs::Scene scene = cas.getScene();

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
    boundingPlanes.clear();

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

    if(numSymmetries < 1){
      outWarn("No input rotational symmteries! Segmentation abort!");
      return UIMA_ERR_NONE;
    }


    symmetries.resize(numSymmetries);

    for(size_t it = 0; it < casSymmetries.size(); it++){
      Eigen::Vector3f currOrigin(casSymmetries[it].origin().x(), casSymmetries[it].origin().y(), casSymmetries[it].origin().z());
      Eigen::Vector3f currOrientation(casSymmetries[it].orientation().x(), casSymmetries[it].orientation().y(), casSymmetries[it].orientation().z());
      symmetries[it] = RotationalSymmetry(currOrigin, currOrientation);
    }

    //allocating containers
    symmetryScores.resize(numSymmetries);
    occlusionScores.resize(numSymmetries);
    cutScores.resize(numSymmetries);
    pointSymScores.resize(numSymmetries);
    pointOcclusionScores.resize(numSymmetries);
    pointPerpendicularScores.resize(numSymmetries);
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

    //get bounding planes
    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);
    boundingPlanes.resize(planes.size());
    if(planes.empty())
    {
      outWarn("Planes are not found! Using default plane z=0");
      boundingPlanes.push_back(Eigen::Vector4f::UnitZ());
    }
    else
    {
      for(size_t planeId = 0; planeId < planes.size(); planeId++)
      {
        boundingPlanes[planeId] = Eigen::Vector4f(planes[planeId].model()[0], planes[planeId].model()[1], planes[planeId].model()[2], planes[planeId].model()[3]);
      }
    }

    //initialize distance map
    dist_map = boost::shared_ptr< DistanceMap< pcl::PointXYZRGBA > >(new DistanceMap <pcl::PointXYZRGBA> (dist_map_resolution));
    dist_map->setBoundingPlanes(boundingPlanes);
    dist_map->setInputCloud(sceneCloud);

    //compute adjacency weigth for smoothness term
    if(!computeCloudAdjacencyWeight<pcl::PointXYZRGBA>(sceneCloud, sceneNormals, rotSymSeg_adjacency_radius, rotSymSeg_num_adjacency_neighbors, sceneGraph, rotSymSeg_adjacency_weight_factor)){
      outWarn("Could not construct adjacency graph!");
      return UIMA_ERR_NONE;
    }


    #pragma omp parallel for
    for(size_t symId = 0; symId < numSymmetries; symId++){
      //compute point scores for each symmetry
      getCloudSymmetryScore<pcl::PointXYZRGBA>(sceneCloud, sceneNormals, symmetries[symId], pointSymScores[symId], rotSymSeg_min_fit_angle, rotSymSeg_max_fit_angle);
      getCloudOcclusionScore<pcl::PointXYZRGBA>(sceneCloud, *dist_map, symmetries[symId], pointOcclusionScores[symId], rotSymSeg_min_occlusion_dist, rotSymSeg_max_occlusion_dist);
      getCloudPerpendicularScore(sceneNormals, symmetries[symId], pointPerpendicularScores[symId], rotSymSeg_max_perpendicular_angle);

      //compute unary weight from scores
      fgWeights[symId].resize(sceneCloud->points.size());
      bgWeights[symId].resize(sceneCloud->points.size());

      for(size_t pId = 0; pId < sceneCloud->points.size(); pId++){
        fgWeights[symId][pId] = (1.0f - pointSymScores[symId][pId]) * (1.0f - pointOcclusionScores[symId][pId]) * (1.0f - pointPerpendicularScores[symId][pId]) * rotSymSeg_fg_weight_factor;

        bgWeights[symId][pId] = (pointSymScores[symId][pId] * (1.0f - pointPerpendicularScores[symId][pId]) + pointOcclusionScores[symId][pId]) * rotSymSeg_bg_weight_factor;
      }

      //sym_mutex.lock();

      std::vector<int> backgroundIds;
      float min_cut_value;
      float max_flow = BoykovMinCut::min_cut(fgWeights[symId], bgWeights[symId], sceneGraph, dsSegmentIds[symId], backgroundIds, min_cut_value);

      if(max_flow < 0.0f){
        outWarn("Could not segment cloud using Boykov min_cut! abort!");
      }

      //compute segment score for filtering
      symmetryScores[symId] = 0.0f;
      occlusionScores[symId] = 0.0f;
      cutScores[symId] = 0.0f;
      if(dsSegmentIds[symId].size() > min_segment_size){
        std::vector<int> boundaryIds, nonBoundaryIds;
        extractBoundaryCloud<pcl::PointXYZRGBA, pcl::Normal>(sceneCloud, sceneNormals, dsSegmentIds[symId], boundaryIds, nonBoundaryIds);

        for(size_t pointIdIt = 0; pointIdIt < nonBoundaryIds.size(); pointIdIt++){
          int pointId = nonBoundaryIds[pointIdIt];
          symmetryScores[symId] += pointSymScores[symId][pointId];
        }
        symmetryScores[symId] /= static_cast<float>(nonBoundaryIds.size());

        for(size_t pointIdIt = 0; pointIdIt < dsSegmentIds[symId].size(); pointIdIt++){
          int pointId = dsSegmentIds[symId][pointIdIt];
          occlusionScores[symId] += pointOcclusionScores[symId][pointId];
        }
        occlusionScores[symId] /= static_cast<float>(dsSegmentIds[symId].size());

        if(dsSegmentIds[symId].size() != sceneCloud->size())
        {
          cutScores[symId] = min_cut_value / static_cast<float>(dsSegmentIds[symId].size());
        }
      }

      if(isDownsampled)
        upsample_cloud(dsSegmentIds[symId], dsMap, segmentIds[symId]);
      else
        segmentIds = dsSegmentIds;

      //sym_mutex.unlock();
    }

    //filtering
    this->filter();

    //extract good segment for visualizer and publish to CAS
    std::vector<pcl::PointIndices> casSegments;
    for(size_t segmentIdIt = 0; segmentIdIt < filteredSegmentIds.size(); segmentIdIt++){
      int segmentId = filteredSegmentIds[segmentIdIt];
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currSegment(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::copyPointCloud(*cloud_ptr, segmentIds[segmentId], *currSegment);
      segments.push_back(currSegment);
      finalSymmetries.push_back(symmetries[segmentId]);

      pcl::PointIndices currSegmentIds;
      currSegmentIds.indices = segmentIds[segmentId];
      casSegments.push_back(currSegmentIds);
    }

    cas.set(VIEW_ROTATIONAL_SEGMENTATION_IDS, casSegments);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(numSymmetries > 0){
      if(firstRun){
        visualizer.addPointCloud(segments[segVisIt], cloudname);
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
        addSymmetryLine(visualizer, finalSymmetries, 0.4f, 0.8f);
        visualizer.addText("Segment " + std::to_string(segVisIt+1) + " / " + std::to_string(segments.size()), 15, 125, 24, 1.0, 1.0, 1.0);
      }
      else{
        visualizer.removeAllShapes();
        visualizer.updatePointCloud(segments[segVisIt], cloudname);
        visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
        addSymmetryLine(visualizer, finalSymmetries, 0.4f, 0.8f);
        visualizer.addText("Segment " + std::to_string(segVisIt+1) + " / " + std::to_string(segments.size()), 15, 125, 24, 1.0, 1.0, 1.0);
      }
    }
  }

private:

  void addSymmetryLine(pcl::visualization::PCLVisualizer& visualizer, std::vector<RotationalSymmetry>& symmetries, float length, float lineWidth){
    for(size_t symId = 0; symId < symmetries.size(); symId++){
      pcl::PointXYZ p1, p2;
      p1.getVector3fMap() = symmetries[symId].getOrigin() + symmetries[symId].getOrientation() * length / 2;
      p2.getVector3fMap() = symmetries[symId].getOrigin() - symmetries[symId].getOrientation() * length / 2;

      std::string id = "sym" + std::to_string(symId);
      visualizer.addLine(p1, p2, id);
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, id);
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, id);
    }
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case 'a':
      segVisIt--;
      if(segVisIt < 0)
        segVisIt = segments.size() - 1;
      break;
    case 'd':
      segVisIt++;
      if(segVisIt >= segments.size())
        segVisIt = 0;
      break;
    default:
      segVisIt = 0;
      break;
    }
    return true;
  }

  inline void filter(){

    for(size_t symId = 0; symId < numSymmetries; symId++){
      if( symmetryScores[symId] < rotSymSeg_max_sym_score &&
          occlusionScores[symId] < rotSymSeg_max_occlusion_score &&
          cutScores[symId] < rotSymSeg_max_cut_score &&
          dsSegmentIds[symId].size() > min_segment_size)
      {
        filteredSegmentIds.push_back(symId);
      }
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RotationalSymmetrySegmentation)
