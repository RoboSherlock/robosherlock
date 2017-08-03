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
#include <rs/graph/Graph.hpp>
#include <rs/graph/GraphAlgorithms.hpp>



using namespace uima;


class BilateralSymmetrySegmentation : public DrawingAnnotator
{
private:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr sceneNormals;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dsSceneCloud;
  pcl::PointCloud<pcl::Normal>::Ptr dsSceneNormals;

  boost::shared_ptr< DistanceMap<pcl::PointXYZRGBA> > dist_map;
  Eigen::Vector4f boundingPlane;

  WeightedGraph sceneGraph;
  std::vector<WeightedGraph> symmetricGraph;

  std::vector<pcl::Correspondences> correspondences;

  std::vector<BilateralSymmetry> symmetries;
  std::vector< std::vector<int> > symmetrySupports;
  std::vector<BilateralSymmetry> finalSymmetries;
  int numSymmetries;

  std::vector< std::vector<int> > dsMap;
  std::vector<int> reversedMap;

  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr search_tree;

  std::vector< float > symmetryScores;
  std::vector< float > occlusionScores;
  std::vector< float > cutScores;
  std::vector< float > symmetrySupportOverlapScores;

  std::vector< std::vector< float > > pointSymScores;
  std::vector< std::vector< float > > pointOcclusionScores;
  std::vector< std::vector< float > > pointPerpendicularScores;

  std::vector< std::vector< float > > fgWeights;
  std::vector< std::vector< float > > bgWeights;

  std::vector< std::vector<int> > dsSegmentIds;
  std::vector< std::vector<int> > segmentIds;

  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segments;
  std::vector<int> filteredSegmentIds;
  std::vector<int> mergedSymmetryIds;

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

  float symmetric_weight_factor;
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
    dsSceneCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    dsSceneNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
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

    ctx.extractValue("symmetric_weight_factor", symmetric_weight_factor);
    ctx.extractValue("fg_weight_factor", fg_weight_factor);
    ctx.extractValue("bg_weight_factor", bg_weight_factor);

    ctx.extractValue("max_sym_score", max_sym_score);
    ctx.extractValue("max_occlusion_score", max_occlusion_score);
    ctx.extractValue("max_cut_score", max_cut_score);
    ctx.extractValue("min_sym_sypport_overlap", min_sym_sypport_overlap);
    ctx.extractValue("min_segment_size", min_segment_size);

    ctx.extractValue("overlap_threshold", overlap_threshold);

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
    symmetries.clear();
    symmetrySupports.clear();
    correspondences.clear();
    symmetricGraph.clear();
    symmetryScores.clear();
    occlusionScores.clear();
    cutScores.clear();
    pointSymScores.clear();
    pointOcclusionScores.clear();
    pointPerpendicularScores.clear();
    symmetrySupportOverlapScores.clear();
    dsMap.clear();
    reversedMap.clear();
    fgWeights.clear();
    bgWeights.clear();
    symmetries.clear();
    finalSymmetries.clear();
    segmentIds.clear();
    dsSegmentIds.clear();
    segments.clear();
    filteredSegmentIds.clear();
    mergedSymmetryIds.clear();

    //get RGB cloud
    cas.get(VIEW_CLOUD, *sceneCloud);

    //get normal cloud
    cas.get(VIEW_NORMALS, *sceneNormals);

    //get Rotational Symmteries
    std::vector<rs::BilateralSymmetry> casSymmetries;
    cas.get(VIEW_BILATERAL_SYMMETRIES, casSymmetries);
    numSymmetries = casSymmetries.size();

    if(numSymmetries < 1){
      outWarn("No input bilateral symmteries! Segmentation abort!");
      return UIMA_ERR_NONE;
    }


    symmetries.resize(numSymmetries);
    symmetrySupports.resize(numSymmetries);
    for(size_t it = 0; it < casSymmetries.size(); it++){
      Eigen::Vector3f currOrigin(casSymmetries[it].origin().x(), casSymmetries[it].origin().y(), casSymmetries[it].origin().z());
      Eigen::Vector3f currNormal(casSymmetries[it].normal().x(), casSymmetries[it].normal().y(), casSymmetries[it].normal().z());
      symmetries[it] = BilateralSymmetry(currOrigin, currNormal);
      symmetrySupports[it] = casSymmetries[it].support();
    }

    //discard color information
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz (new  pcl::PointCloud<pcl::PointXYZ>);
    //pcl::copyPointCloud(*cloud_ptr, *cloudxyz);

    //allocating containers
    correspondences.resize(numSymmetries);
    symmetricGraph.resize(numSymmetries);
    symmetryScores.resize(numSymmetries);
    occlusionScores.resize(numSymmetries);
    cutScores.resize(numSymmetries);
    symmetrySupportOverlapScores.resize(numSymmetries);
    pointSymScores.resize(numSymmetries);
    pointOcclusionScores.resize(numSymmetries);
    pointPerpendicularScores.resize(numSymmetries);
    fgWeights.resize(numSymmetries);
    bgWeights.resize(numSymmetries);
    segmentIds.resize(numSymmetries);
    dsSegmentIds.resize(numSymmetries);

    //downsample the cloud and normal cloud to speed up segmentation
    if(isDownsampled){
      std::vector<int> nearestMap;
      DownsampleMap<pcl::PointXYZRGBA> dc;
      dc.setInputCloud(sceneCloud);
      dc.setLeafSize(downsample_voxel_size);
      dc.filter(*dsSceneCloud);
      dc.getDownsampleMap(dsMap);
      dc.getNearestNeighborMap(nearestMap);
      dc.getReversedMap(reversedMap);

      computeDownsampleNormals(sceneNormals, dsMap, nearestMap, AVERAGE, dsSceneNormals);
    }
    else{
      //dsSceneCloud = sceneCloud;
      //dsSceneNormals = sceneNormals;
      pcl::copyPointCloud(*sceneCloud, *dsSceneCloud);
      pcl::copyPointCloud(*sceneNormals, *dsSceneNormals);
    }

    //initialize distance map
    std::vector<Eigen::Vector4f> planes;
    planes.push_back(boundingPlane);

    dist_map = boost::shared_ptr< DistanceMap< pcl::PointXYZRGBA > >(new DistanceMap <pcl::PointXYZRGBA> (dist_map_resolution));
    dist_map->setBoundingPlanes(planes);
    dist_map->setInputCloud(sceneCloud);

    //main execution
    //compute adjacency weigth for smoothness term
    if(!computeCloudAdjacencyWeight<pcl::PointXYZRGBA>(dsSceneCloud, dsSceneNormals, adjacency_radius, num_adjacency_neighbors, sceneGraph, adjacency_weight_factor))
    {
      outWarn("Could not construct adjacency graph!");
      return UIMA_ERR_NONE;
    }

    //initialize search_tree
    search_tree.reset(new pcl::search::KdTree<pcl::PointXYZRGBA>());
    search_tree->setInputCloud(sceneCloud);

    #pragma omp parallel for
    for(size_t symId = 0; symId < numSymmetries; symId++)
    {
      // setup mask for faster interation
      std::vector<bool> supportMask(dsSceneCloud->size(), false);
      for(size_t pointIdIt = 0; pointIdIt < symmetrySupports[symId].size(); pointIdIt++)
      {
        if(isDownsampled)
        {
          int dsPointId = reversedMap[symmetrySupports[symId][pointIdIt]];
          supportMask[dsPointId] = true;
        }
        else
        {
          supportMask[symmetrySupports[symId][pointIdIt]] = true;
        }
      }

      //compute score
      getCloudBilateralSymmetryScore<pcl::PointXYZRGBA>(sceneCloud, sceneNormals, dsSceneCloud, dsSceneNormals, search_tree, symmetries[symId], correspondences[symId], pointSymScores[symId], 0.01f, 0.174f, 0.02f, correspondence_max_sym_reflected_dist, min_fit_angle, max_fit_angle);
      getCloudBilateralOcclusionScore<pcl::PointXYZRGBA>(dsSceneCloud, *dist_map, symmetries[symId], pointOcclusionScores[symId], min_occlusion_dist, max_occlusion_dist);
      getCloudBilateralPerpendicularScore(dsSceneNormals, symmetries[symId], pointPerpendicularScores[symId], min_perpendicular_angle, max_perpendicular_angle);

      //compute unary weight from scores
      fgWeights[symId].resize(dsSceneCloud->points.size());
      bgWeights[symId].resize(dsSceneCloud->points.size());

      for(size_t pId = 0; pId < dsSceneCloud->points.size(); pId++)
      {
        bgWeights[symId][pId] = pointOcclusionScores[symId][pId] * bg_weight_factor;
      }

      for(size_t corresId = 0; corresId < correspondences[symId].size(); corresId++)
      {
        int pointId = correspondences[symId][corresId].index_query;
        fgWeights[symId][pointId] = (1.0f - pointSymScores[symId][corresId]) * fg_weight_factor;
        if(!supportMask[pointId])
        {
          fgWeights[symId][pointId] *= (1.0f - pointPerpendicularScores[symId][pointId]);
        }
      }

      //compute symmetric weight
      symmetricGraph[symId] = sceneGraph;
      for(size_t corresId = 0; corresId < correspondences[symId].size(); corresId++)
      {
        if(correspondences[symId][corresId].distance < 0.0f || pointSymScores[symId][corresId] >= 1.0f)
        {
          continue;
        }

        int srcPointId, tgtPointId;
        srcPointId = correspondences[symId][corresId].index_query;
        if(isDownsampled)
        {
          //convert to downsample ID
          tgtPointId = reversedMap[correspondences[symId][corresId].index_match];
        }
        else
        {
          tgtPointId = correspondences[symId][corresId].index_match;
        }

        if(srcPointId != tgtPointId)
        {
          // need checking: (1.0f - pointSymScores[symId][corresId]) * symmetric_weight_factor or just symmetric_weight_factor;
          symmetricGraph[symId].addEdge(srcPointId, tgtPointId, symmetric_weight_factor * adjacency_weight_factor); // * (1.0f - pointSymScores[symId][corresId])
        }
      }

      std::vector<int> backgroundIds;
      float min_cut_value;
      float max_flow = BoykovMinCut::min_cut(fgWeights[symId], bgWeights[symId], symmetricGraph[symId], dsSegmentIds[symId], backgroundIds, min_cut_value);

      if(max_flow < 0.0f)
      {
        outWarn("Could not segment cloud using Boykov min_cut! abort!");
      }

      //compute segment score for filtering
      symmetryScores[symId] = 0.0f;
      occlusionScores[symId] = 0.0f;
      cutScores[symId] = 0.0f;
      symmetrySupportOverlapScores[symId] = 0.0f;

      if(dsSegmentIds[symId].size() > min_segment_size)
      {
        std::vector<bool> dsSegmentMask(dsSceneCloud->size(), false);
        for(size_t pointIdIt = 0; pointIdIt < dsSegmentIds[symId].size(); pointIdIt++)
        {
          dsSegmentMask[dsSegmentIds[symId][pointIdIt]] = true;
        }

        //compute symmetry scores
        int numInlier = 0;
        for(size_t corresId = 0; corresId < correspondences[symId].size(); corresId++)
        {
          int srcPointId = correspondences[symId][corresId].index_query;
          int tgtPointId = correspondences[symId][corresId].index_match;
          if(dsSegmentMask[srcPointId] && dsSegmentMask[tgtPointId])
          {
            numInlier++;
            symmetryScores[symId] += pointSymScores[symId][corresId];
          }
        }

        if(numInlier > 0)
        {
          symmetryScores[symId] /= static_cast<float>(numInlier);
        }
        else
        {
          symmetryScores[symId] = 1.0f;
        }

        //compute occlusion score
        for(size_t pointIdIt = 0; pointIdIt < dsSegmentIds[symId].size(); pointIdIt++){
          int pointId = dsSegmentIds[symId][pointIdIt];
          occlusionScores[symId] += pointOcclusionScores[symId][pointId];
        }
        occlusionScores[symId] /= static_cast<float>(dsSegmentIds[symId].size());

        //compute cut score
        if(dsSegmentIds[symId].size() != dsSceneCloud->size())
        {
          cutScores[symId] = min_cut_value / static_cast<float>(dsSegmentIds[symId].size());
        }

        //compute overlap score
        for(size_t pointIdIt = 0; pointIdIt < dsSegmentIds[symId].size(); pointIdIt++)
        {
          int pointId = dsSegmentIds[symId][pointIdIt];
          if(supportMask[pointId])
          {
            symmetrySupportOverlapScores[symId] += 1.0f;
          }
        }
        symmetrySupportOverlapScores[symId] /= static_cast<float>(symmetrySupports[symId].size());
      }

      //if downsampled, upsample the cloud
      if(isDownsampled)
      {
        upsample_cloud(dsSegmentIds[symId], dsMap, segmentIds[symId]);
      }
      else
      {
        segmentIds[symId] = dsSegmentIds[symId];
      }
    }
    //filter segments
    this->filter();
    this->merge();

    //extract good segment for visualizer and publish to CAS
    std::vector<pcl::PointIndices> casSegments;
    for(size_t segmentIdIt = 0; segmentIdIt < mergedSymmetryIds.size(); segmentIdIt++){
      int segmentId = mergedSymmetryIds[segmentIdIt];
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currSegment(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::copyPointCloud(*sceneCloud, segmentIds[segmentId], *currSegment);
      segments.push_back(currSegment);
      finalSymmetries.push_back(symmetries[segmentId]);

      pcl::PointIndices currSegmentIds;
      currSegmentIds.indices = segmentIds[segmentId];
      casSegments.push_back(currSegmentIds);
    }

    cas.set(VIEW_BILATERAL_SEGMENTATION_IDS, casSegments);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";
    std::string symname = "sym" + std::to_string(segVisIt+1);
    if(numSymmetries > 0){
      if(firstRun){
        visualizer.addPointCloud(segments[segVisIt], cloudname);
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
        addSymmetryPlane(visualizer, finalSymmetries[segVisIt], symname, 0.05f, 0.05f);
        visualizer.addText("Segment " + std::to_string(segVisIt+1) + " / " + std::to_string(segments.size()), 15, 125, 24, 1.0, 1.0, 1.0);
      }
      else{
        visualizer.removeAllShapes();
        visualizer.updatePointCloud(segments[segVisIt], cloudname);
        visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
        addSymmetryPlane(visualizer, finalSymmetries[segVisIt], symname, 0.05f, 0.05f);
        visualizer.addText("Segment " + std::to_string(segVisIt+1) + " / " + std::to_string(segments.size()), 15, 125, 24, 1.0, 1.0, 1.0);
      }
    }
  }

private:
  inline void filter(){
    for(size_t symId = 0; symId < numSymmetries; symId++){
      if( symmetryScores[symId] < max_sym_score &&
          occlusionScores[symId] < max_occlusion_score &&
          cutScores[symId] < max_cut_score &&
          segmentIds[symId].size() > min_segment_size &&
          symmetrySupportOverlapScores[symId] > min_sym_sypport_overlap)
      {
        filteredSegmentIds.push_back(symId);
      }
    }
  }

  inline void merge()
  {
    Graph similarSegments(filteredSegmentIds.size());

    for(size_t srcSegmentIdIt = 0; srcSegmentIdIt < filteredSegmentIds.size(); srcSegmentIdIt++)
    {
      int srcSegmentId = filteredSegmentIds[srcSegmentIdIt];
      for(size_t tgtSegmentIdIt = srcSegmentIdIt+1; tgtSegmentIdIt < filteredSegmentIds.size(); tgtSegmentIdIt++)
      {
        int tgtSegmentId = filteredSegmentIds[tgtSegmentIdIt];

        int intersectSize = Intersection(segmentIds[srcSegmentId], segmentIds[tgtSegmentId]).size();
        int unionSize = Union(segmentIds[srcSegmentId], segmentIds[tgtSegmentId]).size();
        float ratio = (float) intersectSize / unionSize;
        if(ratio > overlap_threshold)
        {
          similarSegments.addEdge(srcSegmentIdIt, tgtSegmentIdIt);
        }
      }
    }

    std::vector< std::vector<int> > connectedSegments;
    connectedSegments = extractConnectedComponents(similarSegments);

    for(size_t ccId = 0; ccId < connectedSegments.size(); ccId++)
    {
      int bestId = -1;
      int max_segment_size = 0;
      for(size_t segIdIt = 0; segIdIt < connectedSegments[ccId].size(); segIdIt++)
      {
        int segmentId = filteredSegmentIds[connectedSegments[ccId][segIdIt]];
        if(segmentIds[segmentId].size() > max_segment_size)
        {
          bestId = segmentId;
          max_segment_size = segmentIds[segmentId].size();
        }
      }
      if(bestId != -1)
      {
        mergedSymmetryIds.push_back(bestId);
      }
    }
  }

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
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BilateralSymmetrySegmentation)
