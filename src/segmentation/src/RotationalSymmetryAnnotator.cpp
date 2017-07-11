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

class RotationalSymmetryAnnotator : public DrawingAnnotator
{
private:
  //container for inital symmetries usign PCA solver
  std::vector< std::vector<RotationalSymmetry> > segmentInitialSymmetries;
  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segmentClouds;
  std::vector< std::vector<float> > symSupportSizes;
  std::vector< pcl::PointCloud<pcl::Normal>::Ptr > segmentNormals;
  std::vector<pcl::PointIndices> segments;
  std::vector<Eigen::Vector3f> segment_centroids;

  //container for refined symmetries
  std::vector< std::vector<RotationalSymmetry> > segmentRefinedSymmetries;

  //container for segment level score
  std::vector< std::vector<float> > segmentSymScores;
  std::vector< std::vector<float> > segmentOcclusionScores;
  std::vector< std::vector<float> > segmentPerpendicularScores;
  std::vector< std::vector<float> > segmentCoverageScores;

  //container for point level score
  std::vector< std::vector< std::vector<float> > > pointSymScores;
  std::vector< std::vector< std::vector<float> > > pointOcclusionScores;
  std::vector< std::vector< std::vector<float> > > pointPerpendicularScores;

  //container for filtered symmetries id
  std::vector< std::vector<int> > filteredSymmetries;
  std::vector< std::vector<int> > bestSymmetries;

  //container for final output to CAS
  std::vector<RotationalSymmetry> finalSymmetries;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  boost::shared_ptr< DistanceMap<pcl::PointXYZRGBA> > dist_map;
  Eigen::Vector4f boundingPlane; // this plane will be extracted from PlaneAnnotator

  int numSegments;

  //parameters
  float min_fit_angle;
  float max_fit_angle;

  float min_occlusion_dist;
  float max_occlusion_dist;

  float max_sym_score;
  float max_occlusion_score;
  float max_perpendicular_score;
  float min_coverage_score;

  float dist_map_resolution;

  float boundaryRadiusSearch;
  float boundaryAngleThreshold;

  float max_angle_diff;
  float max_dist_diff;



  double pointSize;

public:
  RotationalSymmetryAnnotator () : DrawingAnnotator(__func__), pointSize(1.0) {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("min_fit_angle", min_fit_angle);
    ctx.extractValue("max_fit_angle", max_fit_angle);
    ctx.extractValue("min_occlusion_dist", min_occlusion_dist);
    ctx.extractValue("max_occlusion_dist", max_occlusion_dist);

    ctx.extractValue("max_sym_score", max_sym_score);
    ctx.extractValue("max_occlusion_score", max_occlusion_score);
    ctx.extractValue("max_perpendicular_score", max_perpendicular_score);
    ctx.extractValue("min_coverage_score", min_coverage_score);

    ctx.extractValue("dist_map_resolution", dist_map_resolution);

    ctx.extractValue("boundaryRadiusSearch", boundaryRadiusSearch);
    ctx.extractValue("boundaryAngleThreshold", boundaryAngleThreshold);

    ctx.extractValue("max_angle_diff", max_angle_diff);
    ctx.extractValue("max_dist_diff", max_dist_diff);

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
    segmentRefinedSymmetries.clear();
    segmentClouds.clear();
    symSupportSizes.clear();
    segmentNormals.clear();
    segment_centroids.clear();
    segmentSymScores.clear();
    segmentOcclusionScores.clear();
    segmentPerpendicularScores.clear();
    segmentCoverageScores.clear();
    pointSymScores.clear();
    pointOcclusionScores.clear();
    pointPerpendicularScores.clear();
    filteredSymmetries.clear();
    bestSymmetries.clear();
    finalSymmetries.clear();

    //allocating containers
    numSegments = segments.size();
    segmentInitialSymmetries.resize(numSegments);
    segmentRefinedSymmetries.resize(numSegments);
    segmentClouds.resize(numSegments);
    symSupportSizes.resize(numSegments);
    segmentNormals.resize(numSegments);
    segment_centroids.resize(numSegments);
    segmentSymScores.resize(numSegments);
    segmentOcclusionScores.resize(numSegments);
    segmentPerpendicularScores.resize(numSegments);
    segmentCoverageScores.resize(numSegments);
    pointSymScores.resize(numSegments);
    pointOcclusionScores.resize(numSegments);
    pointPerpendicularScores.resize(numSegments);
    filteredSymmetries.resize(numSegments);
    bestSymmetries.resize(numSegments);

    //initialize distance map
    std::vector<Eigen::Vector4f> planes;
    planes.push_back(boundingPlane);

    dist_map = boost::shared_ptr< DistanceMap< pcl::PointXYZRGBA > >(new DistanceMap <pcl::PointXYZRGBA> (dist_map_resolution));
    dist_map->setBoundingPlanes(planes);
    dist_map->setInputCloud(cloud);

    //main execution
    #pragma omp parallel for
    for(size_t segmentId = 0; segmentId < numSegments; segmentId++){
      //extract cloud segments
      segmentClouds[segmentId].reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
      segmentNormals[segmentId].reset(new pcl::PointCloud<pcl::Normal>);
      pcl::copyPointCloud(*cloud, segments[segmentId], *segmentClouds[segmentId]);
      pcl::copyPointCloud(*normals, segments[segmentId], *segmentNormals[segmentId]);

      //extract cloud with no boundary
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr non_boundary_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::Normal>::Ptr non_boundary_normal(new pcl::PointCloud<pcl::Normal>);
      std::vector<int> boundary_indices, non_boundary_indices;
      extractBoundaryCloud<pcl::PointXYZRGBA, pcl::Normal>(segmentClouds[segmentId], segmentNormals[segmentId], boundary_indices, non_boundary_indices, boundaryRadiusSearch, boundaryAngleThreshold);
      pcl::copyPointCloud(*segmentClouds[segmentId], non_boundary_indices, *non_boundary_cloud);
      pcl::copyPointCloud(*segmentNormals[segmentId], non_boundary_indices, *non_boundary_normal);

      //detect initial symmetries on each cloud segment using PCA, result 3 symmetries on 3 dimension
      detectInitialSymmetries<pcl::PointXYZRGBA>(segmentClouds[segmentId], segmentInitialSymmetries[segmentId], segment_centroids[segmentId]);

      //refind symmteries using LevenbergMarquardt algorithm (damped least squared fitting)
      refineSymmtries<pcl::PointXYZRGBA>(non_boundary_cloud,
                                         non_boundary_normal,
                                         segment_centroids[segmentId],
                                         symSupportSizes[segmentId],
                                         *dist_map,
                                         segmentInitialSymmetries[segmentId],
                                         segmentRefinedSymmetries[segmentId],
                                         segmentSymScores[segmentId],
                                         segmentOcclusionScores[segmentId],
                                         segmentPerpendicularScores[segmentId],
                                         segmentCoverageScores[segmentId],
                                         pointSymScores[segmentId],
                                         pointOcclusionScores[segmentId],
                                         pointPerpendicularScores[segmentId]);

      //filter out bad estimated symmetries by bounding scores
      filterSymmetries(segmentSymScores[segmentId],
                       segmentOcclusionScores[segmentId],
                       segmentPerpendicularScores[segmentId],
                       segmentCoverageScores[segmentId],
                       segmentId);

      //get best representation symmetry of a segment based on occlusion scores
      getBestSymmetryID(filteredSymmetries[segmentId], segmentOcclusionScores[segmentId], segmentId);
    }

    //create linear container of symmetries for merging similar symmetries
    std::vector<RotationalSymmetry> unmergedSymmetries;
    std::vector<float> unmergedSupportSizes;
    linearizeSegmentData<RotationalSymmetry>(segmentRefinedSymmetries, unmergedSymmetries, bestSymmetries);
    linearizeSegmentData<float>(symSupportSizes, unmergedSupportSizes, bestSymmetries);

    mergeSymmetries(unmergedSymmetries, unmergedSupportSizes, finalSymmetries);

    //convert RotationalSymmetry to CAS Symmetries and push to CAS
    std::vector<rs::RotationalSymmetry> casSymmetries;
    for(size_t it = 0; it < finalSymmetries.size(); it++){
      rs::RotationalSymmetry currSym = rs::create<rs::RotationalSymmetry>(tcas);
      rs::Point3f currOrigin = rs::create<rs::Point3f>(tcas);
      rs::Point3f currOrientation = rs::create<rs::Point3f>(tcas);

      Eigen::Vector3f eigenOrigin = finalSymmetries[it].getOrigin();
      Eigen::Vector3f eigenOrientation = finalSymmetries[it].getOrientation();

      currOrigin.x.set(eigenOrigin[0]);
      currOrigin.y.set(eigenOrigin[1]);
      currOrigin.z.set(eigenOrigin[2]);

      currOrientation.x.set(eigenOrientation[0]);
      currOrientation.y.set(eigenOrientation[1]);
      currOrientation.z.set(eigenOrientation[2]);

      currSym.origin.set(currOrigin);
      currSym.orientation.set(currOrientation);
      casSymmetries.push_back(currSym);
    }

    cas.set(VIEW_ROTATIONAL_SYMMETRIES, casSymmetries);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(firstRun){
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      addSymmetryLine(visualizer, finalSymmetries, 0.4f, 0.8f);
    }
    else{
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      updateSymmetryLine(visualizer, finalSymmetries, 0.4f, 0.8f);
    }

  }

private:
  template<typename PointT>
  inline void detectInitialSymmetries(typename pcl::PointCloud<PointT>::Ptr& cloud, std::vector<RotationalSymmetry>& symmetries, Eigen::Vector3f& segmentCentroid){
    symmetries.clear();
    if(cloud->points.size() < 3)
      outInfo("Segment has under 3 points. Symmetries will not calculated!");

    pcl::PCA<pcl::PointXYZRGBA> pca;
    pca.setInputCloud(cloud);
    segmentCentroid = pca.getMean().head(3);

    symmetries.resize(3);
    symmetries[0] = RotationalSymmetry(segmentCentroid, pca.getEigenVectors().col(0));
    symmetries[1] = RotationalSymmetry(segmentCentroid, pca.getEigenVectors().col(1));
    symmetries[2] = RotationalSymmetry(segmentCentroid, pca.getEigenVectors().col(2));
  }

  template<typename PointT>
  inline void refineSymmtries(typename pcl::PointCloud<PointT>::Ptr& cloud,
                              pcl::PointCloud<pcl::Normal>::Ptr& normals,
                              Eigen::Vector3f& segmentCentroid,
                              std::vector<float>& supportSizes,
                              DistanceMap<PointT>& dist_map,
                              std::vector<RotationalSymmetry>& initialSymmetries,
                              std::vector<RotationalSymmetry>& refinedSymmetries,
                              std::vector<float>& symScores,
                              std::vector<float>& occlusionScores,
                              std::vector<float>& perpendicularScores,
                              std::vector<float>& coverageScores,
                              std::vector< std::vector<float> >& pointSymScores,
                              std::vector< std::vector<float> >& pointOcclusionScores,
                              std::vector< std::vector<float> >& pointPerpendicularScores
                              )
  {
    int initialSymSize = initialSymmetries.size();
    refinedSymmetries.resize(initialSymSize);
    supportSizes.resize(initialSymSize);
    symScores.resize(initialSymSize);
    occlusionScores.resize(initialSymSize);
    perpendicularScores.resize(initialSymSize);
    coverageScores.resize(initialSymSize);
    pointSymScores.resize(initialSymSize);
    pointOcclusionScores.resize(initialSymSize);
    pointPerpendicularScores.resize(initialSymSize);

    for(size_t it = 0; it < initialSymSize; it++){
      RotSymOptimizeFunctorDiff<PointT> functor;
      functor.cloud = cloud;
      functor.normals = normals;
      functor.max_fit_angle = max_fit_angle;

      Eigen::LevenbergMarquardt<RotSymOptimizeFunctorDiff<PointT>, float> optimizer(functor);
      optimizer.parameters.ftol = 1e-10;
      optimizer.parameters.maxfev = 800;

      Eigen::VectorXf sym(6);
      sym.head(3) = initialSymmetries[it].getOrigin();
      sym.tail(3) = initialSymmetries[it].getOrientation();
      optimizer.minimize(sym);
      refinedSymmetries[it] = RotationalSymmetry(sym.head(3), sym.tail(3));
      refinedSymmetries[it].setProjectedOrigin(segmentCentroid);

      supportSizes[it] = static_cast<float>(cloud->points.size());

      symScores[it] = getCloudSymmetryScore<PointT>(cloud, normals, refinedSymmetries[it], pointSymScores[it], min_fit_angle, max_fit_angle);
      occlusionScores[it] = getCloudOcclusionScore<PointT>(cloud, dist_map, refinedSymmetries[it], pointOcclusionScores[it], min_occlusion_dist, max_occlusion_dist);
      perpendicularScores[it] = getCloudPerpendicularScore(normals, refinedSymmetries[it], pointPerpendicularScores[it]);
      coverageScores[it] = getCloudCoverageScore<PointT>(cloud, refinedSymmetries[it]);

      //std::cout << " SymScore: " << symScores[it] << " OccScores: " << occlusionScores[it] << " perScores: " << perpendicularScores[it] << " CovScores: " << coverageScores[it] << '\n';
    }
  }

  inline void filterSymmetries(std::vector<float>& symScores,
                               std::vector<float>& occlusionScores,
                               std::vector<float>& perpendicularScores,
                               std::vector<float>& coverageScores,
                               int segmentId){
    filteredSymmetries[segmentId].clear();
    int symSize = symScores.size();

    for(size_t symId = 0; symId < symSize; symId++){
      if(symScores[symId] < max_sym_score &&
         occlusionScores[symId] < max_occlusion_dist &&
         perpendicularScores[symId] < max_perpendicular_score &&
         coverageScores[symId] > min_coverage_score){
           filteredSymmetries[segmentId].push_back(symId);
         }
    }
  }

  inline void getBestSymmetryID(std::vector<int>& symmetryIds, std::vector<float>& occlusionScores, int segmentId){
    float bestScore = std::numeric_limits<float>::max(); // a.k.a min occlusionScores (consistent cloud)
    int bestSymId = -1;
    int symSize = symmetryIds.size();
    for(size_t it = 0; it < symSize; it++){
      int symId = symmetryIds[it];
      if(occlusionScores[symId] < bestScore){
        bestSymId = symId;
        bestScore = occlusionScores[symId];
      }
    }

    bestSymmetries[segmentId].push_back(bestSymId);
  }

  template<typename Type>
  inline void linearizeSegmentData(typename std::vector< std::vector<Type> >& segmentDataIn, typename std::vector<Type>& segmentDataOut, std::vector< std::vector<int> > indices = std::vector<int>(0)){ // for both scores and Symmetries
    segmentDataOut.clear();

    for(size_t segmentIt = 0; segmentIt < segmentDataIn.size(); segmentIt++){
      if(indices.size() != 0){
        int dataId;
        for(size_t it = 0; it < indices[segmentIt].size(); it++){
          dataId = indices[segmentIt][it];
          if(dataId >= 0 && dataId < segmentDataIn[segmentIt].size()){
            segmentDataOut.push_back(segmentDataIn[segmentIt][dataId]);
          }
        }
      }
      else{
        for(size_t it = 0; it < segmentDataIn[segmentIt].size(); it++){
          segmentDataOut.push_back(segmentDataIn[segmentIt][it]);
        }
      }
    }
  }

  inline bool mergeSymmetries(std::vector<RotationalSymmetry>& symmetries,
                              std::vector<float>& symSupportSize,
                              std::vector<RotationalSymmetry>& mergedSymmetries)
  {
    mergedSymmetries.clear();

    Graph symGraph(symmetries.size());

    for(size_t srcId = 0; srcId < symmetries.size(); srcId++){
      RotationalSymmetry& srcSym = symmetries[srcId];

      for(size_t tgtId = srcId+1; tgtId < symmetries.size();tgtId++){
        RotationalSymmetry& tgtSym = symmetries[tgtId];

        float angle, dist;
        srcSym.getRotSymDifference(tgtSym, angle, dist);

        if(angle < max_angle_diff && dist < max_dist_diff)
          symGraph.addEdge(srcId, tgtId);
      }
    }

    std::vector< std::vector<int> > symConnectedComponents;
    symConnectedComponents = extractConnectedComponents(symGraph);


    for(size_t clusterId = 0; clusterId < symConnectedComponents.size(); clusterId++){
      float maxSize = -1.0f;
      float bestSym = -1;
      for(size_t symIdIt = 0; symIdIt < symConnectedComponents[clusterId].size(); symIdIt++){
        int symId = symConnectedComponents[clusterId][symIdIt];
        if(symSupportSize[symId] > maxSize){
          maxSize = symSupportSize[symId];
          bestSym = symId;
        }
      }
      if(bestSym == -1){
        outError("Could not merge similar rotational symmetries!");
        return false;
      }

      mergedSymmetries.push_back(symmetries[bestSym]);
    }
    return true;
  }

  void addSymmetryLine(pcl::visualization::PCLVisualizer& visualizer, std::vector< std::vector<RotationalSymmetry> >& symmetries, float length, float lineWidth){
    for(size_t segId = 0; segId < numSegments; segId++){
      for(size_t symId = 0; symId < symmetries[segId].size(); symId++){
        pcl::PointXYZ p1, p2;
        p1.getVector3fMap() = symmetries[segId][symId].getOrigin() + symmetries[segId][symId].getOrientation() * length / 2;
        p2.getVector3fMap() = symmetries[segId][symId].getOrigin() - symmetries[segId][symId].getOrientation() * length / 2;

        std::string id = "sym" + std::to_string(segId * symmetries[segId].size() + symId);
        visualizer.addLine(p1, p2, id);
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, id);
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, id);
      }
    }
  }

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

  void updateSymmetryLine(pcl::visualization::PCLVisualizer& visualizer, std::vector< std::vector<RotationalSymmetry> >& symmetries, float length, float lineWidth){
    for(size_t segId = 0; segId < numSegments; segId++){
      for(size_t symId = 0; symId < symmetries[segId].size(); symId++){
        pcl::PointXYZ p1, p2;
        p1.getVector3fMap() = symmetries[segId][symId].getOrigin() + symmetries[segId][symId].getOrientation() * length / 2;
        p2.getVector3fMap() = symmetries[segId][symId].getOrigin() - symmetries[segId][symId].getOrientation() * length / 2;

        std::string id = "sym" + std::to_string(segId * symmetries[segId].size() + symId);
        visualizer.removeShape(id);
        visualizer.addLine(p1, p2, id);
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, id);
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, id);
      }
    }
  }

  void updateSymmetryLine(pcl::visualization::PCLVisualizer& visualizer, std::vector<RotationalSymmetry>& symmetries, float length, float lineWidth){
    for(size_t symId = 0; symId < symmetries.size(); symId++){
      pcl::PointXYZ p1, p2;
      p1.getVector3fMap() = symmetries[symId].getOrigin() + symmetries[symId].getOrientation() * length / 2;
      p2.getVector3fMap() = symmetries[symId].getOrigin() - symmetries[symId].getOrientation() * length / 2;

      std::string id = "sym" + std::to_string(symId);
      visualizer.removeShape(id);
      visualizer.addLine(p1, p2, id);
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, id);
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, id);
    }
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RotationalSymmetryAnnotator)
