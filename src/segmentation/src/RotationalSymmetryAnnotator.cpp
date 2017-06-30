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

#include <rs/segmentation/array_utils.hpp>
#include <rs/segmentation/RotationalSymmetry.hpp>
#include <rs/segmentation/RotationalSymmetryScoring.hpp>
#include <rs/segmentation/BoundarySegmentation.hpp>
#include <rs/occupancy_map/DistanceMap.hpp>
#include <rs/NonLinearOptimization/Functor.hpp>




using namespace uima;

class RotationalSymmetryAnnotator : public DrawingAnnotator
{
private:
  //container for inital symmetries usign PCA solver
  std::vector< std::vector<RotationalSymmetry> > segmentInitialSymmetries;
  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segmentClouds;
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
  std::vector< std::vector< std::vector<float> > >pointSymScores;
  std::vector< std::vector< std::vector<float> > > pointOcclusionScores;
  std::vector< std::vector< std::vector<float> > > pointPerpendicularScores;

  //container for filtered symmetries id
  std::vector< std::vector<int> > filteredSymmetries;
  std::vector<int> bestSymmetries;

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

    //allocating containers
    numSegments = segments.size();
    segmentInitialSymmetries.resize(numSegments);
    segmentRefinedSymmetries.resize(numSegments);
    segmentClouds.resize(numSegments);
    segmentNormals.resize(numSegments);
    segment_centroids.resize(numSegments);
    segmentSymScores.resize(numSegments);
    segmentOcclusionScores.resize(numSegments);
    segmentPerpendicularScores.resize(numSegments);
    segmentCoverageScores.resize(numSegments);
    pointSymScores.resize(numSegments);
    pointOcclusionScores.resize(numSegments);
    pointPerpendicularScores.resize(numSegments);

    //initialize distance map
    std::vector<Eigen::Vector4f> planes;
    planes.push_back(boundingPlane);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr test(new pcl::PointCloud<pcl::PointXYZRGBA>);
    test->width = 3;
    test->height = 1;
    test->points.resize(test->width * test->height);

    test->points[0].getVector3fMap() = Eigen::Vector3f(100.0f, 100.0f, 100.0f);
    test->points[1].getVector3fMap() = Eigen::Vector3f(0.0f, 100.0f, 100.0f);
    test->points[2].getVector3fMap() = Eigen::Vector3f(100.0f, 0.0f, 100.0f);

    dist_map = boost::shared_ptr< DistanceMap< pcl::PointXYZRGBA > >(new DistanceMap <pcl::PointXYZRGBA> (0.005f));
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
    }

    //TODO: filter refined symmtries by sym score, occlusion score, perpendicularity score and coverage score

    //TODO: merge similar symmtries by their orientation and points

    //TODO: define CAS Symmetries Type

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(firstRun){
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      addSymmetryLine(visualizer, segmentRefinedSymmetries, 0.2f, 0.4f);
    }
    else{
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      updateSymmetryLine(visualizer, segmentRefinedSymmetries, 0.2f, 0.4f);
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

      symScores[it] = getCloudSymmetryScore<PointT>(cloud, normals, refinedSymmetries[it], pointSymScores[it], min_fit_angle, max_fit_angle);
      occlusionScores[it] = getCloudOcclusionScore<PointT>(cloud, dist_map, refinedSymmetries[it], pointOcclusionScores[it], min_occlusion_dist, max_occlusion_dist);
      perpendicularScores[it] = getCloudPerpendicularScore(normals, refinedSymmetries[it], pointPerpendicularScores[it]);
      coverageScores[it] = getCloudCoverageScore<PointT>(cloud, refinedSymmetries[it]);
    }
  }

  template<typename PointT>
  inline void filterSymmetries(){

  }

  template<typename PointT>
  inline void getBestSymmetry(){

  }

  template<typename PointT>
  inline void mergeSymmetries(){

  }

  void addSymmetryLine(pcl::visualization::PCLVisualizer& visualizer, std::vector< std::vector<RotationalSymmetry> > symmetries, float length, float lineWidth){
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

  void updateSymmetryLine(pcl::visualization::PCLVisualizer& visualizer, std::vector< std::vector<RotationalSymmetry> > symmetries, float length, float lineWidth){
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

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RotationalSymmetryAnnotator)
