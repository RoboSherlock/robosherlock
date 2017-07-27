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
#include <rs/segmentation/BilateralSymmetry.hpp>
#include <rs/segmentation/BilateralSymmetryScoring.hpp>
#include <rs/segmentation/BoundarySegmentation.hpp>
#include <rs/occupancy_map/DistanceMap.hpp>
#include <rs/occupancy_map/DownsampleMap.hpp>
#include <rs/NonLinearOptimization/Functor.hpp>
#include <rs/graph/Graph.hpp>
#include <rs/graph/GraphAlgorithms.hpp>

using namespace uima;

class BilateralSymmetryAnnotator : public DrawingAnnotator
{
private:
  //container for inital symmetries usign PCA solver
  std::vector< std::vector<BilateralSymmetry> > segmentInitialSymmetries;
  std::vector< std::vector<BilateralSymmetry> > segmentRefinedSymmetries;

  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segmentClouds;
  std::vector< pcl::PointCloud<pcl::Normal>::Ptr > segmentNormals;
  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segmentDSClouds;
  std::vector< pcl::PointCloud<pcl::Normal>::Ptr > segmentDSNormals;

  std::vector< std::vector<float> > symSupportSizes;
  std::vector<pcl::PointIndices> segments;
  std::vector<Eigen::Vector3f> segment_centroids;

  std::vector< std::vector< std::vector<float> > > pointSymScores;
  std::vector< std::vector< std::vector<float> > > pointOcclusionScores;

  std::vector< std::vector<float> > occlusionScores;
  std::vector< std::vector<float> > segmentInlierScores;
  std::vector< std::vector<float> > corresInlierScores;

  std::vector< std::vector<bool> > validSymmetries;

  boost::shared_ptr< DistanceMap<pcl::PointXYZRGBA> > dist_map;
  Eigen::Vector4f boundingPlane; // this plane will be extracted from PlaneAnnotator

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  bool isDownsampled;
  float downsample_voxel_size;

  int angle_division;

  float correspondence_search_radius;
  float correspondence_max_normal_fit_error;
  float correspondence_min_sym_dist;
  float correspondence_max_sym_reflected_dist;

  int refine_max_iteration;
  float refine_min_inlier_sym_score;
  float refine_max_inlier_sym_score;

  float min_occlusion_dist;
  float max_occlusion_dist;

  float max_occlusion_score;
  float min_segment_inlier_score;
  float min_corres_inlier_score;

  int numSegments;

  double pointSize;

public:
  BilateralSymmetryAnnotator () : DrawingAnnotator(__func__), pointSize(1.0) {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("isDownsampled", isDownsampled);
    ctx.extractValue("downsample_voxel_size", downsample_voxel_size);

    ctx.extractValue("angle_division", angle_division);

    ctx.extractValue("correspondence_search_radius", correspondence_search_radius);
    ctx.extractValue("correspondence_max_normal_fit_error", correspondence_max_normal_fit_error);
    ctx.extractValue("correspondence_min_sym_dist", correspondence_min_sym_dist);
    ctx.extractValue("correspondence_max_sym_reflected_dist", correspondence_max_sym_reflected_dist);

    ctx.extractValue("refine_max_iteration", refine_max_iteration);
    ctx.extractValue("refine_min_inlier_sym_score", refine_min_inlier_sym_score);
    ctx.extractValue("refine_max_inlier_sym_score", refine_max_inlier_sym_score);

    ctx.extractValue("min_occlusion_dist", min_occlusion_dist);
    ctx.extractValue("max_occlusion_dist", max_occlusion_dist);

    ctx.extractValue("max_occlusion_score", max_occlusion_score);
    ctx.extractValue("min_segment_inlier_score", min_segment_inlier_score);
    ctx.extractValue("min_corres_inlier_score", min_corres_inlier_score);

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
    segmentDSClouds.clear();
    segmentDSNormals.clear();
    segment_centroids.clear();
    pointSymScores.clear();
    pointOcclusionScores.clear();
    occlusionScores.clear();
    segmentInlierScores.clear();
    corresInlierScores.clear();
    validSymmetries.clear();


    //allocating containers
    numSegments = segments.size();
    segmentInitialSymmetries.resize(numSegments);
    segmentRefinedSymmetries.resize(numSegments);
    segmentClouds.resize(numSegments);
    segmentDSClouds.resize(numSegments);
    segmentDSNormals.resize(numSegments);
    symSupportSizes.resize(numSegments);
    segmentNormals.resize(numSegments);
    segment_centroids.resize(numSegments);
    pointSymScores.resize(numSegments);
    pointOcclusionScores.resize(numSegments);
    occlusionScores.resize(numSegments);
    segmentInlierScores.resize(numSegments);
    corresInlierScores.resize(numSegments);
    validSymmetries.resize(numSegments);

    //initialize distance map
    std::vector<Eigen::Vector4f> planes;
    planes.push_back(boundingPlane);

    dist_map = boost::shared_ptr< DistanceMap< pcl::PointXYZRGBA > >(new DistanceMap <pcl::PointXYZRGBA> (dist_map_resolution));
    dist_map->setBoundingPlanes(planes);
    dist_map->setInputCloud(cloud);

    #pragma omp parallel for
    for(size_t segmentId = 0; segmentId < numSegments; segmentId++){
      //extract cloud segments
      segmentClouds[segmentId].reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
      segmentNormals[segmentId].reset(new pcl::PointCloud<pcl::Normal>);
      pcl::copyPointCloud(*cloud, segments[segmentId], *segmentClouds[segmentId]);
      pcl::copyPointCloud(*normals, segments[segmentId], *segmentNormals[segmentId]);

      segmentDSClouds[segmentId].reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
      segmentDSNormals[segmentId].reset(new pcl::PointCloud<pcl::Normal>);

      if(isDownsampled)
      {
        std::vector< std::vector<int> > dsMap;
        std::vector<int> nearestMap;
        DownsampleMap<pcl::PointXYZRGBA> ds;
        ds.setInputCloud(segmentClouds[segmentId]);
        ds.setLeafSize(downsample_voxel_size);
        ds.filter(*segmentDSClouds[segmentId]);
        ds.getDownsampleMap(dsMap);
        ds.getNearestNeighborMap(nearestMap);

        computeDownsampleNormals(segmentNormals[segmentId], dsMap, nearestMap, AVERAGE, segmentDSNormals[segmentId]);
      }
      else{
        segmentDSClouds[segmentId] = segmentClouds[segmentId];
        segmentDSNormals[segmentId] = segmentNormals[segmentId];
      }

      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
      tree->setInputCloud(segmentClouds[segmentId]);

      detectInitialSymmetries<pcl::PointXYZRGBA>(segmentClouds[segmentId], segmentInitialSymmetries[segmentId], segment_centroids[segmentId]);

      std::vector< BilateralSymmetry > temp_symmetries;
      if(! refineBilateralSymmetryPosition<pcl::PointXYZRGBA>(segmentClouds[segmentId],
                                                              segmentNormals[segmentId],
                                                              segmentDSClouds[segmentId],
                                                              segmentDSNormals[segmentId],
                                                              tree,
                                                              segmentInitialSymmetries[segmentId],
                                                              temp_symmetries))
      {
        continue;
      }

      if(! refineBilateralSymmetryFitting<pcl::PointXYZRGBA>(segmentClouds[segmentId],
                                                             segmentNormals[segmentId],
                                                             segmentDSClouds[segmentId],
                                                             segmentDSNormals[segmentId],
                                                             segment_centroids[segmentId],
                                                             tree,
                                                             *dist_map,
                                                             temp_symmetries,
                                                             segmentRefinedSymmetries[segmentId],
                                                             pointSymScores[segmentId],
                                                             pointOcclusionScores[segmentId],
                                                             occlusionScores[segmentId],
                                                             segmentInlierScores[segmentId],
                                                             corresInlierScores[segmentId],
                                                             validSymmetries[segmentId]))
      {
        continue;
      }


    }

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(firstRun){
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      addSymmetryPlanes(visualizer, segmentRefinedSymmetries, 0.05f, 0.05f);
    }
    else{
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeAllShapes();
      addSymmetryPlanes(visualizer, segmentRefinedSymmetries, 0.05f, 0.05f);
    }

  }

private:
  template<typename PointT>
  inline bool detectInitialSymmetries(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                      std::vector<BilateralSymmetry> &symmetries,
                                      Eigen::Vector3f &segmentCentroid)
  {
    symmetries.clear();

    if(cloud->size() < 3)
    {
      outWarn("Cloud does not have sufficient points to detect symmetry!");
      return false;
    }

    pcl::PCA<PointT> pca;
    pca.setInputCloud(cloud);
    segmentCentroid = pca.getMean().head(3);
    Eigen::Matrix3f basis = pca.getEigenVectors();

    //ensure axes are right hand coordinate
    if(basis.col(0).cross(basis.col(1)).dot(basis.col(2)) < 0)
    {
      basis.col(2) *= -1.0f;
    }

    std::vector<Eigen::Vector3f> points;
    generateHemisphere(angle_division, points);

    for(size_t pointId = 0; pointId < points.size(); pointId++)
    {
      symmetries.push_back(BilateralSymmetry(segmentCentroid, basis * points[pointId]));
    }
    return true;
  }

  template<typename PointT>
  inline bool refineBilateralSymmetryPosition(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                              pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                              typename pcl::PointCloud<PointT>::Ptr &dsCloud,
                                              pcl::PointCloud<pcl::Normal>::Ptr &dsNormals,
                                              typename pcl::search::KdTree<PointT>::Ptr &tree,
                                              std::vector<BilateralSymmetry> &initial_symmetries,
                                              std::vector<BilateralSymmetry> &refined_symmetries)
  {

    if(cloud->size() == 0 || dsCloud->size() == 0)
    {
      outWarn("No point in cloud! Cloud need at least one point!");
      return false;
    }

    refined_symmetries.resize(initial_symmetries.size());

    #pragma omp parallel for
    for(size_t symId = 0; symId < initial_symmetries.size(); symId++)
    {
      pcl::Correspondences symCorrespondences;

      refined_symmetries[symId] = initial_symmetries[symId];

      bool success = findBilateralSymmetryCorrespondences<PointT>(cloud, normals, dsCloud, dsNormals, tree, initial_symmetries[symId], symCorrespondences, NEIGHBOR_RADIUS, correspondence_search_radius, correspondence_max_normal_fit_error, correspondence_min_sym_dist, correspondence_max_sym_reflected_dist);

      if(success)
      {
        std::vector<float> positionFitErrors(symCorrespondences.size());
        for(size_t it = 0; it < symCorrespondences.size(); it++)
        {
          int queryId = symCorrespondences[it].index_query;
          int matchId = symCorrespondences[it].index_match;
          positionFitErrors[it] = initial_symmetries[symId].getBilSymPositionFitError(dsCloud->points[queryId].getVector3fMap(), cloud->points[matchId].getVector3fMap());
        }

        float medianError = median<float>(positionFitErrors);
        refined_symmetries[symId].setOrigin(initial_symmetries[symId].getOrigin() + initial_symmetries[symId].getNormal() * medianError);
      }
    }
    return true;
  }

  template<typename PointT>
  inline bool refineBilateralSymmetryFitting(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                             pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                             typename pcl::PointCloud<PointT>::Ptr &dsCloud,
                                             pcl::PointCloud<pcl::Normal>::Ptr &dsNormals,
                                             Eigen::Vector3f &segmentCentroid,
                                             typename pcl::search::KdTree<PointT>::Ptr &tree,
                                             DistanceMap<PointT> &dist_map,
                                             std::vector<BilateralSymmetry> &initial_symmetries,
                                             std::vector<BilateralSymmetry> &refined_symmetries,
                                             std::vector< std::vector<float> > &point_symmetry_scores,
                                             std::vector< std::vector<float> > &point_occlusion_scores,
                                             std::vector<float> &occlusion_scores,
                                             std::vector<float> &segment_inlier_scores,
                                             std::vector<float> &corres_inlier_scores,
                                             std::vector<bool> &valid_symmetries)
  {
    if(cloud->size() == 0 || dsCloud->size() == 0)
    {
      outWarn("No point in cloud! Cloud need at least one point!");
      return false;
    }

    refined_symmetries.resize(initial_symmetries.size());
    point_symmetry_scores.resize(initial_symmetries.size());
    point_occlusion_scores.resize(initial_symmetries.size());
    occlusion_scores.resize(initial_symmetries.size());
    segment_inlier_scores.resize(initial_symmetries.size());
    corres_inlier_scores.resize(initial_symmetries.size());
    valid_symmetries.resize(initial_symmetries.size(), true);

    BilSymOptimizeFunctorDiff<PointT> functor;
    functor.cloud = cloud;
    functor.normals = normals;
    functor.dsCloud = dsCloud;

    pcl::Correspondences correspondences;

    #pragma omp parallel for
    for(size_t symId = 0; symId < initial_symmetries.size(); symId++)
    {
      refined_symmetries[symId] = initial_symmetries[symId];

      BilateralSymmetry last_symmetry;
      for(size_t iteration = 0; iteration < max_iter; iteration++)
      {
        last_symmetry = refined_symmetries[symId];

        correspondences.clear();
        bool success = findBilateralSymmetryCorrespondences<PointT>(cloud, normals, dsCloud, dsNormals, tree, refined_symmetries[symId], correspondences, NEAREST, correspondence_search_radius, correspondence_max_normal_fit_error, correspondence_min_sym_dist, correspondence_max_sym_reflected_dist);

        if(success)
        {
          Eigen::VectorXf x(6);
          x.head(3) = refined_symmetries[symId].getOrigin();
          x.tail(3) = refined_symmetries[symId].getNormal();

          functor.correspondences = correspondences;

          Eigen::LevenbergMarquardt< BilSymOptimizeFunctorDiff<PointT>, float> optimizer(functor);
          optimizer.minimize(x);

          refined_symmetries[symId] = BilateralSymmetry(x.head(3), x.tail(3));
          refined_symmetries[symId].setProjectedOrigin(segmentCentroid);

          float angleDiff, distDiff;
          refined_symmetries[symId].bilateralSymDiff(last_symmetry, angleDiff, distDiff);
          if(angleDiff < 0.0017f && distDiff < 0.0005f)
          {
            break;
          }
        }
        else
        {
          valid_symmetries[symId] = false;
          break;
        }
      }

      //compute symmetry score
      getCloudBilateralSymmetryScore(cloud, normals, dsCloud, dsNormals, tree, refined_symmetries[symId], correspondences, point_symmetry_scores[symId], correspondence_search_radius, correspondence_max_normal_fit_error, correspondence_min_sym_dist, correspondence_max_sym_reflected_dist, refine_min_inlier_sym_score, refine_max_inlier_sym_score);
      getCloudBilateralOcclusionScore(dsCloud, dist_map, refined_symmetries[symId], point_occlusion_scores[symId], min_occlusion_dist, max_occlusion_dist);

      float inlierSum = 0.0f;
      for(size_t corresId = 0; corresId < correspondences.size(); corresId++)
      {
        inlierSum += (1.0f - point_symmetry_scores[corresId]);
      }

      occlusion_scores[symId] = mean(point_occlusion_scores);
      segment_inlier_scores[symId] = inlierSum / static_cast<float>(dsCloud->size());
      corres_inlier_scores[symId] = inlierSum / static_cast<float>(correspondences.size());
    }

    return true;
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
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BilateralSymmetryAnnotator)
