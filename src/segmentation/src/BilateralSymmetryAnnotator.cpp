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
  std::vector< BilateralSymmetry > finalSymmetries;
  std::vector< int > finalSupportSizeIds;
  std::vector< std::vector<int> > filteredSymmetryIds;

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
  std::vector<Eigen::Vector4f> boundingPlanes; // this plane will be extracted from PlaneAnnotator

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  bool bilSymAnn_isDownsampled;
  bool naive_detection;

  float downsample_voxel_size;

  int angle_division;

  float dist_map_resolution;

  float correspondence_search_radius;
  float correspondence_max_normal_fit_error;
  float correspondence_min_sym_dist;
  float correspondence_max_sym_reflected_dist;

  int refine_max_iteration;
  float refine_min_inlier_sym_score;
  float refine_max_inlier_sym_score;

  float bilSymAnn_min_occlusion_dist;
  float bilSymAnn_max_occlusion_dist;

  float bilSymAnn_max_occlusion_score;
  float min_segment_inlier_score;
  float min_corres_inlier_score;

  float sym_angle_diff;
  float sym_dist_diff;

  int numSegments;

  double pointSize;

public:
  BilateralSymmetryAnnotator () : DrawingAnnotator(__func__), pointSize(1.0) {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("bilSymAnn_isDownsampled", bilSymAnn_isDownsampled);
    ctx.extractValue("naive_detection", naive_detection);
    ctx.extractValue("downsample_voxel_size", downsample_voxel_size);

    ctx.extractValue("angle_division", angle_division);

    ctx.extractValue("dist_map_resolution", dist_map_resolution);

    ctx.extractValue("correspondence_search_radius", correspondence_search_radius);
    ctx.extractValue("correspondence_max_normal_fit_error", correspondence_max_normal_fit_error);
    ctx.extractValue("correspondence_min_sym_dist", correspondence_min_sym_dist);
    ctx.extractValue("correspondence_max_sym_reflected_dist", correspondence_max_sym_reflected_dist);

    ctx.extractValue("refine_max_iteration", refine_max_iteration);
    ctx.extractValue("refine_min_inlier_sym_score", refine_min_inlier_sym_score);
    ctx.extractValue("refine_max_inlier_sym_score", refine_max_inlier_sym_score);

    ctx.extractValue("bilSymAnn_min_occlusion_dist", bilSymAnn_min_occlusion_dist);
    ctx.extractValue("bilSymAnn_max_occlusion_dist", bilSymAnn_max_occlusion_dist);

    ctx.extractValue("bilSymAnn_max_occlusion_score", bilSymAnn_max_occlusion_score);
    ctx.extractValue("min_segment_inlier_score", min_segment_inlier_score);
    ctx.extractValue("min_corres_inlier_score", min_corres_inlier_score);

    ctx.extractValue("sym_angle_diff", sym_angle_diff);
    ctx.extractValue("sym_dist_diff", sym_dist_diff);

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

    //get RGB cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    cas.get(VIEW_CLOUD_OBJECTS, *cloud_ptr);
    if(cloud_ptr->size() == 0)
    {
      outInfo("Input Object cloud address is empty! Using scene cloud");
      cas.get(VIEW_CLOUD, *cloud_ptr);
      cas.get(VIEW_NORMALS, *normals);
    }
    else
    {
      //get normal cloud
      cas.get(VIEW_NORMALS_OBJECTS, *normals);
    }
    cloud = cloud_ptr;

    //get segments
    cas.get(VIEW_SEGMENT_IDS, segments);

    //clearing for consecutive frame
    segmentInitialSymmetries.clear();
    segmentRefinedSymmetries.clear();
    finalSymmetries.clear();
    filteredSymmetryIds.clear();
    finalSupportSizeIds.clear();
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
    filteredSymmetryIds.resize(numSegments);
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

      if(bilSymAnn_isDownsampled)
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
      //NOTE: somehow this optimization does not increase accuracy of symmetry pose
      /*if(! refineBilateralSymmetryPosition<pcl::PointXYZRGBA>(segmentClouds[segmentId],
                                                              segmentNormals[segmentId],
                                                              segmentDSClouds[segmentId],
                                                              segmentDSNormals[segmentId],
                                                              tree,
                                                              segmentInitialSymmetries[segmentId],
                                                              temp_symmetries))
      {
        continue;
      }*/

      temp_symmetries = segmentInitialSymmetries[segmentId];
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

      //filter symmetries based on score and linearize data to an array
      this->filterSymmetries(segmentId);
    }

    this->mergeSymmetries();
    //linearizeSegmentData<BilateralSymmetry>(segmentRefinedSymmetries, finalSymmetries);

    //convert BilateralSymmetry to CAS Symmetries and push to CAS
    std::vector<rs::BilateralSymmetry> casSymmetries;
    for(size_t symId = 0; symId < finalSymmetries.size(); symId++)
    {
      rs::BilateralSymmetry currSym = rs::create<rs::BilateralSymmetry>(tcas);
      rs::Point3f currOrigin = rs::create<rs::Point3f>(tcas);
      rs::Point3f currNormal = rs::create<rs::Point3f>(tcas);

      Eigen::Vector3f eigenOrigin = finalSymmetries[symId].getOrigin();
      Eigen::Vector3f eigenNormal = finalSymmetries[symId].getNormal();

      currOrigin.x.set(eigenOrigin[0]);
      currOrigin.y.set(eigenOrigin[1]);
      currOrigin.z.set(eigenOrigin[2]);

      currNormal.x.set(eigenNormal[0]);
      currNormal.y.set(eigenNormal[1]);
      currNormal.z.set(eigenNormal[2]);

      currSym.origin.set(currOrigin);
      currSym.normal.set(currNormal);
      currSym.support.set(segments[finalSupportSizeIds[symId]].indices);
      casSymmetries.push_back(currSym);
    }

    outInfo("Total detected symmetries: " << casSymmetries.size());

    cas.set(VIEW_BILATERAL_SYMMETRIES, casSymmetries);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(firstRun){
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      addSymmetryPlanes(visualizer, finalSymmetries, 0.05f, 0.05f);
    }
    else{
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeAllShapes();
      addSymmetryPlanes(visualizer, finalSymmetries, 0.05f, 0.05f);
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

    if(naive_detection)
    {
      symmetries.push_back(BilateralSymmetry(segmentCentroid, basis.col(1)));
      symmetries.push_back(BilateralSymmetry(segmentCentroid, basis.col(2)));
      symmetries.push_back(BilateralSymmetry(segmentCentroid, basis.col(0)));
    }
    else
    {
      std::vector<Eigen::Vector3f> points;
      generateHemisphere(angle_division, points);

      for(size_t pointId = 0; pointId < points.size(); pointId++)
      {
        symmetries.push_back(BilateralSymmetry(segmentCentroid, basis * points[pointId]));
      }
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
      pcl::Correspondences correspondences;

      refined_symmetries[symId] = initial_symmetries[symId];

      //NOTE: first approach, finding correspondences based on error of reflected normal and src normal
      //bool success = findBilateralSymmetryCorrespondences<PointT>(cloud, normals, dsCloud, dsNormals, tree, initial_symmetries[symId], correspondences, NEIGHBOR_RADIUS, correspondence_search_radius, correspondence_max_normal_fit_error, correspondence_min_sym_dist, correspondence_max_sym_reflected_dist);

      //NOTE:second approach
      bool success = true;
      //finding correspondences
      Eigen::Vector3f symOrigin = initial_symmetries[symId].getOrigin();
      Eigen::Vector3f symNormal = initial_symmetries[symId].getNormal();

      typename pcl::PointCloud<PointT>::Ptr projectedCloud(new pcl::PointCloud<PointT>);
      cloudToPlaneProjection<PointT>(cloud, symOrigin, symNormal, projectedCloud);

      typename pcl::PointCloud<PointT>::Ptr projectedDSCloud(new pcl::PointCloud<PointT>);
      cloudToPlaneProjection<PointT>(dsCloud, symOrigin, symNormal, projectedDSCloud);

      typename pcl::search::KdTree<PointT> projectedTree;
      projectedTree.setInputCloud(projectedCloud);

      for(size_t pointId = 0; pointId < projectedDSCloud->size();pointId++)
      {
        Eigen::Vector3f srcPoint = dsCloud->points[pointId].getVector3fMap();
        Eigen::Vector3f srcNormal(dsNormals->points[pointId].normal_x, dsNormals->points[pointId].normal_y, dsNormals->points[pointId].normal_z);

        std::vector<float> dists;
        std::vector<int> neighbors;
        projectedTree.radiusSearch(projectedDSCloud->points[pointId], correspondence_search_radius, neighbors, dists);

        int bestId = -1;
        float minNormalFitError = std::numeric_limits<float>::max();
        for(size_t it = 0; it < neighbors.size();it++)
        {
          int neighborId = neighbors[it];
          Eigen::Vector3f tgtPoint = cloud->points[neighborId].getVector3fMap();
          Eigen::Vector3f tgtNormal(normals->points[neighborId].normal_x, normals->points[neighborId].normal_y, normals->points[neighborId].normal_z);

          if(std::abs(initial_symmetries[symId].pointSignedDist(srcPoint) - initial_symmetries[symId].pointSignedDist(tgtPoint)) < correspondence_min_sym_dist)
          {
            continue;
          }

          float currNormalFitError = initial_symmetries[symId].getBilSymNormalFitError(srcNormal, tgtNormal);

          if(currNormalFitError > correspondence_max_normal_fit_error)
          {
            continue;
          }

          if(currNormalFitError < minNormalFitError)
          {
            minNormalFitError = currNormalFitError;
            bestId = neighborId;
          }
        }

        if(bestId != -1)
        {
          correspondences.push_back(pcl::Correspondence(pointId, bestId, minNormalFitError));
        }
      }

      pcl::registration::CorrespondenceRejectorOneToOne correspRejectOneToOne;
      correspRejectOneToOne.getRemainingCorrespondences(correspondences, correspondences);

      if (correspondences.size() == 0)
      {
        success = false;
      }

      if(success)
      {
        std::vector<float> positionFitErrors(correspondences.size());
        for(size_t it = 0; it < correspondences.size(); it++)
        {
          int queryId = correspondences[it].index_query;
          int matchId = correspondences[it].index_match;
          positionFitErrors[it] = initial_symmetries[symId].getBilSymPositionFitError(dsCloud->points[queryId].getVector3fMap(), cloud->points[matchId].getVector3fMap());
        }

        float medianError = median<float>(positionFitErrors);
        refined_symmetries[symId].setOrigin(symOrigin + symNormal * medianError);
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
      for(size_t iteration = 0; iteration < refine_max_iteration; iteration++)
      {
        last_symmetry = refined_symmetries[symId];

        correspondences.clear();

        //NOTE: first approach, finding correspondences based on error of reflected normal and src normal
        //bool success = findBilateralSymmetryCorrespondences<PointT>(cloud, normals, dsCloud, dsNormals, tree, refined_symmetries[symId], correspondences, NEIGHBOR_RADIUS, correspondence_search_radius, correspondence_max_normal_fit_error, correspondence_min_sym_dist, correspondence_max_sym_reflected_dist);

        //NOTE: second approach
        bool success = true;
        //finding correspondences
        Eigen::Vector3f symOrigin = refined_symmetries[symId].getOrigin();
        Eigen::Vector3f symNormal = refined_symmetries[symId].getNormal();

        for(size_t pointId = 0; pointId < dsCloud->size();pointId++)
        {
          Eigen::Vector3f srcPoint = dsCloud->points[pointId].getVector3fMap();
          Eigen::Vector3f srcNormal(dsNormals->points[pointId].normal_x, dsNormals->points[pointId].normal_y, dsNormals->points[pointId].normal_z);

          Eigen::Vector3f reflectedSrcPoint = refined_symmetries[symId].reflectPoint(srcPoint);
          Eigen::Vector3f reflectedSrcNormal = refined_symmetries[symId].reflectNormal(srcNormal);

          std::vector<float> dists(1);
          std::vector<int> neighbors(1);
          PointT searchPoint;
          searchPoint.getVector3fMap() = reflectedSrcPoint;
          tree->nearestKSearch(searchPoint, 1, neighbors, dists);

          Eigen::Vector3f tgtPoint = cloud->points[neighbors[0]].getVector3fMap();
          Eigen::Vector3f tgtNormal(normals->points[neighbors[0]].normal_x, normals->points[neighbors[0]].normal_y, normals->points[neighbors[0]].normal_z);

          if(std::abs(refined_symmetries[symId].pointSignedDist(srcPoint) - refined_symmetries[symId].pointSignedDist(tgtPoint)) < correspondence_min_sym_dist)
          {
            continue;
          }

          if(dists[0] > correspondence_max_sym_reflected_dist * correspondence_max_sym_reflected_dist)
          {
            continue;
          }

          float normalError = refined_symmetries[symId].getBilSymNormalFitError(srcNormal, tgtNormal);

          if(normalError > correspondence_max_normal_fit_error)
          {
            continue;
          }

          correspondences.push_back(pcl::Correspondence(pointId, neighbors[0], dists[0]));

        }

        pcl::registration::CorrespondenceRejectorOneToOne correspRejectOneToOne;
        correspRejectOneToOne.getRemainingCorrespondences(correspondences, correspondences);

        if (correspondences.size() == 0)
        {
          success = false;
        }

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
          if(angleDiff < 0.05f && distDiff < 0.0001f)
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
      getCloudBilateralSymmetryScore<PointT>(cloud, normals, dsCloud, dsNormals, tree, refined_symmetries[symId], correspondences, point_symmetry_scores[symId], correspondence_search_radius, correspondence_max_normal_fit_error, correspondence_min_sym_dist, correspondence_max_sym_reflected_dist, refine_min_inlier_sym_score, refine_max_inlier_sym_score);
      getCloudBilateralOcclusionScore<PointT>(dsCloud, dist_map, refined_symmetries[symId], point_occlusion_scores[symId], bilSymAnn_min_occlusion_dist, bilSymAnn_max_occlusion_dist);

      float inlierSum = 0.0f;
      for(size_t corresId = 0; corresId < correspondences.size(); corresId++)
      {
        inlierSum += (1.0f - point_symmetry_scores[symId][corresId]);
      }

      occlusion_scores[symId] = mean(point_occlusion_scores[symId]);
      segment_inlier_scores[symId] = inlierSum / static_cast<float>(dsCloud->size());
      corres_inlier_scores[symId] = inlierSum / static_cast<float>(correspondences.size());
    }

    return true;
  }

  inline void filterSymmetries(int segmentId)
  {
    for(size_t symId = 0; symId < segmentRefinedSymmetries[segmentId].size(); symId++)
    {
      if(validSymmetries[segmentId][symId])
      {
        if(occlusionScores[segmentId][symId] < bilSymAnn_max_occlusion_score &&
           segmentInlierScores[segmentId][symId] > min_segment_inlier_score&&
           corresInlierScores[segmentId][symId] > min_corres_inlier_score)
        {
          filteredSymmetryIds[segmentId].push_back(symId);
        }
      }
    }
  }

  inline void mergeSymmetries()
  {
    std::vector<float> linear_occlusion_score;
    std::vector<BilateralSymmetry> linear_symmetries;
    std::vector<int> linear_support_size_ids;
    for(size_t segmentId = 0; segmentId < numSegments; segmentId++)
    {
      for(size_t symIdIt = 0; symIdIt < filteredSymmetryIds[segmentId].size(); symIdIt++)
      {
        int symId = filteredSymmetryIds[segmentId][symIdIt];
        linear_occlusion_score.push_back(occlusionScores[segmentId][symId]);
        linear_symmetries.push_back(segmentRefinedSymmetries[segmentId][symId]);
        linear_support_size_ids.push_back(segmentId);
      }
    }

    Graph symGraph(linear_symmetries.size());

    for(size_t srcId = 0; srcId < linear_symmetries.size(); srcId++)
    {
      BilateralSymmetry srcSym = linear_symmetries[srcId];

      for(size_t tgtId = srcId+1; tgtId < linear_symmetries.size(); tgtId++)
      {
        BilateralSymmetry tgtSym = linear_symmetries[tgtId];

        float angleDiff, distDiff;
        srcSym.bilateralSymDiff(tgtSym, angleDiff, distDiff);
        if(angleDiff < sym_angle_diff && distDiff < sym_dist_diff)
        {
          symGraph.addEdge(srcId, tgtId);
        }
      }
    }

    std::vector< std::vector<int> > symConnectedComponents;
    symConnectedComponents = extractConnectedComponents(symGraph);

    for(size_t clusterId = 0; clusterId < symConnectedComponents.size(); clusterId++)
    {
      float minScore = std::numeric_limits<float>::max();
      float bestSym = -1;
      for(size_t symIdIt = 0; symIdIt < symConnectedComponents[clusterId].size(); symIdIt++)
      {
        int symId = symConnectedComponents[clusterId][symIdIt];
        if(linear_occlusion_score[symId] < minScore)
        {
          minScore = linear_occlusion_score[symId];
          bestSym = symId;
        }
      }

      if(bestSym == -1)
      {
        outWarn("Could not merge similar bilateral symmetries!");
      }

      finalSupportSizeIds.push_back(linear_support_size_ids[bestSym]);
      finalSymmetries.push_back(linear_symmetries[bestSym]);
    }
  }

  template<typename Type>
  inline void linearizeSegmentData(typename std::vector< std::vector<Type> >& segmentDataIn, typename std::vector<Type>& segmentDataOut, std::vector< std::vector<int> > indices = std::vector< std::vector<int> >()){ // for both scores and Symmetries
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
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BilateralSymmetryAnnotator)
