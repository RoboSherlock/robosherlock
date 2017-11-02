/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
#include <rs/mapping/DistanceMap.hpp>
#include <rs/NonLinearOptimization/Functor.hpp>
#include <rs/graph/Graph.hpp>
#include <rs/graph/GraphAlgorithms.hpp>

#include <rs/visualization/Primitives.hpp>


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
  std::vector<Eigen::Vector4f> boundingPlanes; // this plane will be extracted from PlaneAnnotator

  int numSegments;

  //parameters
  float rotSymAnn_min_fit_angle;
  float rotSymAnn_max_fit_angle;

  float rotSymAnn_min_occlusion_dist;
  float rotSymAnn_max_occlusion_dist;

  float rotSymAnn_max_sym_score;
  float rotSymAnn_max_occlusion_score;
  float rotSymAnn_max_perpendicular_score;
  float rotSymAnn_min_coverage_score;

  float dist_map_resolution;

  float boundaryRadiusSearch;
  float boundaryAngleThreshold;

  float max_angle_diff;
  float max_dist_diff;

  double pointSize;

public:
  RotationalSymmetryAnnotator () : DrawingAnnotator(__func__), pointSize(1.0)
  {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("rotSymAnn_min_fit_angle", rotSymAnn_min_fit_angle);
    ctx.extractValue("rotSymAnn_max_fit_angle", rotSymAnn_max_fit_angle);
    ctx.extractValue("rotSymAnn_min_occlusion_dist", rotSymAnn_min_occlusion_dist);
    ctx.extractValue("rotSymAnn_max_occlusion_dist", rotSymAnn_max_occlusion_dist);

    ctx.extractValue("rotSymAnn_max_sym_score", rotSymAnn_max_sym_score);
    ctx.extractValue("rotSymAnn_max_occlusion_score", rotSymAnn_max_occlusion_score);
    ctx.extractValue("rotSymAnn_max_perpendicular_score", rotSymAnn_max_perpendicular_score);
    ctx.extractValue("rotSymAnn_min_coverage_score", rotSymAnn_min_coverage_score);

    ctx.extractValue("dist_map_resolution", dist_map_resolution);

    ctx.extractValue("boundaryRadiusSearch", boundaryRadiusSearch);
    ctx.extractValue("boundaryAngleThreshold", boundaryAngleThreshold);

    ctx.extractValue("max_angle_diff", max_angle_diff);
    ctx.extractValue("max_dist_diff", max_dist_diff);

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

    //get RGB objects cloud
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
    boundingPlanes.clear();

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

    //main execution
    #pragma omp parallel for
    for(size_t segmentId = 0; segmentId < numSegments; segmentId++)
    {
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
      this->filterSymmetries(segmentId);

      //get best representation symmetry of a segment based on occlusion scores
      this->getBestSymmetryID(segmentId);
    }

    //create linear container of symmetries for merging similar symmetries
    std::vector<RotationalSymmetry> unmergedSymmetries;
    std::vector<float> unmergedSupportSizes;
    linearizeSegmentData<RotationalSymmetry>(segmentRefinedSymmetries, unmergedSymmetries, bestSymmetries);
    linearizeSegmentData<float>(symSupportSizes, unmergedSupportSizes, bestSymmetries);

    mergeSymmetries(unmergedSymmetries, unmergedSupportSizes, finalSymmetries);

    //convert RotationalSymmetry to CAS Symmetries and push to CAS
    std::vector<rs::RotationalSymmetry> casSymmetries;
    for(size_t it = 0; it < finalSymmetries.size(); it++)
    {
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

    if(firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      addSymmetryLines(visualizer, finalSymmetries, 0.4f, 0.8f);
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      addSymmetryLines(visualizer, finalSymmetries, 0.4f, 0.8f);
    }
  }

private:
  template<typename PointT>
  inline void detectInitialSymmetries(typename pcl::PointCloud<PointT>::Ptr &cloud, std::vector<RotationalSymmetry> &symmetries, Eigen::Vector3f &segmentCentroid)
  {
    symmetries.clear();
    if(cloud->points.size() < 3)
    {
      outInfo("Segment has under 3 points. Symmetries will not calculated!");
    }

    pcl::PCA<pcl::PointXYZRGBA> pca;
    pca.setInputCloud(cloud);
    segmentCentroid = pca.getMean().head(3);

    symmetries.resize(3);
    symmetries[0] = RotationalSymmetry(segmentCentroid, pca.getEigenVectors().col(0));
    symmetries[1] = RotationalSymmetry(segmentCentroid, pca.getEigenVectors().col(1));
    symmetries[2] = RotationalSymmetry(segmentCentroid, pca.getEigenVectors().col(2));
  }

  template<typename PointT>
  inline void refineSymmtries(typename pcl::PointCloud<PointT>::Ptr &cloud,
                              pcl::PointCloud<pcl::Normal>::Ptr &normals,
                              Eigen::Vector3f &segmentCentroid,
                              std::vector<float> &supportSizes,
                              DistanceMap<PointT> &dist_map,
                              std::vector<RotationalSymmetry> &initialSymmetries,
                              std::vector<RotationalSymmetry> &refinedSymmetries,
                              std::vector<float> &symScores,
                              std::vector<float> &occlusionScores,
                              std::vector<float> &perpendicularScores,
                              std::vector<float> &coverageScores,
                              std::vector< std::vector<float> > &pointSymScores,
                              std::vector< std::vector<float> > &pointOcclusionScores,
                              std::vector< std::vector<float> > &pointPerpendicularScores)
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

    for(size_t it = 0; it < initialSymSize; it++)
    {
      RotSymOptimizeFunctorDiff<PointT> functor;
      functor.cloud = cloud;
      functor.normals = normals;
      functor.max_fit_angle = rotSymAnn_max_fit_angle;

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

      symScores[it] = getCloudRotationalSymmetryScore<PointT>(cloud, normals, refinedSymmetries[it], pointSymScores[it], rotSymAnn_min_fit_angle, rotSymAnn_max_fit_angle);
      occlusionScores[it] = getCloudRotationalOcclusionScore<PointT>(cloud, dist_map, refinedSymmetries[it], pointOcclusionScores[it], rotSymAnn_min_occlusion_dist, rotSymAnn_max_occlusion_dist);
      perpendicularScores[it] = getCloudRotationalPerpendicularScore(normals, refinedSymmetries[it], pointPerpendicularScores[it]);
      coverageScores[it] = getCloudRotationalCoverageScore<PointT>(cloud, refinedSymmetries[it]);
    }
  }

  inline void filterSymmetries(int segmentId)
  {
    filteredSymmetries[segmentId].clear();
    int symSize = segmentSymScores[segmentId].size();

    for(size_t symId = 0; symId < symSize; symId++)
    {
      if(segmentSymScores[segmentId][symId] < rotSymAnn_max_sym_score &&
         segmentOcclusionScores[segmentId][symId] < rotSymAnn_max_occlusion_score &&
         segmentPerpendicularScores[segmentId][symId] < rotSymAnn_max_perpendicular_score &&
         segmentCoverageScores[segmentId][symId] > rotSymAnn_min_coverage_score)
      {
        filteredSymmetries[segmentId].push_back(symId);
      }
    }
  }

  inline void getBestSymmetryID(int segmentId)
  {
    float bestScore = std::numeric_limits<float>::max(); // a.k.a min occlusionScores (consistent cloud)
    int bestSymId = -1;
    int symSize = filteredSymmetries[segmentId].size();
    for(size_t it = 0; it < symSize; it++)
    {
      int symId = filteredSymmetries[segmentId][it];
      if(segmentOcclusionScores[segmentId][symId] < bestScore)
      {
        bestSymId = symId;
        bestScore = segmentOcclusionScores[segmentId][symId];
      }
    }

    bestSymmetries[segmentId].push_back(bestSymId);
  }

  inline bool mergeSymmetries(std::vector<RotationalSymmetry> &symmetries,
                              std::vector<float> &symSupportSize,
                              std::vector<RotationalSymmetry> &mergedSymmetries)
  {
    mergedSymmetries.clear();

    Graph symGraph(symmetries.size());

    for(size_t srcId = 0; srcId < symmetries.size(); srcId++)
    {
      RotationalSymmetry &srcSym = symmetries[srcId];

      for(size_t tgtId = srcId+1; tgtId < symmetries.size();tgtId++)
      {
        RotationalSymmetry &tgtSym = symmetries[tgtId];

        float angle, dist;
        srcSym.getRotSymDifference(tgtSym, angle, dist);

        if( (angle < max_angle_diff || (M_PI - angle) < max_angle_diff) && dist < max_dist_diff)
        {
          symGraph.addEdge(srcId, tgtId);
        }
      }
    }

    std::vector< std::vector<int> > symConnectedComponents;
    symConnectedComponents = extractConnectedComponents(symGraph);

    for(size_t clusterId = 0; clusterId < symConnectedComponents.size(); clusterId++)
    {
      float maxSize = -1.0f;
      float bestSym = -1;
      for(size_t symIdIt = 0; symIdIt < symConnectedComponents[clusterId].size(); symIdIt++)
      {
        int symId = symConnectedComponents[clusterId][symIdIt];
        if(symSupportSize[symId] > maxSize)
        {
          maxSize = symSupportSize[symId];
          bestSym = symId;
        }
      }
      if(bestSym == -1)
      {
        outError("Could not merge similar rotational symmetries!");
        return false;
      }

      mergedSymmetries.push_back(symmetries[bestSym]);
    }
    return true;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(RotationalSymmetryAnnotator)
