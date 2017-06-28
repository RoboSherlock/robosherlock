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
#include <rs/occupancy_map/DistanceMap.hpp>
#include <rs/NonLinearOptimization/Functor.hpp>



using namespace uima;

class RotationalSymmetryAnnotator : public DrawingAnnotator
{
private:
  std::vector< std::vector<RotationalSymmetry> > segmentSymmetries;
  std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > segmentClouds;
  std::vector<pcl::PointIndices> segments;
  std::vector<Eigen::Vector3f> segment_centroids;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

  boost::shared_ptr< DistanceMap<pcl::PointXYZ> > dist_map;

  int numSegments;

  double pointSize;

public:
  RotationalSymmetryAnnotator () : DrawingAnnotator(__func__), pointSize(1.0) {
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
    numSegments = segments.size();
    segmentSymmetries.resize(numSegments);
    segmentClouds.resize(numSegments);
    segment_centroids.resize(numSegments);

    //main execution
    #pragma omp parallel for
    for(size_t segmentId = 0; segmentId < numSegments; segmentId++){
      segmentClouds[segmentId].reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::copyPointCloud(*cloud, segments[segmentId], *segmentClouds[segmentId]);

      detectInitialSymmetries<pcl::PointXYZRGBA>(segmentClouds[segmentId], segmentSymmetries[segmentId], segment_centroids[segmentId]);

      //TODO: refine symmtries using LevenbergMarquardt algorithm without boudary points

      //TODO: filter refined symmtries by sym score, occlusion score, perpendicularity score and coverage score

      //TODO: merge similar symmtries by their orientation and points

      //TODO: define CAS Symmetries Type
    }



    Eigen::Vector4f plane(0.0f, 0.0f, 1.0f, 0.0f);
    std::vector<Eigen::Vector4f> planes;
    planes.push_back(plane);

    pcl::PointCloud<pcl::PointXYZ>::Ptr test(new pcl::PointCloud<pcl::PointXYZ>);
    test->width = 3;
    test->height = 1;
    test->points.resize(test->width * test->height);

    test->points[0].getVector3fMap() = Eigen::Vector3f(100.0f, 100.0f, 100.0f);
    test->points[1].getVector3fMap() = Eigen::Vector3f(0.0f, 100.0f, 100.0f);
    test->points[2].getVector3fMap() = Eigen::Vector3f(100.0f, 0.0f, 100.0f);

    pcl::PointXYZ searchPoint;
    searchPoint.x = 130.0f;
    searchPoint.y = 100.0f;
    searchPoint.z = 100.0f;

    dist_map = boost::shared_ptr< DistanceMap< pcl::PointXYZ > >(new DistanceMap <pcl::PointXYZ> (0.1f, 0.1f, 0.3f));
    dist_map->setBoundingPlanes(planes);
    dist_map->setInputCloud(test);
    int index;
    float dist;

    dist_map->getNearestOccupiedDistance(searchPoint, index, dist);
    std::cout << "Point 1 index: " << index << " with dist: " << dist << '\n';

    searchPoint.x = 0.0f;
    searchPoint.y = 70.0f;
    dist_map->getNearestOccupiedDistance(searchPoint, index, dist);
    std::cout << "Point 2 index: " << index << " with dist: " << dist << '\n';
    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer, const bool firstRun)
  {
    const std::string cloudname = this->name + "_cloud";

    if(firstRun){
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      addSymmetryLine(visualizer, 0.2f, 0.2f);
    }
    else{
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      updateSymmetryLine(visualizer, 0.2f, 0.2f);
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
  inline void refineSymmtries(){

  }

  template<typename PointT>
  inline void filterSymmetries(){

  }

  template<typename PointT>
  inline void mergeSymmetries(){

  }

  template<typename PointT>
  inline float getCloudSymmetryScore(typename pcl::PointCloud<PointT>::Ptr& cloud,
                                     pcl::PointCloud<pcl::Normal>::Ptr& normals,
                                     RotationalSymmetry& symmetry,
                                     std::vector<float>& point_symmetry_scores,
                                     float min_fit_angle = 0.0f,
                                     float max_fit_angle = M_PI / 2)
  {
    point_symmetry_scores.resize(cloud.points.size());

    for(size_t it = 0; it < cloud.points.size(); it++){
      Eigen::Vector3f point =  cloud->points[it].getVector3fMap();
      Eigen::Vector3f normal( normals->points[it].data_c[0], normals->points[it].data_c[1], normals->points[it].data_c[2]);

      float angle = getRotSymFitError(point, normal, symmetry);
      float score = (angle - min_fit_angle) / (max_fit_angle - min_fit_angle);

      score = clamp(score, 0.0f, 1.0f);
      point_symmetry_scores[it] = score;
    }

    return mean(point_symmetry_scores);
  }

  template<typename PointT>
  inline float getCloudOcclusionScore(typename pcl::PointCloud<PointT>& cloud,
                                      DistanceMap<PointT>& dist_map,
                                      RotationalSymmetry& symmetry,
                                      std::vector<float>& point_occlusion_scores,
                                      float min_occlusion_dist = 0.0f,
                                      float max_occlusion_dist = 1.0f,
                                      int redundant_factor = 12)
  {
    point_occlusion_scores.resize(cloud.points.size());

    return 0.0f;
  }

  void addSymmetryLine(pcl::visualization::PCLVisualizer& visualizer, float length, float lineWidth){
    for(size_t segId = 0; segId < numSegments; segId++){
      for(size_t symId = 0; symId < segmentSymmetries[segId].size(); symId++){
        pcl::PointXYZ p1, p2;
        p1.getVector3fMap() = segmentSymmetries[segId][symId].getOrigin() + segmentSymmetries[segId][symId].getOrientation() * length / 2;
        p2.getVector3fMap() = segmentSymmetries[segId][symId].getOrigin() - segmentSymmetries[segId][symId].getOrientation() * length / 2;

        std::string id = "sym" + std::to_string(segId * segmentSymmetries[segId].size() + symId);
        visualizer.addLine(p1, p2, id);
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, id);
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, id);
      }
    }
  }

  void updateSymmetryLine(pcl::visualization::PCLVisualizer& visualizer, float length, float lineWidth){
    for(size_t segId = 0; segId < numSegments; segId++){
      for(size_t symId = 0; symId < segmentSymmetries[segId].size(); symId++){
        pcl::PointXYZ p1, p2;
        p1.getVector3fMap() = segmentSymmetries[segId][symId].getOrigin() + segmentSymmetries[segId][symId].getOrientation() * length / 2;
        p2.getVector3fMap() = segmentSymmetries[segId][symId].getOrigin() - segmentSymmetries[segId][symId].getOrientation() * length / 2;

        std::string id = "sym" + std::to_string(segId * segmentSymmetries[segId].size() + symId);
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
