/**
 * Copyright 2017 University of Bremen, Institute for Artificial Intelligence
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

#ifndef __CONTOUR_FITTING_CLASSIFIER_H__
#define __CONTOUR_FITTING_CLASSIFIER_H__

#include <map>
#include <numeric>
#include <set>
#include <string>
#include <vector>

#include <uima/api.hpp>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/DrawingAnnotator.h>

#include <rs/utils/GeometryCV.h>
#include <rs/utils/Drawing.h>


using namespace uima;


/** \struct Mesh ContourFittingClassifier.cpp
 *  \brief Holds a polygon mesh vertices, triangles and normals
 */
struct Mesh
{
  /// \brief Vertices of the mesh
  std::vector<cv::Point3f> points;

  /// \brief Vertex normals, 1-1 correspondance to vertices
  std::vector<cv::Vec3f> normals;

  /// \brief Vectex indices for mesh triangles
  std::vector<std::vector<int>> triangles;

  /// \brief A median point of mesh
  ///   Used for proper perspective on silhouettes
  cv::Point3f origin;

  /// \brief Load mesh from PLY file
  /// \param[in] filename Relative path PLY file
  /// \return             A mesh structure if file was successfully loaded
  ///   Throws std::invalid_argument if file was not found
  static Mesh readFromFile(std::string const &filename);
};

/// \brief Extracts edges from image region into an array of points
/// \param[in] grayscale  An 8bit grayscale source image
/// \param[in] inputROI   A region where to look for edges
/// \return               std::vector of 2d points originating from the top-left corner of the image
///   Threshold for canny operator is chosen automatically
std::vector<cv::Point2f> getCannyEdges(const cv::Mat &grayscale,
                                       const cv::Rect &inputROI);

/// \brief Save 3d point to pcd file (for debug purposes)
/// \param[in] filename Path to store file to
/// \param[in] points      std::vector of 3d points
void saveToFile(const std::string filename,
                const std::vector<cv::Point3f> &points);

/// \brief Check if lookup tables are valid, and fill-in if not
/// \param[in]  camera  A camera object to create tables for
/// \param[in]  size    Size of the image produced by camera
/// \param[out] lookupX Horisontal lookup table
/// \param[out] lookupY Vertical lookup table
///   Lookup tables are ussed to get 3d coordinates of each observed point on
///   image, multiplying it's value from the lookup table on depth at given point
void checkViewCloudLookup(const ::Camera &camera, const cv::Size size,
                          cv::Mat &lookupX, cv::Mat &lookupY);

/// \brief Convert [0,infinity) distance to to a score value, which is higher if the distance is smaller
/// \param[in] distance Some value in range (0, +infinity)
/// \return             Score for this distance, positive for (0, 1) inputs
double distanceToScore(const double distance) noexcept;

/// \brief Create a PCL mesh from ::Mesh (used for visualisation purposes)
/// \param[in]  mesh  Polygon mesh
/// \param[in]  trans Affine transformation 3x4 matrix
/// \param[in]  tint  Value in [0,1] range to set surface color to
/// \return           A tuple (cloud, indices) which could be used for PCLVisualiser::addPolygonMesh()
std::tuple<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, std::vector<pcl::Vertices>>
    meshToPCLMesh(const ::Mesh &mesh, const cv::Mat &trans, const float tint);

/// \class MeshFootprint ContourFittingClassifier.cpp
/// \brief Stores an outer silhouette of a mesh at a specific pose using given camera
class MeshFootprint
{
  /// \brief Outer silhouette of mesh
  public: std::vector<cv::Point2f> outerEdge;

  /// \brief Surface edges of mesh
  public: std::vector<cv::Point2f> innerEdges;

  /// \brief Pose at which footprint was created
  public: ::PoseRT pose;

  /// \brief Constructor
  /// \param[in]  mesh      A mesh of which to create a footprint
  /// \param[in]  pose      Transformation applied to mesh before projecting
  /// \param[in]  camera    Camera which is used to project points
  /// \param[in]  imageSize Size of the image to rasterize image on (defines density of points)
  /// \param[in]  offset    Shift mesh before rotation to median point if true
  public: MeshFootprint(const ::Mesh &mesh, const ::PoseRT &pose,
      ::Camera &camera, const int imageSize, const bool offset = true);
};

/// \class MeshEdgeModel ContourFittingClassifier.cpp
/// \brief A container for footprint samples of a specific mesh
class MeshEdgeModel
{
  /// \brief An identifier of the mesh
  public: std::string name;

  /// \brief 3d mesh
  public: ::Mesh mesh;

  /// \brief Camera to create footprints with
  public: ::Camera camera;

  /// \brief Create and add footprint to the model
  /// \param[in] pose Pose to create footprint at
  /// \param[in] size Density of footprint points
  public: void addFootprint(const ::PoseRT &pose, const size_t size)
  {
    this->items.emplace_back(this->mesh, pose, this->camera, size);
  }

  /// \brief Automatically sample poses and add footprints to the model
  /// \param[in] rotationAxisSamples  number of rotation axis vector samples
  /// \param[in] rotationAngleSamples number of angle steps at each axis sample
  /// \param[in] size                 Density of footprint points
  public: void addSampledFootprints(const size_t rotationAxisSamples,
                                    const size_t rotationAngleSamples,
                                    const size_t size);

  /// \brief Save model to file
  /// \param[in] filename Path to model cache file
  public: void saveToFile(const std::string filename);

  /// \brief Load model from file
  /// \param[in] filename Path to model cache file
  /// \return True if model loaded
  public: bool loadFromFile(const std::string filename);

  /// \brief Footprint samples in the model
  public: std::vector<MeshFootprint> items;
};

/// \class PoseHypothsis ContourFittingClassifier.cpp
/// \brief An implementation of ::RankingItem interface containing pose data
class PoseHypothesis : public ::RankingItem<std::string, int>
{
  /// \brief Constructor
  public: PoseHypothesis(const std::string cId, const int sId,
                         const double score) :
    RankingItem<std::string, int>(cId, sId, score)
  {
  }

  /// \brief A hypothesis assumes that a mesh might be at this pose
  public: ::PoseRT pose;
};

/// \class ContourFittingClassifier ContourFittingClassifier.cpp
/// \brief Annotator which fits one of 3d meshes into a segment
///   Repairs ViewCloud and depth map at segments provides fitted meshes poses
class ContourFittingClassifier : public DrawingAnnotator
{
  friend class ContourFittingUnitTest;

  /// \brief Constructor
  public: ContourFittingClassifier() : DrawingAnnotator(__func__)
  {
  }

  // Documentation inherited
  public: TyErrorId initialize(AnnotatorContext &ctx);

  // Documentation inherited
  public: TyErrorId destroy();

  // Documentation inherited
  public: TyErrorId processWithLock(CAS &tcas,
                                    ResultSpecification const &res_spec);

  // Documentation inherited
  protected: void drawImageWithLock(cv::Mat &disp);

  // Documentation inherited
  protected: void fillVisualizerWithLock(
      pcl::visualization::PCLVisualizer &visualizer, const bool firstRun);

  /// \brief Write successfully classified hypothesis information to CAS
  /// \param[in,out] tcas           CAS
  /// \param[in,out] casImageDepth  Depth map to repair and write to CAS
  /// \param[in,out] casViewCloud   PCL cloud to repair and write to CAS
  /// \param[in]     hypothesis     Hupothesis being writed
  /// \param[in]     camera         Camera used to capture CAS image data
  protected: void drawHypothesisToCAS(
      CAS &tcas, cv::Mat &casImageDepth,
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr casViewCloud,
      const ::PoseHypothesis &hypothesis,
      const ::Camera &camera);

  /// \brief Orients mesh that way, that it's top ditection is orthogonal to support plane, and the origin point of mesh is in plane
  /// \param[in] initialPose          Initial mesh pose
  /// \param[in] meshAnchorPoint      A point in local meshes coordinates which is put on the plane
  /// \param[in] supportPlaneNormal   A vector orthogonal to plane
  /// \param[in] supportPlaneDistance A free term of plane equation
  /// \param[in] camera               Camera used to capture CAS image data
  /// \param[in] jacobian             Jacobian of PoseRT -> 2d-edge errors
  /// \return                         A new pose for the object
  ///   Assume that object's default orientation is bottom down.
  ///   The anchor point is aligned with plane along view ray.
  protected: ::PoseRT alignObjectsPoseWithPlane(const ::PoseRT &initialPose,
      const cv::Vec3f meshAnchorPoint, const cv::Vec3f supportPlaneNormal,
      const float supportPlaneDistance, const ::Camera &camera,
      const cv::Mat &jacobian);

  /// \brief Extract 3d points which corresponts to normal discontinuities visible on a mesh
  /// \param[in] mesh       Polygonal mesh
  /// \param[in] pose       Pose the mesh is observed at
  /// \param[in] camera     Camera to rasterize mesh
  /// \param[in] imageSize Size of the image to rasterize points to
  /// \return               std::vector of 3d points at default mesh's position
  protected: std::vector<cv::Point3f> getMeshSurfaceEdgesAtPose(
      const ::Mesh &mesh, const ::PoseRT &pose,
      const ::Camera &camera, const cv::Size imageSize);

  /// \brief Directory to store MeshEdgeModels at
  private: std::string cachePath{"/tmp"};

  /// \brief Number of rotation axis samples for footprint generation
  private: int rotationAxisSamples{10};

  /// \brief Number of rotation angle samples for footprint generation
  private: int rotationAngleSamples{10};

  /// \brief Footprint generation image size
  private: int footprintImageSize{240};

  /// \brief Reject all hypotheses which score is lower than value
  private: float rejectScoreLevel{0.001};

  /// \brief Keep only hypotheses which has a score this much lower than maximum one in set
  private: float normalizedAcceptScoreLevel{0.9};

  /// \brief Repair CAS data if true
  private: bool repairPointCloud{false};

  /// \brief Draw polygon meshes if PCL visualizer if true
  private: bool visualizeSolidMeshes{false};

  /// \brief Refine poses using ICP if true
  private: bool performICPPoseRefinement{false};

  /// \brief Align poses with support plane if true
  private: bool applySupportPlaneAssumption{false};

  /// \brief Maximal number of iterations to run ICP for
  private: int icp2d3dIterationsLimit{100};

  /// \brief Trained edge models for meshes
  private: std::map<std::string, ::MeshEdgeModel> edgeModels;

  /// \brief Currently visible segments
  private: std::vector<ImageSegmentation::Segment> segments;

  /// \brief Labels for segments
  private: std::vector<std::string> labels;

  /// \brief Good hypotheses to visualize
  private: std::vector<std::vector<::PoseHypothesis>> poseHypotheses;

  /// \brief Debug point set for visualisation
  private: std::vector<std::vector<cv::Point2f>> fittedSilhouettes;

  /// \brief Debug point set for visualisation
  private: std::vector<std::vector<cv::Point2f>> debugPointsRed;

  /// \brief Debug point set for visualisation
  private: std::vector<std::vector<cv::Point2f>> debugPointsBlue;

  /// \brief Histograms for hypotheses
  private: std::vector<cv::Mat> histograms;

  /// \brief Current RGB image resized to depth map size
  private: cv::Mat imageRGB;

  /// \brief Debug depth map for visualisation
  private: cv::Mat depthMap;

  /// \brief Current view cloud
  private: pcl::PointCloud<pcl::PointXYZRGBA>::Ptr viewCloud{
      new pcl::PointCloud<pcl::PointXYZRGBA>};

  /// \brief Horisontal camera lookup table
  private: cv::Mat lookupX;

  /// \brief Vertical camera lookup table
  private: cv::Mat lookupY;

  /// \brief Camera model of current data sample
  private: ::Camera camera;

  /// \brief Camera used to generate mesh footprints
  private: ::Camera footprintCamera;
};

#endif /*__CONTOUR_FITTING_CLASSIFIER_H__*/
