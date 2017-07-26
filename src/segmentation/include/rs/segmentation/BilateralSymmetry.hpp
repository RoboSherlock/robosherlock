#ifndef BILATERAL_SYMMETRY_HPP
#define BILATERAL_SYMMETRY_HPP

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <rs/segmentation/Geometry.hpp>
#include <rs/utils/output.h>

class BilateralSymmetry{
private:
  Eigen::Vector3f origin;
  Eigen::Vector3f normal;
public:

  BilateralSymmetry() : origin(Eigen::Vector3f::Zero()), normal(Eigen::Vector3f::Zero()) {}
  BilateralSymmetry(const Eigen::Vector3f& orig, const Eigen::Vector3f& nor) : origin(orig), normal(nor.normalized()) {}
  BilateralSymmetry(const Eigen::Vector4f& plane){
    planeToPointNormal<float>(plane, origin, normal);
  }

  ~BilateralSymmetry() {}

  Eigen::Vector3f getOrigin() const { return origin;}
  Eigen::Vector3f getNormal() const { return normal;}

  Eigen::Vector4f getPlane() const{
    Eigen::Vector4f plane;
    plane.head(3) = normal;
    plane[3] = -origin.dot(normal);
    return plane;
  }

  inline void setOrigin(const Eigen::Vector3f& orig) { origin = orig;}
  inline void setNormal(const Eigen::Vector3f& nor)  { normal = nor;}

  Eigen::Vector3f projectPoint(const Eigen::Vector3f& point) const{
    return pointToPlaneProjection<float>(point, origin, normal);
  }

  inline void setProjectedOrigin(const Eigen::Vector3f& point){
    origin = this->projectPoint(point);
  }

  inline float pointSignedDist(const Eigen::Vector3f& point) const{
    return pointToPlaneSignedNorm<float>(point, origin, normal);
  }

  inline void bilateralSymDiff(const BilateralSymmetry& target, float& angle, float& dist){
    angle = lineLineAngle<float>(this->normal, target.getNormal());
    dist = pointToPointNorm<float>(this->origin, target.getOrigin());
  }

  inline Eigen::Vector3f reflectPoint(const Eigen::Vector3f& point) const{
    return (point - 2 * this->normal * this->normal.dot(point - this->origin));
  }

  inline Eigen::Vector3f reflectNormal(const Eigen::Vector3f& normal) const{
    return (normal - 2 * normal.dot(this->normal) * this->normal);
  }

  inline float getBilSymNormalFitError(const Eigen::Vector3f& normal1, const Eigen::Vector3f& normal2){
    Eigen::Vector3f reflectedNormal2 = reflectNormal(normal2);
    float value = clamp(reflectedNormal2.dot(normal1), -1.0f, 1.0f);
    return std::acos(value);
  }

  inline float getBilSymPositionFitError(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2){
    Eigen::Vector3f mid = (point1 + point2) / 2;
    return pointSignedDist(mid);
  }
};

std::ostream& operator<<(std::ostream& output, const BilateralSymmetry& symmetry){
  output << "Origin: " << symmetry.getOrigin().transpose() << '\n';
  output << "Normal: " << symmetry.getNormal().transpose() << '\n';
  return output;
}

enum CorrespondenceMethod{
  NEAREST,
  NEIGHBOR_RADIUS
};

template<typename PointT>
inline bool findBilateralSymmetryCorrespondences(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                                 pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                                 typename pcl::PointCloud<PointT>::Ptr &dsCloud,
                                                 pcl::PointCloud<pcl::Normal>::Ptr &dsNormals,
                                                 typename pcl::search::KdTree<PointT>::Ptr &tree,
                                                 BilateralSymmetry &symmetry,
                                                 pcl::Correspondences &correspondences,
                                                 CorrespondenceMethod method,
                                                 float search_radius = 0.01f,
                                                 float max_normal_fit_error = 0.174f,
                                                 float min_sym_corresspondence_dist = 0.02f,
                                                 float max_sym_corresspondence_reflected_dist = 0.005
                                                 )
{
  if(cloud->size() == 0 || dsCloud->size() == 0)
  {
    outWarn("No point in cloud! Cloud need at least one point!");
    return false;
  }

  correspondences.clear();
  Eigen::Vector3f symOrigin = symmetry.getOrigin();
  Eigen::Vector3f symNormal = symmetry.getNormal();

  for(size_t pointId = 0; pointId < dsCloud->size(); pointId++)
  {
    Eigen::Vector3f srcPoint = dsCloud->points[pointId].getVector3fMap();
    Eigen::Vector3f srcNormal(dsNormals->points[pointId].normal_x, dsNormals->points[pointId].normal_y, dsNormals->points[pointId].normal_z);

    Eigen::Vector3f reflectedSrcPoint = symmetry.reflectPoint(srcPoint);
    Eigen::Vector3f reflectedSrcNormal = symmetry.reflectNormal(srcNormal);

    std::vector<float> dists;
    std::vector<int> neighborIndices;
    PointT searchPoint;
    searchPoint.getVector3fMap() = reflectedSrcPoint;

    if(method == NEIGHBOR_RADIUS)
    {
      tree->radiusSearch(searchPoint, search_radius, neighborIndices, dists);

      int bestId = -1;
      float minNormalFitError = std::numeric_limits<float>::max();
      for(size_t it = 0; it < neighborIndices.size(); it++)
      {
        int id = neighborIndices[it];
        Eigen::Vector3f tgtPoint = cloud->points[id].getVector3fMap();
        Eigen::Vector3f tgtNormal(normals->points[id].normal_x, normals->points[id].normal_y, normals->points[id].normal_z);

        if(std::abs(symmetry.pointSignedDist(srcPoint) - symmetry.pointSignedDist(tgtPoint)) < min_sym_corresspondence_dist)
        {
          continue;
        }

        float currNormalFitError = symmetry.getBilSymNormalFitError(reflectedSrcNormal, tgtNormal);

        if(currNormalFitError > max_normal_fit_error)
        {
          continue;
        }

        if(currNormalFitError < minNormalFitError)
        {
          minNormalFitError = currNormalFitError;
          bestId = id;
        }
      }

      if(bestId != -1)
      {
        correspondences.push_back(pcl::Correspondence(pointId, bestId, minNormalFitError));
      }
    }
    else if(method == NEAREST)
    {
      tree->nearestKSearch(searchPoint, 1, neighborIndices, dists);

      Eigen::Vector3f tgtPoint = cloud->points[neighborIndices[0]].getVector3fMap();
      Eigen::Vector3f tgtNormal(normals->points[neighborIndices[0]].normal_x, normals->points[neighborIndices[0]].normal_y, normals->points[neighborIndices[0]].normal_z);

      if(std::abs(symmetry.pointSignedDist(srcPoint) - symmetry.pointSignedDist(tgtPoint)) < min_sym_corresspondence_dist)
      {
        continue;
      }

      if(dists[0] > max_sym_corresspondence_reflected_dist * max_sym_corresspondence_reflected_dist)
      {
        continue;
      }

      float normalFitError = symmetry.getBilSymNormalFitError(reflectedSrcNormal, tgtNormal);

      if(normalFitError > max_normal_fit_error)
      {
        continue;
      }

      correspondences.push_back(pcl::Correspondence(pointId, neighborIndices[0], dists[0]));
    }
  }

  pcl::registration::CorrespondenceRejectorOneToOne corresRejectOneToOne;
  corresRejectOneToOne.getRemainingCorrespondences(correspondences, correspondences);

  if(correspondences.size() == 0)
  {
    return false;
  }
  return true;
}

#endif
