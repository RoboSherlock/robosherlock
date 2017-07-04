#ifndef ROTATIONAL_SYMMETRY_ANNOTATOR_HPP
#define ROTATIONAL_SYMMETRY_ANNOTATOR_HPP

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <rs/segmentation/Geometry.hpp>
#include <rs/utils/output.h>

class RotationalSymmetry {
private:
  Eigen::Vector3f origin;
  Eigen::Vector3f orientation;

public:
  RotationalSymmetry() : origin(Eigen::Vector3f::Zero()), orientation(Eigen::Vector3f::Zero()) {}
  RotationalSymmetry(const Eigen::Vector3f& point, const Eigen::Vector3f& vec) : origin(point), orientation(vec.normalized()) {}

  Eigen::Vector3f getOrigin() const{
    return origin;
  }

  Eigen::Vector3f getOrientation() const{
    return orientation;
  }

  void setOrigin(Eigen::Vector3f& point) {
    origin = point;
  }

  void setProjectedOrigin(Eigen::Vector3f& point){
    origin = projectPoint(point);
  }

  void setOrientation(Eigen::Vector3f& vec){
    orientation = vec.normalized();
  }

  Eigen::Vector3f projectPoint(const Eigen::Vector3f& point){
    return pointToLineProjection<float>(point, origin, origin + orientation);
  }

  float pointDistance(const Eigen::Vector3f& point){
    return pointToLineNorm<float>(point, origin, origin + orientation);
  }

  Eigen::Matrix3f getRotationMatrix(float angle){
    return Eigen::AngleAxisf(angle, orientation).toRotationMatrix();
  }

  Eigen::Vector3f rotatePoint(const Eigen::Vector3f& point, float angle){
    Eigen::Vector3f projectedPoint = projectPoint(point);
    return projectedPoint + getRotationMatrix(angle) * (point - projectedPoint);
  }

  template<typename PointT>
  void rotateCloud(typename pcl::PointCloud<PointT>& cloud_in, typename pcl::PointCloud<PointT>& cloud_out, float angle){
    pcl::copyPointCloud(cloud_in, cloud_out);

    for(size_t it = 0; it < cloud_in.points.size(); it++){
      cloud_out.points[it].getVector3fMap() = rotatePoint(cloud_in.points[it].getVector3fMap(), angle);
    }
  }

  template<typename PointT>
  void rotateCloud(typename pcl::PointCloud<PointT>& cloud_in, std::vector<int>& indices, typename pcl::PointCloud<PointT>& cloud_out, float angle){
    pcl::copyPointCloud(cloud_in, indices, cloud_out);

    for(size_t it = 0; it < indices.size(); it++){
      cloud_out.points[indices[it]].getVector3fMap() = rotatePoint(cloud_in.points[indices[it]].getVector3fMap(), angle);
    }
  }

  template<typename PointT>
  inline void populateCloud(typename pcl::PointCloud<PointT>& cloud_in,
                            std::vector<int>& indices,
                            typename pcl::PointCloud<PointT>& cloud_out,
                            float step_angle = M_PI / 4)
  {
    cloud_out.clear();
    int redundant_factor = static_cast<int>(M_PI * 2 / step_angle) + 1;

    pcl::PointCloud<PointT> rotatedCloud;
    for(size_t it = 0;  it < redundant_factor; it++){
      rotateCloud(cloud_in, indices, rotatedCloud, step_angle * it);
      cloud_out += rotatedCloud;
    }
  }

  template<typename PointT>
  inline void populateCloud(typename pcl::PointCloud<PointT>& cloud_in,
                            typename pcl::PointCloud<PointT>& cloud_out,
                            float step_angle = M_PI / 4)
  {
    std::vector<int> indices(cloud_in.points.size());
    for(size_t it = 0; it < cloud_in.points.size(); it++)
      indices[it] = it;

    populateCloud(cloud_in, indices, cloud_out, step_angle);
  }

  inline void getRotSymDifference(const RotationalSymmetry& target, float& angle, float& dist){
    angle = lineLineAngle<float>(orientation, target.getOrientation());
    dist = lineToLineNorm<float>(origin, origin + orientation, target.getOrigin(), target.getOrigin() + target.getOrientation());
  }

};

inline float getRotSymFitError(Eigen::Vector3f& point,
                               Eigen::Vector3f& normal,
                               RotationalSymmetry& symmetry)
{
  Eigen::Vector3f projectedPoint = symmetry.projectPoint(point);
  Eigen::Vector3f planeNormal = (point - projectedPoint).cross(symmetry.getOrientation());

  float angle = std::abs(planeNormal.dot(normal) / planeNormal.norm());
  angle = clamp(angle, 0.0f, 1.0f);

  return std::asin(angle);
}

inline float getRotSymPerpendicularity(Eigen::Vector3f& normal,
                                       RotationalSymmetry& symmetry,
                                       float threshold = M_PI / 2)
{
  if(threshold == 0.0)
    return 1.0f;

  float angle = lineLineAngle(symmetry.getOrientation(), normal);

  angle /= threshold;
  angle = std::min(angle, 1.0f);

  return 1.0f - angle;
}



#endif
