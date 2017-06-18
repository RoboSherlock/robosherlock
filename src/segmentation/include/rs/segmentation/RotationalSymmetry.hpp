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

  void setOrigin(Eigen::Vector3f point) {
    origin = point;
  }

  void setOrientation(Eigen::Vector3f vec){
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

  Eigen::Vector3f rotatePoint(Eigen::Vector3f& point, float angle){
    Eigen::Vector3f projectedPoint = projectPoint(point);
    return projectedPoint + getRotationMatrix(angle) * (point - projectedPoint);
  }

  template<typename PointT>
  void rotateCloud(typename pcl::PointCloud<PointT>& cloud_in, typename pcl::PointCloud<PointT>& cloud_out, float angle){
    pcl::copyPointCloud(cloud_in, cloud_out);

    for(size_t it = 0; it < cloud_in.points.size(); it++){
      cloud_out.points[it] = rotatePoint(cloud_in.points[it].getVector3fMap(), angle);
    }
  }

  template<typename PointT>
  void rotateCloud(typename pcl::PointCloud<PointT>& cloud_in, std::vector<int>& indices, typename pcl::PointCloud<PointT>& cloud_out, float angle){
    pcl::copyPointCloud(cloud_in, indices, cloud_out);

    for(size_t it = 0; it < indices.size(); it++){
      cloud_out.points[indices[it]] = rotatePoint(cloud_in.points[indices[it]].getVector3fMap(), angle);
    }
  }
};

#endif
