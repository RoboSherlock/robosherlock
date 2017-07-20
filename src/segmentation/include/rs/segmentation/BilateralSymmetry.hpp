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
    planeToPointNormal(plane, origin, normal);
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
    return pointToPlaneProjection(point, origin, normal);
  }

  inline void setProjectedOrigin(const Eigen::Vector3f& point){
    origin = this->projectPoint(point);
  }

  inline float pointSignedDist(const Eigen::Vector3f& point) const{
    return pointToPlaneSignedNorm(point, origin, normal);
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

#endif
