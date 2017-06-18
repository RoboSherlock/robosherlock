#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <eigen3/Eigen/Dense>
#include <cmath>


template<typename Type>
inline Type clamp(const Type val, const Type minVal, const Type maxVal){
  Type result = val;
  result = std::max(minVal, result);
  result = std::min(maxVal, result);
  return result;
}

template<class Type>
inline Type pointToLineNorm(const Eigen::Matrix<Type, 3, 1>& point, const Eigen::Matrix<Type, 3, 1>& linePoint1, const Eigen::Matrix<Type, 3, 1>& linePoint2){
  return (point - linePoint1).cross(point - linePoint2).norm() / (linePoint2 - linePoint1).norm();
}

template<class Type>
inline Eigen::Matrix<Type, 3, 1> pointToLineProjection(const Eigen::Matrix<Type, 3, 1>& point, const Eigen::Matrix<Type, 3, 1>& linePoint1, const Eigen::Matrix<Type, 3, 1>& linePoint2){
  Eigen::Vector3f line = linePoint2 - linePoint1;
  return linePoint1 + (point - linePoint1).dot(line) * line / line.norm();
}

template<class Type>
inline Type pointToPlaneSignedNorm(const Eigen::Matrix<Type, 3, 1>& point, const Eigen::Matrix<Type, 3, 1>& planePoint, const Eigen::Matrix<Type, 3, 1>& planeNormal){
  Eigen::Matrix<Type, 3, 1> line = point - planePoint;
  return line.dot(planeNormal);
}

template<class Type>
inline Eigen::Matrix<Type, 3, 1> pointToPlaneProjection(const Eigen::Matrix<Type, 3, 1>& point, const Eigen::Matrix<Type, 3, 1>& planePoint, const Eigen::Matrix<Type, 3, 1>& planeNormal){
  Eigen::Matrix<Type, 3, 1> line = point - planePoint;
  return point - planeNormal * pointToPlaneSignedNorm(point, planePoint, planeNormal);
}

template<class Type>
inline Type lineToLineNorm(const Eigen::Matrix<Type, 3, 1>& line1Point1,
                                  const Eigen::Matrix<Type, 3, 1>& line1Point2,
                                  const Eigen::Matrix<Type, 3, 1>& line2Point1,
                                  const Eigen::Matrix<Type, 3, 1>& line2Point2,
                                  const Type eps = 1e-10){
  Eigen::Matrix<Type, 3, 1> line1 = line1Point1 - line1Point2;
  Eigen::Matrix<Type, 3, 1> line2 = line2Point1 - line2Point2;

  Type denom = line1.cross(line2).norm();
  if(denom < eps)
    return pointToLineNorm(line1Point1, line2Point1, line2Point2);

  Eigen::Matrix<Type, 3, 1> line3 = line1Point1 - line2Point1;
  return std::abs(line3.dot(line1.cross(line2))) / denom;
}


#endif
