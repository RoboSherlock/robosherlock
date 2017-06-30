#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <eigen3/Eigen/Dense>
#include <cmath>


template<typename Type>
inline Type clamp(Type val, Type minVal, Type maxVal){
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
inline void planeToPointNormal(const Eigen::Matrix<Type, 4, 1>& plane, Eigen::Matrix<Type, 3, 1>& point, Eigen::Matrix<Type, 3, 1>& normal){
  normal << plane[0], plane[1], plane[2];
  Type denom = normal.norm();
  normal /= denom; // normalize plane normal
  point = normal * (- plane[3] / denom);
}

template<class Type>
inline void pointNormalToPlane(const Eigen::Matrix<Type, 3, 1>& point, const Eigen::Matrix<Type, 3, 1>& normal, Eigen::Matrix<Type, 4, 1>& plane){
  plane.head(3) = normal;
  plane(3) = -normal.dot(point);
}

template<class Type>
inline Type pointToPlaneSignedNorm(const Eigen::Matrix<Type, 3, 1>& point, const Eigen::Matrix<Type, 3, 1>& planePoint, const Eigen::Matrix<Type, 3, 1>& planeNormal){
  Eigen::Matrix<Type, 3, 1> line = point - planePoint;
  return line.dot(planeNormal);
}

template<class Type>
inline Type pointToPlaneSignedNorm(const Eigen::Matrix<Type, 3, 1>& point, const Eigen::Matrix<Type, 4, 1>& plane){
  Eigen::Vector3f planePoint, planeNormal;
  planeToPointNormal(plane, planePoint, planeNormal);
  return pointToPlaneSignedNorm(point, planePoint, planeNormal);
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

template<class Type>
inline Type vecVecAngle(const Eigen::Matrix<Type, 3, 1>& v1,
                        const Eigen::Matrix<Type, 3, 1>& v2
                                  )
{
  v1.normalized();
  v2.normalized();
  return std::acos(clamp(v1.dot(v2), -1.0f, 1.0f));
}

//vecter vector angle clockwise
template<class Type>
inline Type vecVecAngleClockwise(const Eigen::Matrix<Type, 3, 1>& v1,
                                 const Eigen::Matrix<Type, 3, 1>& v2,
                                 const Eigen::Matrix<Type, 3, 1>& normal
                                  )
{
  v1.normalized();
  v2.normalized();
  Type denom = v1.dot(v2);
  Type nom = clamp(normal.dot(v1.cross(v2)), -1.0f, 1.0f);
  return std::atan2(nom, denom);
}

template<class Type>
inline Type lineLineAngle(const Eigen::Matrix<Type, 3, 1>& v1,
                        const Eigen::Matrix<Type, 3, 1>& v2
                                  ){
  return std::abs(vecVecAngle<Type>(v1, v2));
}

//This function return bounded angle in 2 * PI
template<class Type>
inline Type angleDifferent(const Type angle1, const Type angle2){
  Type result = angle2 - angle1;
  if(result < 0)
    result += 2 * M_PI;
  return result;
}


#endif
