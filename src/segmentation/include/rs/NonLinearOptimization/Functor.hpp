#ifndef FUNCTOR_HPP
#define FUNCTOR_HPP

#include <unsupported/Eigen/NonLinearOptimization>

#include <rs/segmentation/RotationalSymmetry.hpp>
#include <rs/segmentation/BilateralSymmetry.hpp>
#include <rs/occupancy_map/DistanceMap.hpp>

#include <pcl/registration/correspondence_rejection_one_to_one.h>

//NOTE: This class was implemented for compatible use of Eigen NonLinearOptimization module
template <typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct OptimizationFunctor{
  typedef _Scalar Scalar; // for compatible purpose with NumericalDiff class

  enum{
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };

  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  int m_inputs, m_outputs;

  OptimizationFunctor() : m_inputs(InputsAtCompileTime), m_outputs(ValuesAtCompileTime) {}
  OptimizationFunctor(int inputs, int outputs) : m_inputs(inputs), m_outputs(outputs) {}

  int inputs() const {return m_inputs;}
  int values() const {return m_outputs;}
};


template<typename PointT>
struct RotSymOptimizeFunctor : OptimizationFunctor<float>
{

  typename pcl::PointCloud<PointT>::Ptr cloud;
  typename pcl::PointCloud<pcl::Normal>::Ptr normals;
  float max_fit_angle;

  RotSymOptimizeFunctor() : max_fit_angle(1.0f) {}

  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const{
    RotationalSymmetry symmetry(x.head(3), x.tail(3));

    for(size_t it = 0; it < cloud->points.size(); it++){
      Eigen::Vector3f point = cloud->points[it].getVector3fMap();
      Eigen::Vector3f normal( normals->points[it].normal_x, normals->points[it].normal_y, normals->points[it].normal_z);
      float angle = getRotSymFitError(point, normal, symmetry);

      fvec(it) = std::min(angle, max_fit_angle);
    }

    return 0;
  }

  int inputs() const { return 6; }
  int values() const { return this->cloud->points.size(); }
};

template <typename PointT>
struct RotSymOptimizeFunctorDiff : Eigen::NumericalDiff< RotSymOptimizeFunctor<PointT> > {};

template<typename PointT>
struct BilSymOptimizeFunctor : OptimizationFunctor<float>
{
  typename pcl::PointCloud<PointT>::Ptr cloud;
  typename pcl::PointCloud<pcl::Normal>::Ptr normals;
  typename pcl::PointCloud<PointT>::Ptr dsCloud;

  DistanceMap<PointT> distMap;
  pcl::Correspondences correspondences;

  BilSymOptimizeFunctor() {};

  int inputs() const { return 6; }
  int values() const { return this->correspondences.size(); }

  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
  {
    BilateralSymmetry symmetry(x.head(3), x.tail(3));

    for(size_t it = 0; it < this->correspondences.size(); it++)
    {
      int srcPointId = correspondences[it].index_query;
      int tgtPointId = correspondences[it].index_match;

      Eigen::Vector3f srcPoint = dsCloud->points[srcPointId].getVector3fMap();

      Eigen::Vector3f tgtPoint = cloud->points[tgtPointId].getVector3fMap();
      Eigen::Vector3f tgtNormal(normals->points[tgtPointId].normal_x, normals->points[tgtPointId].normal_y, normals->points[tgtPointId].normal_z);

      Eigen::Vector3f reflectedTgtPoint = symmetry.reflectPoint(tgtPoint);
      Eigen::Vector3f reflectedTgtNormal = symmetry.reflectPoint(tgtNormal);

      fvec(i) = std::abs(pointToPlaneSignedNorm<float>(srcPoint, reflectedTgtPoint, reflectedTgtNormal));
    }

    return 0;
  }
};

template <typename PointT>
struct BilSymOptimizeFunctorDiff : Eigen::NumericalDiff< BilSymOptimizeFunctor<PointT> > {};

#endif
