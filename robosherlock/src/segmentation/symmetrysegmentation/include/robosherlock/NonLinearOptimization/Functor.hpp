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

#ifndef __FUNCTOR_HPP__
#define __FUNCTOR_HPP__

#include <unsupported/Eigen/NonLinearOptimization>

#include <robosherlock/symmetrysegmentation/RotationalSymmetry.hpp>
#include <robosherlock/symmetrysegmentation/BilateralSymmetry.hpp>
#include <robosherlock/mapping/DistanceMap.hpp>

#include <pcl/registration/correspondence_rejection_one_to_one.h>

/** \brief This supper struct (functor) was implemented for compatible use of Eigen NonLinearOptimization module.
 *  Each model of non linear optimization must be overrived its own operator() as a child of this class, based on choosen Scalar*/
template <typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct OptimizationFunctor
{

  /** \brief Type for compatible purpose with Eigen::NumericalDiff class. It represents type of variable for optimization (float, symmetries, etc) */
  typedef _Scalar Scalar;

  enum
  {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };

  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  int m_inputs, m_outputs;

  OptimizationFunctor() : m_inputs(InputsAtCompileTime), m_outputs(ValuesAtCompileTime) {}
  OptimizationFunctor(int inputs, int outputs) : m_inputs(inputs), m_outputs(outputs) {}

  int inputs() const { return m_inputs; }
  int values() const { return m_outputs; }
};

/** \brief Child struct (functor) of OptimizationFunctor, it is the model for optimizing RotationalSymmetry poses.
 *  Given 3D pointcloud with normals and an initial 3D rotational symmetry axis refine the symmetry axis such that
 * it minimizes the error of fit between the symmetry and the points.
 * \note The result optimizing may have non-unit normals
 */
template<typename PointT>
struct RotSymOptimizeFunctor : OptimizationFunctor<float>
{
  /** \brief input cloud. */
  typename pcl::PointCloud<PointT>::Ptr cloud;

  /** \brief input normals. */
  typename pcl::PointCloud<pcl::Normal>::Ptr normals;

  /** \brief max fit error of symmetry and point normal. */
  float max_fit_angle;

  /** \brief Default constructor. */
  RotSymOptimizeFunctor() : max_fit_angle(1.0f) {}

  /** \brief overrived method of optimization for RotationalSymmetry, it it minimizes the error of fit between the symmetry and the points */
  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
  {
    RotationalSymmetry symmetry(x.head(3), x.tail(3));

    for(size_t it = 0; it < cloud->points.size(); it++)
    {
      Eigen::Vector3f point = cloud->points[it].getVector3fMap();
      Eigen::Vector3f normal( normals->points[it].normal_x, normals->points[it].normal_y, normals->points[it].normal_z);
      float angle = getRotSymFitError(point, normal, symmetry);

      fvec(it) = std::min(angle, max_fit_angle);
    }

    return 0;
  }

  /** \brief Dimension of input parameter, in this case: 6 for position and orientation of RotationalSymmetry. */
  int inputs() const { return 6; }

  /** \brief Cloud point size. */
  int values() const { return this->cloud->points.size(); }
};

/** \brief A dumb struct interfaces with Eigen::NonLinearOptimization */
template <typename PointT>
struct RotSymOptimizeFunctorDiff : Eigen::NumericalDiff< RotSymOptimizeFunctor<PointT> > {};


/** \brief Child struct (functor) of OptimizationFunctor, it is the model for optimizing BilateralSymmetry poses.
 *  Given 3D original cloud and downsampled cloud with normals and an initial 3D bilateral symmetry, refine the symmetry normal such that
 * it minimizes the error of distance between downsampled point to plane of reflected original point and normal.
 * \note The result optimizing may have non-unit normals
 */
template<typename PointT>
struct BilSymOptimizeFunctor : OptimizationFunctor<float>
{
  typename pcl::PointCloud<PointT>::Ptr cloud;
  typename pcl::PointCloud<pcl::Normal>::Ptr normals;
  typename pcl::PointCloud<PointT>::Ptr dsCloud;

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

      fvec(it) = std::abs(pointToPlaneSignedNorm<float>(srcPoint, reflectedTgtPoint, reflectedTgtNormal));
    }

    return 0;
  }
};

/** \brief A dumb struct interfaces with Eigen::NonLinearOptimization */
template <typename PointT>
struct BilSymOptimizeFunctorDiff : Eigen::NumericalDiff< BilSymOptimizeFunctor<PointT> > {};

#endif // __FUNCTOR_HPP__
