/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE. 
 *
 *
 *
 */

#include <pcl/segmentation/boost.h>
#include <pcl/segmentation/comparator.h>
#include <stdlib.h>
#include <algorithm> 
#include <cmath>

namespace pcl
{
 /** \brief ValueClusterComparator is a comparator to extract clusters based on euclidean distance + value threshold
   * This needs to be run as a second pass after extracting planar surfaces, using MultiPlaneSegmentation for example.
   *
   * \author Tobias Hahn
   */
 template<typename PointT, typename PointNT>
 class ValueClusterComparator: public Comparator<PointT>
 {
   public:
     typedef typename Comparator<PointT>::PointCloud PointCloud;
     typedef typename Comparator<PointT>::PointCloudConstPtr PointCloudConstPtr;
     
     typedef typename pcl::PointCloud<PointNT> PointCloudN;
     typedef typename PointCloudN::Ptr PointCloudNPtr;
     typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;
     
     typedef boost::shared_ptr<ValueClusterComparator<PointT, PointNT> > Ptr;
     typedef boost::shared_ptr<const ValueClusterComparator<PointT, PointNT> > ConstPtr;

     using pcl::Comparator<PointT>::input_;
     
     /** \brief Empty constructor for ValueClusterComparator. */
     ValueClusterComparator ()
       : normals_ ()
       , angular_threshold_ (0.0f)
       , distance_threshold_ (0.005f)
       , depth_dependent_ ()
       , z_axis_ ()
       , value_threshold_ (0.1f)
	   , interval_ (0.09f)
	   , discretize_ (false)
     {
     }
     
     /** \brief Destructor for ValueClusterComparator. */
     virtual
     ~ValueClusterComparator ()
     {
     }

     virtual void 
     setInputCloud (const PointCloudConstPtr& cloud)
     {
       input_ = cloud;
       Eigen::Matrix3f rot = input_->sensor_orientation_.toRotationMatrix ();
       z_axis_ = rot.col (2);
     }
     
     /** \brief Provide a pointer to the input normals.
       * \param[in] normals the input normal cloud
       */
       inline void
       setInputNormals (const PointCloudNConstPtr &normals)
       {
         normals_ = normals;
       }

	   /** \brief Set the flag for discretization of value values
		* \param[in] The value to set it to
		*/
	   inline void setDiscretization(bool set) {
		discretize_ = set;	
	   }
 
       /** \brief Get the input normals. */
       inline PointCloudNConstPtr
       getInputNormals () const
       {
         return (normals_);
       }
 
       /** \brief Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.
         * \param[in] angular_threshold the tolerance in radians
         */
       virtual inline void
       setAngularThreshold (float angular_threshold)
       {
         angular_threshold_ = cosf (angular_threshold);
       }
       
       /** \brief Get the angular threshold in radians for difference in normal direction between neighboring points, to be considered part of the same plane. */
       inline float
       getAngularThreshold () const
       {
         return (acos (angular_threshold_) );
       }
 
       /** \brief Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.
         * \param[in] distance_threshold the tolerance in meters 
         * \param depth_dependent
         */
       inline void
       setDistanceThreshold (float distance_threshold, bool depth_dependent)
       {
         distance_threshold_ = distance_threshold;
         depth_dependent_ = depth_dependent;
       }
 
       /** \brief Get the distance threshold in meters (d component of plane equation) between neighboring points, to be considered part of the same plane. */
       inline float
       getDistanceThreshold () const
       {
         return (distance_threshold_);
       }

       /** \brief Set the tolerance in points for difference in hue between neighboring points, to be considered part of the same plane.
         * \param[in] hue_threshold the tolerance in points
         */
       inline void
       setValueThreshold (float value_threshold)
       {
         value_threshold_ = value_threshold;
		 interval_ = value_threshold - 0.01;
		 if (interval_ <= 0) {
		 	outInfo("Interval for discretization too low, set to 0.01!");
			interval_ = 0.01;
		 }
       }
 
       /** \brief Get the hue threshold in points between neighboring points, to be considered part of the same plane. */
       inline float
       getValueThreshold () const
       {
         return (value_threshold_);
       }
 
       /** \brief Compare points at two indices by their plane equations.  True if the angle between the normals is less than the angular threshold,
         * and the difference between the d component of the normals is less than distance threshold, else false
         * \param idx1 The first index for the comparison
         * \param idx2 The second index for the comparison
         */
       virtual bool
       compare (int idx1, int idx2) const
       {
		 float value1 = input_->points[idx1].v;
		 float value2 = input_->points[idx2].v;
		 if (discretize_) {
			value1 = value1 - fmod(value1, interval_);
			value2 = value2 - fmod(value2, interval_);
		 }
		 float diff = std::abs(value1 - value2);

		 if (value_threshold_ < diff)
	   		return false;
         
         
         float dist_threshold = distance_threshold_;
         if (depth_dependent_)
         {
           Eigen::Vector3f vec = input_->points[idx1].getVector3fMap ();
           float z = vec.dot (z_axis_);
           dist_threshold *= z * z;
         }
 
         float dx = input_->points[idx1].x - input_->points[idx2].x;
         float dy = input_->points[idx1].y - input_->points[idx2].y;
         float dz = input_->points[idx1].z - input_->points[idx2].z;
         float dist = sqrtf (dx*dx + dy*dy + dz*dz);
 
         return (dist < dist_threshold);
       }
       
     protected:
       PointCloudNConstPtr normals_;
 
       float angular_threshold_;
       float distance_threshold_;
       bool depth_dependent_;
       Eigen::Vector3f z_axis_;
       float value_threshold_;
	   float interval_;
	   bool discretize_;
   };
 }

