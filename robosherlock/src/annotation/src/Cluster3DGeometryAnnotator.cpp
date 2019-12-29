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

/*Annotator for geting basic attributes of PointClouds. Currently:
  *-initialy pose guess based on centroid
  *-semantical size annotation (big or small)
  *-3D bounding box
  */

#include <uima/api.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/output.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/utils/common.h>
#include <robosherlock/DrawingAnnotator.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG

using namespace uima;

class Cluster3DGeometryAnnotation : public DrawingAnnotator
{
private:
  typedef pcl::PointXYZRGBA PointT;

  struct OrientedBoundingBox
  {
    tf::Transform objectToWorld;
    float width, depth, height, volume;

    std::string semanticSize;
    tf::Stamped<tf::Pose> poseCam, poseWorld;
    cv::Rect rect_;
  };

  cv::Mat disp;
  pcl::PointCloud<PointT>::Ptr dispCloud;
  double pointSize;
  std::vector<OrientedBoundingBox> orientedBoundingBoxes;
  tf::StampedTransform camToWorld, worldToCam;
  std::vector<float> plane_model;
  bool projectOnPlane_, overwriteExistingPoseEstimate_, sorFilter_;

public:

  Cluster3DGeometryAnnotation(): DrawingAnnotator(__func__), pointSize(1), projectOnPlane_(false), overwriteExistingPoseEstimate_(false), sorFilter_(false)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("projectOnPlane"))
    {
      ctx.extractValue("projectOnPlane", projectOnPlane_);
    }
    if(ctx.isParameterDefined("estimateAll"))
    {
      ctx.extractValue("overwriteExistingPoseEstimate", overwriteExistingPoseEstimate_);
    }
    if(ctx.isParameterDefined("sor_filter"))
    {
      ctx.extractValue("sor_filter", sorFilter_);
    }


    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");

    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>());

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    std::vector<rs::Plane> planes;

    cas.get(VIEW_CLOUD, *cloud_ptr);
    dispCloud = cloud_ptr;
    cas.get(VIEW_COLOR_IMAGE, disp);

    scene.annotations.filter(planes);
    if(planes.empty())
    {
      return UIMA_ERR_ANNOTATOR_MISSING_INFO;
    }

    tf::StampedTransform head_to_map;
    rs::conversion::from(scene.viewPoint.get(), head_to_map);
    plane_model = planes[0].model();

    scene.identifiables.filter(clusters);

    orientedBoundingBoxes.resize(clusters.size());

    camToWorld.setIdentity();
    if(scene.viewPoint.has())
    {
      rs::conversion::from(scene.viewPoint.get(), camToWorld);
    }
    else
    {
      outInfo("No camera to world transformation!!!");
    }
    worldToCam = tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);
    Eigen::Affine3d eigenTransform;
    tf::transformTFToEigen(camToWorld, eigenTransform);

    //iterate over clusters
    #pragma omp parallel for
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::ObjectHypothesis &cluster = clusters[i];
      if(!cluster.points.has())
      {
        continue;
      }
      pcl::PointIndicesPtr indices(new pcl::PointIndices());
      rs::conversion::from(static_cast<rs::ReferenceClusterPoints>(cluster.points.get()).indices.get(), *indices);

      pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
      pcl::PointCloud<PointT>::Ptr cluster_transformed(new pcl::PointCloud<PointT>());
      pcl::ExtractIndices<PointT> ei;
      ei.setInputCloud(cloud_ptr);
      ei.setIndices(indices);
      ei.filter(*cluster_cloud);

      if(sorFilter_)
      {
        outDebug("Before SOR filter: " << cluster_cloud->points.size());
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cluster_cloud);
        sor.setMeanK(100);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cluster_cloud);
        outDebug("After SOR filter: " << cluster_cloud->points.size());
      }

      //transform Point Cloud to map coordinates
      pcl::transformPointCloud<PointT>(*cluster_cloud, *cluster_transformed, eigenTransform);

      OrientedBoundingBox &box = orientedBoundingBoxes[i];
      rs::conversion::from(cluster.rois().roi.get(),box.rect_);

      //computeBoundingBoxPCA(cluster_transformed, box);
      computeBoundingBoxMinArea(cluster_transformed, box);
      //computeBoundingBoxMoments(cluster_transformed, box);

      computeSemnaticSize(box);
      computePose(box);
      drawImage(box);
    }

    for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::ObjectHypothesis &cluster = clusters[i];
      OrientedBoundingBox &box = orientedBoundingBoxes[i];

      rs::BoundingBox3D box3d = rs::create<rs::BoundingBox3D>(tcas);
      box3d.volume(box.volume);
      box3d.width(box.width);
      box3d.depth(box.depth);
      box3d.height(box.height);

      rs::Geometry geometry = rs::create<rs::Geometry>(tcas);
      geometry.boundingBox(box3d);

      double dist = std::fabs(pcl::pointToPlaneDistanceSigned(pcl::PointXYZ(static_cast<float>(box.poseCam.getOrigin().x()),
                                                                            static_cast<float>(box.poseCam.getOrigin().y()),
                                                                            static_cast<float>(box.poseCam.getOrigin().z())),
                                                              static_cast<double>(plane_model[0]), static_cast<double>(plane_model[1]),
                                                              static_cast<double>(plane_model[2]), static_cast<double>(plane_model[3])));
      geometry.distanceToPlane.set(dist);
      cluster.annotations.append(geometry);

      rs::SemanticSize semSize = rs::create<rs::SemanticSize>(tcas);;
      semSize.source.set("Cluster3DGeometryAnnotator");

      float lowerThreshold = 0.0012f,
            middleThreshold = 0.004f,
            largestObjVolume = 0.125;


      if(box.volume < lowerThreshold)
      {
        semSize.size.set("small");
        semSize.confidence.set(std::abs(lowerThreshold / 2 - box.volume) / (lowerThreshold / 2));
      }
      else if(box.volume < middleThreshold)
      {
        semSize.size.set("medium");
        semSize.confidence.set(std::abs((middleThreshold - lowerThreshold) / 2 - box.volume) / (middleThreshold - lowerThreshold) / 2);
      }
      else   //if(box.volume < 0.02)
      {
        semSize.size.set("large");
        semSize.confidence.set(std::abs((largestObjVolume - middleThreshold) / 2 - box.volume) / (largestObjVolume - middleThreshold) / 2);
      }
      cluster.annotations.append(semSize);

      std::vector<rs::PoseAnnotation> poses;
      cluster.annotations.filter(poses);

      if(projectOnPlane_)
      {
        rs::common::projectPointOnPlane(box.poseCam, plane_model);
        tf::Transform transform(box.poseCam.getRotation(), box.poseCam.getOrigin());
        box.poseWorld = tf::Stamped<tf::Pose>(camToWorld * transform, camToWorld.stamp_, camToWorld.frame_id_);
      }

      if(poses.empty())
      {
        rs::PoseAnnotation poseAnnotation = rs::create<rs::PoseAnnotation>(tcas);
        poseAnnotation.source.set("3DEstimate");
        poseAnnotation.camera.set(rs::conversion::to(tcas, box.poseCam));
        poseAnnotation.world.set(rs::conversion::to(tcas, box.poseWorld));
        cluster.annotations.append(poseAnnotation);
      }
      else if(overwriteExistingPoseEstimate_)
      {
        poses[0].source.set("3DEstimate");
        poses[0].camera.set(rs::conversion::to(tcas, box.poseCam));
        poses[0].world.set(rs::conversion::to(tcas, box.poseWorld));
      }


    }
    return UIMA_ERR_NONE;
  }

  void project2D(const pcl::PointCloud<PointT>::ConstPtr &cloud, std::vector<cv::Point> &points, cv::Point3f &min, cv::Point3f &max) const
  {
    min = cv::Point3f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
    max = cv::Point3f(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
    points.resize(cloud->points.size() * 2);

    for(size_t i = 0; i < cloud->points.size(); ++i)
    {

      const PointT &point = cloud->points[i];

      if(point.x < min.x)
      {
        min.x = point.x;
      }
      else if(point.x > max.x)
      {
        max.x = point.x;
      }

      if(point.y < min.y)
      {
        min.y = point.y;
      }
      else if(point.y > max.y)
      {
        max.y = point.y;
      }

      if(point.z < min.z)
      {
        min.z = point.z;
      }
      else if(point.z > max.z)
      {
        max.z = point.z;
      }
    }

    cv::Point corner((max.x - min.x) * 1000, (max.y - min.y) * 1000);
    for(size_t i = 0, j = cloud->points.size(); i < cloud->points.size(); ++i, ++j)
    {
      const PointT &point = cloud->points[i];
      points[i] = cv::Point((point.x - min.x) * 1000, (point.y - min.y) * 1000);
      points[j] = corner - points[i];
    }
  }

  void computeBoundingBoxPCA(pcl::PointCloud<PointT>::Ptr cloud, OrientedBoundingBox &box)
  {
    pcl::PCA<PointT> pca;
    pcl::PointCloud<PointT> cloudProjected;

    pca.setInputCloud(cloud);
    pca.project(*cloud, cloudProjected);

    PointT proj_min, proj_max, min_pt, max_pt;
    pcl::getMinMax3D(cloudProjected, proj_min, proj_max);
    pca.reconstruct(proj_min, min_pt);
    pca.reconstruct(proj_max, max_pt);

    tf::Transform objectToWorld;

    tf::Vector3 trans;
    Eigen::Vector3d translation = pca.getMean().head(3).cast<double>();
    tf::vectorEigenToTF(translation, trans);
    objectToWorld.setOrigin(trans);

    Eigen::Quaterniond quaternion(pca.getEigenVectors().cast<double>());
    tf::Quaternion quat;
    tf::quaternionEigenToTF(quaternion, quat);
    objectToWorld.setRotation(quat);

    box.objectToWorld = objectToWorld;
    box.width = fabs(proj_max.x - proj_min.x);
    box.height = fabs(proj_max.y - proj_min.y);
    box.depth = fabs(proj_max.z - proj_min.z);
    box.volume = box.width * box.depth * box.height;
  }

  void computeBoundingBoxMinArea(pcl::PointCloud<PointT>::Ptr cloud, OrientedBoundingBox &box) const
  {
    cv::Point3f min, max;
    std::vector<cv::Point> points;

    project2D(cloud, points, min, max);

    cv::RotatedRect rect = cv::minAreaRect(points);
    if(rect.size.width < rect.size.height)
    {
      rect.angle += 90;
      rect.size = cv::Size2f(rect.size.height, rect.size.width);
    }

    tf::Vector3 trans = tf::Vector3((max.x + min.x) / 2.0, (max.y + min.y) / 2.0, (max.z + min.z) / 2.0);
    float sinA, cosA;

    sinA = sin(rect.angle / 180.0 * M_PI);
    cosA = cos(rect.angle / 180.0 * M_PI);

    tf::Matrix3x3 rot;
    rot.setValue(cosA, -sinA, 0, sinA, cosA,  0, 0, 0, 1);

    tf::Transform objectToWorld;
    objectToWorld.setOrigin(trans);
    objectToWorld.setBasis(rot);

    box.objectToWorld = objectToWorld;
    box.width = rect.size.width / 1000.0;
    box.height = rect.size.height / 1000.0;
    box.depth = max.z - min.z;
    box.volume = box.width * box.depth * box.height;
  }

  void computeBoundingBoxMoments(pcl::PointCloud<PointT>::Ptr cloud, OrientedBoundingBox &box) const
  {
    cv::Point3f min, max;
    std::vector<cv::Point> points;

    project2D(cloud, points, min, max);

    size_t rows = (int)((max.y - min.y) * 1000) + 1;
    size_t cols = (int)((max.x - min.x) * 1000) + 1;
    cv::Mat img = cv::Mat::zeros(rows, cols, CV_8U);

    for(size_t i = 0; i < points.size(); ++i)
    {
      const cv::Point &p = points[i];
      img.at<uint8_t>(p.y, p.x) += 1;
    }

    cv::Moments moments = cv::moments(img, false);

    double x = moments.m10 / moments.m00;
    double y = moments.m01 / moments.m00;
    tf::Vector3 trans((max.x + min.x) / 2.0, (max.y + min.y) / 2.0, (max.z + min.z) / 2.0);

    double alpha = 0.5 * atan2(2 * moments.mu11, moments.mu20 - moments.mu02);
    double sinA = sin(alpha);
    double cosA = cos(alpha);

    tf::Matrix3x3 rot;
    rot.setValue(cosA, -sinA, 0, sinA, cosA, 0, 0, 0, 1);

    double xx = (moments.m20 / moments.m00) - (x * x);
    double xy = (moments.m11 / moments.m00) - (x * y);
    double yy = (moments.m02 / moments.m00) - (y * y);

    double sin2 = sinA * sinA;
    double cos2 = cosA * cosA;
    double cs2xy = 2 * sinA * cosA * xy;
    double lengthX = 2 * sqrt(cos2 * xx + cs2xy + sin2 * yy);
    double lengthY = 2 * sqrt(sin2 * xx - cs2xy + cos2 * yy);

    tf::Transform objectToWorld;
    objectToWorld.setOrigin(trans);
    objectToWorld.setBasis(rot);

    //transform = worldToCam * transform;

    box.objectToWorld = objectToWorld;
    box.width = lengthX / 500.0;
    box.height = lengthY / 500.0;
    box.depth = max.z - min.z;
    box.volume = box.width * box.depth * box.height;
  }

  void computeSemnaticSize(OrientedBoundingBox &box) const
  {
    if(box.volume < 0.0012)
    {
      box.semanticSize = "small";
    }
    else if(box.volume < 0.004)
    {
      box.semanticSize = "medium";
    }
    else   //if(box.volume < 0.02)
    {
      box.semanticSize = "large";
    }
    /*else
    {
      box.semanticSize = "extra large";
    }*/
  }

  void computePose(OrientedBoundingBox &box) const
  {
    //compute camera and world pose
    box.poseCam = tf::Stamped<tf::Pose>(worldToCam * box.objectToWorld, camToWorld.stamp_, camToWorld.child_frame_id_);
    box.poseWorld = tf::Stamped<tf::Pose>(box.objectToWorld, camToWorld.stamp_, camToWorld.frame_id_);
  }

  void drawImage(OrientedBoundingBox &box)
  {
    cv::rectangle(disp, box.rect_,cv::Scalar(0,0,255));
    cv::putText(disp,box.semanticSize,cv::Point(box.rect_.x,box.rect_.y-10),CV_FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(255,255,255));
  }

  void drawImageWithLock(cv::Mat &d)
  {
    d = disp.clone();
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;

    if(firstRun)
    {
      visualizer.addPointCloud(dispCloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(dispCloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.removeAllShapes();
    }

    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0.2, 0, 0), 1, 0, 0, "X");
    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0.2, 0), 0, 1, 0, "Y");
    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 0.2), 0, 0, 1, "Z");

    tf::Vector3 origin = worldToCam * tf::Vector3(0, 0, 0);
    tf::Vector3 lineX = worldToCam * tf::Vector3(0.2, 0, 0);
    tf::Vector3 lineY = worldToCam * tf::Vector3(0, 0.2, 0);
    tf::Vector3 lineZ = worldToCam * tf::Vector3(0, 0, 0.2);

    pcl::PointXYZ pclOrigin(origin.x(), origin.y(), origin.z());
    pcl::PointXYZ pclLineX(lineX.x(), lineX.y(), lineX.z());
    pcl::PointXYZ pclLineY(lineY.x(), lineY.y(), lineY.z());
    pcl::PointXYZ pclLineZ(lineZ.x(), lineZ.y(), lineZ.z());

    visualizer.addLine(pcl::PointXYZ(0, 0, 0), pclOrigin, 1, 1, 1, "line");
    visualizer.addLine(pclOrigin, pclLineX, 1, 0, 0, "lineX");
    visualizer.addLine(pclOrigin, pclLineY, 0, 1, 0, "lineY");
    visualizer.addLine(pclOrigin, pclLineZ, 0, 0, 1, "lineZ");

    for(int i = 0; i < orientedBoundingBoxes.size(); ++i)
    {
      OrientedBoundingBox &box = orientedBoundingBoxes[i];
      std::ostringstream oss;
      oss << "box_" << i;

      tf::Vector3 originB = box.poseCam * tf::Vector3(0, 0, 0);
      tf::Vector3 lineXB = box.poseCam * tf::Vector3(0.2, 0, 0);
      tf::Vector3 lineYB = box.poseCam * tf::Vector3(0, 0.2, 0);
      tf::Vector3 lineZB = box.poseCam * tf::Vector3(0, 0, 0.2);

      pcl::PointXYZ pclOriginB(originB.x(), originB.y(), originB.z());
      pcl::PointXYZ pclLineXB(lineXB.x(), lineXB.y(), lineXB.z());
      pcl::PointXYZ pclLineYB(lineYB.x(), lineYB.y(), lineYB.z());
      pcl::PointXYZ pclLineZB(lineZB.x(), lineZB.y(), lineZB.z());

      visualizer.addLine(pclOrigin, pclOriginB, 1, 1, 1, "line_" + oss.str());
      visualizer.addLine(pclOriginB, pclLineXB, 1, 0, 0, "lineX_" + oss.str());
      visualizer.addLine(pclOriginB, pclLineYB, 0, 1, 0, "lineY_" + oss.str());
      visualizer.addLine(pclOriginB, pclLineZB, 0, 0, 1, "lineZ_" + oss.str());

      Eigen::Vector3d translation;
      Eigen::Quaterniond rotation;

      tf::vectorTFToEigen(box.poseCam.getOrigin(), translation);
      tf::quaternionTFToEigen(box.poseCam.getRotation(), rotation);

      visualizer.addCube(translation.cast<float>(), rotation.cast<float>(), box.width, box.height, box.depth, oss.str());
    }
  }
};
// This macro exports an entry point that is used to create the annotator.
MAKE_AE(Cluster3DGeometryAnnotation)
