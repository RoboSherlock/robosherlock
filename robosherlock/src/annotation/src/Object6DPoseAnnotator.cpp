/**
 * Copyright 2020 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Franklin Kenghagho Kenfack <fkenghag@cs.uni-bremen.de>
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
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>
#include <rs/utils/common.h>
#include <rs/DrawingAnnotator.h>
#include <rs/annotation/ObjectNameMapItem.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/package.h>
#include<boost/random.hpp>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG

using namespace uima;

class Object6DPoseAnnotator : public DrawingAnnotator
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
  int  icpMaximumIterations_, maxClusterPoints_, minClusterPoints_;
  float icpTransformationEpsilon_, icpMaxCorrespondenceDistance_,
        icpEuclideanFitnessEpsilon_, icpRANSACOutlierRejectionThreshold_,
        maxClusterDistance_;
  int intersectionDeviation_;
  std::string sceneObjectNameMap_;
  std::vector<ObjectNameMapItem> object_name_map_items;
  std::vector<std::string> target_object_names;
  std::vector<icu::UnicodeString> icp_domain_;
  std::vector<icu::UnicodeString> cls_domain_;

public:

  Object6DPoseAnnotator(): DrawingAnnotator(__func__), pointSize(1), projectOnPlane_(false), overwriteExistingPoseEstimate_(false), sorFilter_(false)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    icpMaximumIterations_=1000000;
    icpTransformationEpsilon_=1e-19;
    icpMaxCorrespondenceDistance_=1;
    icpEuclideanFitnessEpsilon_=9e-7;
    icpRANSACOutlierRejectionThreshold_=0.01;
    maxClusterPoints_=20000;
    minClusterPoints_=0;
    maxClusterDistance_=0.08;
    intersectionDeviation_=100;
    projectOnPlane_=false;
    sorFilter_=false;
    sceneObjectNameMap_="scene_object_name_map.yaml";
     printf("\nOK1\n");
    icp_domain_.clear();
    cls_domain_.clear();
     printf("\nOK2\n");
    if(ctx.isParameterDefined("icp_domain"))
    {
      ctx.extractValue("icp_domain", icp_domain_);
    }
    if(ctx.isParameterDefined("cls_domain"))
    {
      ctx.extractValue("cls_domain", cls_domain_);
    }
    if(ctx.isParameterDefined("sceneObjectNameMap"))
    {
      ctx.extractValue("sceneObjectNameMap", sceneObjectNameMap_);
    }
    if(ctx.isParameterDefined("icpMaximumIterations"))
    {
      ctx.extractValue("icpMaximumIterations", icpMaximumIterations_);
    }
    if(ctx.isParameterDefined("icpTransformationEpsilon"))
    {
      ctx.extractValue("icpTransformationEpsilon", icpTransformationEpsilon_);
    }
    if(ctx.isParameterDefined("icpMaxCorrespondenceDistance"))
    {
      ctx.extractValue("icpMaxCorrespondenceDistance", icpMaxCorrespondenceDistance_);
    }
    if(ctx.isParameterDefined("icpEuclideanFitnessEpsilon"))
    {
      ctx.extractValue("icpEuclideanFitnessEpsilon", icpEuclideanFitnessEpsilon_);
    }
    if(ctx.isParameterDefined("icpRANSACOutlierRejectionThreshold"))
    {
      ctx.extractValue("icpRANSACOutlierRejectionThreshold", icpRANSACOutlierRejectionThreshold_);
    }
    if(ctx.isParameterDefined("maxClusterPoints"))
    {
      ctx.extractValue("maxClusterPoints", maxClusterPoints_);
    }
    if(ctx.isParameterDefined("minClusterPoints"))
    {
      ctx.extractValue("minClusterPoints", minClusterPoints_);
    }
    if(ctx.isParameterDefined("maxClusterDistance"))
    {
      ctx.extractValue("maxClusterDistance", maxClusterDistance_);
    }
    if(ctx.isParameterDefined("intersectionDeviation"))
    {
      ctx.extractValue("intersectionDeviation", intersectionDeviation_);
    }
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
    //ROS_WARN("////////////////////// Cluster Size: %d //////////////////",clusters.size());
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
    Eigen::Affine3d eigenTransform,eigenTransform1;
    Eigen::Affine3d eigenTransformWtC;
    tf::transformTFToEigen(camToWorld, eigenTransform);
     tf::transformTFToEigen(camToWorld, eigenTransform1);
    tf::transformTFToEigen(worldToCam, eigenTransformWtC);
    std::vector<int> handables;
    for(size_t i = 0; i < clusters.size(); ++i)
        handables.push_back(-1);
    //iterate over clusters
    omp_set_nested(1);
    #pragma omp parallel for
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      Eigen::Affine3d eigenTransformobj;
      eigenTransformobj=eigenTransform;
      rs::ObjectHypothesis &cluster = clusters[i];
      if(!cluster.points.has())
      {
        continue;
      }
      std::string obj_name="";
      std::string obj_color="";
      std::vector<rs::Classification> classes;
      std::vector<rs::SemanticColor> colors;
      cluster.annotations.filter(classes);
      cluster.annotations.filter(colors);
      if(classes.size()<=0)
          continue;

      for(int r=0;r<classes.size();r++)
          if(classes[r].source.get()=="RobotVQA" && classes[r].featurename.get()==("type")){
              obj_name=classes[r].classname.get();
              break;
          }

      for(int r=0;r<colors.size();r++)
          if(colors[r].source.get()=="RobotVQA"){
              obj_color=colors[r].color.get();
              break;
          }

      std::string filename=ros::package::getPath("robosherlock")+"/config/"+sceneObjectNameMap_;
      readObjectNameMapItem(filename, target_object_names, object_name_map_items);
      ObjectNameMapItem item;
      for(int k=0;k<object_name_map_items.size();k++){
          item = object_name_map_items[k];
          if(obj_name.find(item.category)!=std::string::npos){
              obj_name=item.category;
              break;
          }
      }


      ROS_WARN("********* Object Name: %s ***************",obj_name.data());
      icu::UnicodeString object_name;
      object_name=object_name.fromUTF8(obj_name+";"+obj_color) ;

      pcl::PointIndicesPtr indices(new pcl::PointIndices());
      rs::conversion::from(static_cast<rs::ReferenceClusterPoints>(cluster.points.get()).indices.get(), *indices);

      pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
      pcl::PointCloud<PointT>::Ptr cluster_transformed(new pcl::PointCloud<PointT>());
      pcl::ExtractIndices<PointT> ei;
      ei.setInputCloud(cloud_ptr);
      ei.setIndices(indices);
      ei.filter(*cluster_cloud);
      std::vector<PointT> scene_points;

      if(cluster_cloud->size()>maxClusterPoints_ || cluster_cloud->size()<=minClusterPoints_ )
      {
        continue;
      }


      if(sorFilter_)
      {
          if((std::find(cls_domain_.begin(), cls_domain_.end(), object_name) != cls_domain_.end()))
              maxClusterDistance_=1.08;
      refinePointcloud(cluster_cloud, scene_points);
      cluster_cloud->points.clear();
      for(int i=0;i<scene_points.size();i++)
          cluster_cloud->points.push_back(scene_points.at(i));
      }

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudf (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudff (new pcl::PointCloud<pcl::PointXYZRGBA>);

      if(cluster_cloud->size()>maxClusterPoints_ || cluster_cloud->size()<=minClusterPoints_ )
      {
        continue;
      }


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

      if(cluster_cloud->size()>maxClusterPoints_ || cluster_cloud->size()<=minClusterPoints_ )
      {
        continue;
      }

      std::vector<int> indice;
      ROS_WARN("///////////////////////// 1. Size Franklin %d *********************",cluster_cloud->size());
      pcl::removeNaNFromPointCloud(*cluster_cloud, *cloud0, indice);
      ROS_WARN("///////////////////////// 1. Size Franklin %d *********************",cluster_cloud->size());
      cluster_cloud->clear();
      cluster_cloud=cloud0;
      ROS_WARN("///////////////////////// 1. Size Franklin %d *********************",cluster_cloud->size());
      /*********************************************************************************/

      if(cluster_cloud->size()>maxClusterPoints_ || cluster_cloud->size()<=minClusterPoints_ )
      {
        continue;
      }

      OrientedBoundingBox &box = orientedBoundingBoxes[i];
      if(!(std::find(icp_domain_.begin(), icp_domain_.end(), object_name) != icp_domain_.end())){
          //transform Point Cloud to map coordinates
          pcl::transformPointCloud<PointT>(*cluster_cloud, *cluster_transformed, eigenTransform1);
          OrientedBoundingBox &box = orientedBoundingBoxes[i];
          rs::conversion::from(cluster.rois().roi.get(),box.rect_);
          ROS_WARN("Hello Frank");
          if((std::find(cls_domain_.begin(), cls_domain_.end(), object_name) != cls_domain_.end()))
              computeBoundingBoxPCA2(cluster_transformed, box);
          else
              computeBoundingBoxMinArea(cluster_transformed, box);
          ROS_WARN("Bye Frank");
      }else{

          //centroid
          Eigen::Vector4f pcaCentroid;
          std::map<std::string,std::string>::iterator it;


          std::string cad_model_path="";
          if(item.fromPackage){
              cad_model_path=ros::package::getPath("rs_resources")+"/"+item.filePath;

          }
          ROS_WARN("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX %s",cad_model_path.data());
          if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (cad_model_path, *cloud1) == -1) //* load the file
            {
             ROS_WARN("++++++++++++++++++++++ POINTCLOUD NOT LOADED ********************");

            }
          else{
             ROS_WARN("++++++++++++++++++++++ POINTCLOUD LOADED: %d %d points +++++++",cloud1->points.size(),cloud_ptr->points.size());
            // cloud_ptr=cloud1;
             for(int l=0;l<cloud1->points.size();l++){
                 float scale=item.scale;
                 cloud1->points.at(l)._PointXYZRGBA::x/=scale;
                 cloud1->points.at(l)._PointXYZRGBA::y/=scale;
                 cloud1->points.at(l)._PointXYZRGBA::z/=scale;
             }
             pcl::compute3DCentroid(*cloud1, pcaCentroid);

          }

          /******************* Shift testing cloud *********************************/
          //centroid
          Eigen::Vector4f pcaCentroid1;
          pcl::compute3DCentroid(*cluster_cloud, pcaCentroid1);
          ROS_WARN("%f %f %f %f | %f %f %f %f",pcaCentroid.x(),pcaCentroid.y(),pcaCentroid.z(),pcaCentroid.w(),pcaCentroid1.x(),pcaCentroid1.y(),pcaCentroid1.z(),pcaCentroid1.w());
          for(int l=0;l<cloud1->points.size();l++){

              cloud1->points.at(l)._PointXYZRGBA::x+=(0*pcaCentroid1.x()-1*pcaCentroid.x());
              cloud1->points.at(l)._PointXYZRGBA::y+=(0*pcaCentroid1.y()-1*pcaCentroid.y());
              cloud1->points.at(l)._PointXYZRGBA::z+=(0*pcaCentroid1.z()-1*pcaCentroid.z());
              //cloud_ptr->points.push_back(cloud1->points.at(l));

          }
          //pcl::compute3DCentroid(*cloud1, pcaCentroid);
          //ROS_WARN("%f %f %f %f | %f %f %f %f",pcaCentroid.x(),pcaCentroid.y(),pcaCentroid.z(),pcaCentroid.w(),pcaCentroid1.x(),pcaCentroid1.y(),pcaCentroid1.z(),pcaCentroid1.w());



          /************* ICP *********************************/

          std::vector<Eigen::Affine3f> transformation_true;
          std::vector<Eigen::Matrix4f> listTransform;
          std::vector<Eigen::Affine3f> listTransformr;
          std::vector<float> scores;
          float min_score=50000;
          int index=-1;
          Eigen::Vector4f pcaCentroid2;
          Eigen::Matrix3f eigenVectorsPCA;
          //transform Point Cloud to map coordinates
          pcl::transformPointCloud<PointT>(*cluster_cloud, *cloud2, eigenTransform);
          computePCAAxis(cloud2, pcaCentroid2, eigenVectorsPCA);
          getAllTransforms(eigenVectorsPCA,transformation_true, eigenTransformWtC, pcaCentroid, pcaCentroid1, pcaCentroid2,item);
          scores.resize(transformation_true.size());
          listTransform.resize(transformation_true.size());
          listTransformr.resize(transformation_true.size());
          ROS_WARN("///////////////////////// Franklin %d *********************",transformation_true.size());
          handables.at(i)=i;
          //omp_set_num_threads(14);
          #pragma omp parallel for
          for(int n=0;n<transformation_true.size();n++){
               //random_transform(transformation_true);
               //pcl::transformPointCloud( *cluster_cloud, *cloudff, transformation_true.at(n));


              pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
              icp.setMaximumIterations (icpMaximumIterations_);
              icp.setTransformationEpsilon (icpTransformationEpsilon_);
              icp.setMaxCorrespondenceDistance (icpMaxCorrespondenceDistance_);
              icp.setEuclideanFitnessEpsilon (icpEuclideanFitnessEpsilon_);
              icp.setRANSACOutlierRejectionThreshold (icpRANSACOutlierRejectionThreshold_);
               pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudg (new pcl::PointCloud<pcl::PointXYZRGBA>);
               pcl::transformPointCloud( *cloud1, *cloud3, transformation_true.at(n));
               icp.setInputSource(cluster_cloud);
               icp.setInputTarget(cloud3);
               icp.align(*cloudg);
               scores.at(n)=min_score;
               if (icp.hasConverged()){

                   ROS_WARN("ICP HAS CONVERGED!!!");
                   //Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation ();
                   //std::cout<<"trans %n"<<transformationMatrix<<std::endl;
                   std::cout << "\n  ICP has converged, score is " << icp.getFitnessScore () <<" "<<obj_name<<n<< std::endl;
                   const int id = omp_get_thread_num();
                   ROS_WARN("Hello World from thread %d", id);

                   scores.at(n)=icp.getFitnessScore ();

                  /* if(min_score>icp.getFitnessScore()){
                       min_score=icp.getFitnessScore ();
                       index=n;
                   }*/
                   listTransformr.at(n)=(transformation_true.at(n));
                   listTransform.at(n)=(icp.getFinalTransformation());
                   //pcl::transformPointCloud( *cluster_cloud, *cloudff, transformationMatrix);
                   //*cloudf+=*cloud1;
                   // *cloudff+=*cloud1;
                   //pcl::io::savePCDFileASCII ("/home/franklin/Desktop/ws/rs_ws/data/Tigercup"+std::to_string(20+n)+".pcd", *cloudf);
                   //pcl::io::savePCDFileASCII ("/home/franklin/Desktop/ws/rs_ws/data/Tigercup7.pcd", *cloudff);

               }else
                   ROS_INFO("ICP COULD NOT CONVERGE!!!");
           }
          /***************************************************/
          for(int q=0;q<scores.size();q++)
              if(min_score>scores.at(q)){
                                 min_score=scores.at(q);
                                 index=q;
                             }
          if(index>-1){
               //Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation ();
               std::cout<<"final trans %n"<<listTransform.at(index).matrix()<<std::endl;
               std::cout<<"score"<<min_score<<" "<<index<<std::endl;
               //Eigen::Affine3f M();
               pcl::transformPointCloud(*cloud1, *cloudff, listTransform.at(index).inverse()*listTransformr.at(index).matrix());
               *cloud_ptr+=*cloudff;
               *cluster_cloud=*cloudff;
               eigenTransformobj=eigenTransformobj*(listTransform.at(index).inverse()*listTransformr.at(index).matrix()).cast<double>();
           }
           //transform Point Cloud to map coordinates
           pcl::transformPointCloud<PointT>(*cluster_cloud, *cluster_transformed, eigenTransform);
           rs::conversion::from(cluster.rois().roi.get(),box.rect_);
           computeBoundingBoxPCA(cluster_transformed, box, eigenTransformobj,item);
          //computeBoundingBoxMinArea(cluster_transformed, box);
          // computeBoundingBoxMoments(cluster_transformed, box);
      }
      ROS_ERROR("OBJECT NAME: %s, OBJECT ID: %i",obj_name.data(),i);
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
      semSize.source.set("Object6DPoseAnnotator");

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

  void  getAllTransforms(Eigen::Matrix3f eigenVectorsPCA,std::vector<Eigen::Affine3f>& transformation_true, Eigen::Affine3d eigenTransformWtC, Eigen::Vector4f pcaCentroid, Eigen::Vector4f pcaCentroid1, Eigen::Vector4f pcaCentroid2, ObjectNameMapItem item){
        Eigen::Matrix3f M;
        Eigen::Matrix3f N=Eigen::Matrix3f::Identity();

        M.matrix().col(abs(item.axisMap[0])-1)=(abs(item.axisMap[0])*1.0/item.axisMap[0])*eigenVectorsPCA.col(0);
        M.matrix().col(abs(item.axisMap[1])-1)=(abs(item.axisMap[1])*1.0/item.axisMap[1])*eigenVectorsPCA.col(1);
        M.matrix().col(abs(item.axisMap[2])-1)=(abs(item.axisMap[2])*1.0/item.axisMap[2])*eigenVectorsPCA.col(2);

        Eigen::Affine3f R (eigenTransformWtC.matrix().block<3,3>(0,0).cast<float>()*M);
        Eigen::Translation3f T ( (pcaCentroid1.x()-0*pcaCentroid.x()), (pcaCentroid1.y()-0*pcaCentroid.y()), (pcaCentroid1.z()-0*pcaCentroid.z()) );
        //Eigen::Translation3f T (0.0,0.0,0.0);
        transformation_true.push_back(T * R) ;

        Eigen::Matrix3f M1;
        M1.matrix().col(0)=M.col(1);
        M1.matrix().col(1)=M.col(0);
        M1.matrix().col(2)=-M.col(2);
        Eigen::Affine3f R1 (M1);
        transformation_true.push_back(T * R1) ;

        Eigen::Matrix3f M2;
        M2.matrix().col(0)=M.col(1);
        M2.matrix().col(1)=M.col(2);
        M2.matrix().col(2)=M.col(0);
        Eigen::Affine3f R2 (M2);
        transformation_true.push_back(T * R2) ;

        Eigen::Matrix3f M3;
        M3.matrix().col(0)=-M.col(0);
        M3.matrix().col(1)=M.col(2);
        M3.matrix().col(2)=M.col(1);
        Eigen::Affine3f R3 (M3);
        transformation_true.push_back(T * R3);

        Eigen::Matrix3f M4;
        M4.matrix().col(0)=M.col(2);
        M4.matrix().col(1)=-M.col(1);
        M4.matrix().col(2)=M.col(0);
        Eigen::Affine3f R4 (M4);
        transformation_true.push_back(T * R4) ;

        Eigen::Matrix3f M5;
        M5.matrix().col(0)=M.col(2);
        M5.matrix().col(1)=M.col(0);
        M5.matrix().col(2)=M.col(1);
        Eigen::Affine3f R5 (M5);
        transformation_true.push_back(T * R5) ;

        Eigen::Matrix3f M6;
        M6.matrix().col(0)=M.col(0);
        M6.matrix().col(1)=-M.col(2);
        M6.matrix().col(2)=M.col(1);
        Eigen::Affine3f R6 (M6);
        transformation_true.push_back(T * R6);

        Eigen::Matrix3f M7;
        M7.matrix().col(0)=M.col(0);
        M7.matrix().col(1)=M.col(2);
        M7.matrix().col(2)=-M.col(1);
        Eigen::Affine3f R7 (M7);
        transformation_true.push_back(T * R7);

        Eigen::Matrix3f M8;
        M8.matrix().col(0)=M.col(0);
        M8.matrix().col(1)=-M.col(2);
        M8.matrix().col(2)=M.col(1);
        Eigen::Affine3f R8 (M8);
        transformation_true.push_back(T * R8);

        Eigen::Matrix3f M9;
        M9.matrix().col(0)=M.col(0);
        M9.matrix().col(1)=M.col(2);
        M9.matrix().col(2)=-M.col(1);
        Eigen::Affine3f R9 (M9);
        transformation_true.push_back(T * R9);

        Eigen::Matrix3f M10;
        M10.matrix().col(0)=-M.col(2);
        M10.matrix().col(1)=M.col(1);
        M10.matrix().col(2)=M.col(0);
        Eigen::Affine3f R10 (M10);
        transformation_true.push_back(T * R10) ;

        Eigen::Matrix3f M11;
        M11.matrix().col(0)=M.col(2);
        M11.matrix().col(1)=M.col(1);
        M11.matrix().col(2)=-M.col(0);
        Eigen::Affine3f R11 (M11);
        transformation_true.push_back(T * R11) ;






  };

  void random_transform(Eigen::Affine3f& transformation_true){
      //set up generator
      boost::random::mt19937 gen; // alternative to rand()
      gen.seed( std::time(0) ); // random seed with current time in second
      boost::random::uniform_real_distribution<float> frand( 0.0, 6.28 ); // random gen between 1.0 and 3.2

      // create random transformation: R and T

          // random rotation matrix
          Eigen::Vector3f axis;
          axis.setRandom().normalize();
          float angle = frand( gen );
          Eigen::Affine3f R ( Eigen::AngleAxis<float> ( angle, axis ) );

          // random translation vector
          Eigen::Translation3f T ( 0.0, 0.0, 0.0 );

          std::cout << "true R" << std::endl << R.matrix() << std::endl
                    << "true T" << std::endl << T .vector() << std::endl;

          /*if ( use_scale )
          {
            float scale = frand( gen );
            R.matrix().topLeftCorner(3,3) *= scale;
            std::cout << "true sR" << std::endl << R.matrix() << std::endl
                      << "true scale " << scale << std::endl;
          }*/

          // R and T
          transformation_true = T * R ; // shoul be in this order if you mean (Rx + T).   If R*T, then R(x+t) !


         std::cout << "true transformation" << std::endl << transformation_true.matrix() << std::endl;

  }

  double cluster_distance( PointT p,  PointT q) const{
      return std::sqrt(std::pow(p.x-q.x,2.0)+std::pow(p.y-q.y,2.0)+std::pow(p.z-q.z,2.0));
  }


  void refinePointcloud( const pcl::PointCloud<PointT>::ConstPtr &cloud, std::vector<PointT> &scene_points) const{

      /***** HIERARCHICAL SEMANTIC CLUSTERING: REMOVING MISSEGMENTATION POINTS ************************************************************************************/

      //max distance among clusters
      //ROS_WARN("max distance among clusters");
      double max_cluster_distance= maxClusterDistance_;
      int intersection_deviation=intersectionDeviation_;
      //initialize the set of points
      //ROS_WARN("initialize the set of points");
      std::vector<int> set_points;
      for(size_t i = 0; i < cloud->points.size(); ++i)
      {
        set_points.push_back(i);
         //ROS_WARN("points:   %f %f %f",cloud->points.at(i)._PointXYZRGBA::x,cloud->points.at(i).y,cloud->points.at(i).z);
      }

      //initialize set of clusters
      //ROS_WARN("initialize the set of cluster");
      std::vector<std::vector<int>> clusters;
      //build list of clusters
      //ROS_WARN("build list of clusters");
     for(int k=0;k<set_points.size();k++){
          //set the tracker
          //ROS_WARN("set the tracker");
          //ROS_WARN("size backup  %d %d",set_points.size(),cloud->points.size());
          //initialize cluster
          //ROS_WARN("initialize cluster");
          std::vector<int> cluster;
          int origin=set_points.at(k);
          cluster.push_back(origin);
          //look for cluster points
          //ROS_WARN("look for cluster points");
          //ROS_WARN(" mittel size backup  %d %d",set_points.size(),cluster.size());
          for(int i=k+1;i<set_points.size();i++){
              // //ROS_WARN("points: %d %d %d %d %f %f %f, %f %f %f",origin, set_points.at(i),set_points.size(),cloud->points.size(),cloud->points.at(origin)._PointXYZRGBA::x,cloud->points.at(origin).y,cloud->points.at(origin).z,cloud->points.at(set_points.at(i)).x,cloud->points.at(set_points.at(i)).y,cloud->points.at(set_points.at(i)).z);
              ////ROS_WARN("point distance %f",this->cluster_distance(cloud->points.at(origin),cloud->points.at(set_points.at(i))));
              if(this->cluster_distance(cloud->points.at(origin),cloud->points.at(set_points.at(i)))<=max_cluster_distance){
                   ////ROS_WARN("track touched points start %d %d %d %d",origin,set_points.at(i),set_points.size(),cloud->points.size());
                  cluster.push_back(set_points.at(i));
                  // //ROS_WARN("track touched points end %d %d %d %d",origin,set_points.at(i),set_points.size(),cloud->points.size());
              }

          }

          //add cluster to list of clusters
          //ROS_WARN("add cluster to list of clusters");
          std::sort(cluster.begin(),cluster.end());
          clusters.push_back(cluster);
      }
      //aggregation of clusters
      //ROS_WARN("aggregation of clusters");
      std::vector<std::vector<int>> backup_clusters;
      std::vector<std::vector<int>> aggregate_clusters;
      std::vector<std::vector<int>> tracker;
      for(int i=0;i<clusters.size();i++)
          backup_clusters.push_back(clusters.at(i));
      //ROS_WARN("building aggregations");

      do{
          tracker=aggregate_clusters;
          aggregate_clusters.clear();
          //ROS_WARN(" tracker vs aggregation %d %d",tracker.size(),aggregate_clusters.size());
          while(!backup_clusters.empty()){
              std::vector<int> cluster;
              std::vector<int> aggregate_cluster;
              backup_clusters.clear();
              cluster=clusters.at(0);
              //ROS_WARN("building an aggregation");
              for(int i=0;i<clusters.size();i++){
                  //ROS_WARN("size of an aggregation: %d %d %d ",cluster.size(),clusters.size(),aggregate_cluster.size());
                  std::set_union(cluster.begin(),cluster.end(), clusters.at(i).begin(), clusters.at(i).end(),  std::back_inserter(aggregate_cluster));
                   //ROS_WARN("size1 of an aggregation: %d",cluster.size() );
                  //aggregate_cluster.resize(iter_end-aggregate_cluster.begin());
                   //ROS_WARN("size2 of an aggregation: %d",cluster.size() );
                  if(aggregate_cluster.size()<cluster.size()+clusters.at(i).size()){
                      cluster=aggregate_cluster;
                      aggregate_cluster.clear();
                      //aggregate_cluster.resize(cloud->points.size());
                       //ROS_WARN("size3 of an aggregation: %d",cluster.size() );
                  }else{
                      aggregate_cluster.clear();
                      //aggregate_cluster.resize(cloud->points.size());
                      backup_clusters.push_back(clusters.at(i));
                      //ROS_WARN("size4 of an aggregation: %d",cluster.size() );
                  }
              }
              //unexamined clusters
              //ROS_WARN("unexamined clusters");
              clusters.clear();
              for(int i=0;i<backup_clusters.size();i++)
                 clusters.push_back(backup_clusters.at(i));
              //add aggregate cluster to list of final clusters
              //ROS_WARN("add aggregate cluster to list of final clusters %d",cluster.size());
              aggregate_clusters.push_back(cluster);
          }
          backup_clusters=aggregate_clusters;
          clusters=aggregate_clusters;
      }while(aggregate_clusters.size()!=tracker.size());


      //look for the biggest cluster
      //ROS_WARN("look for the biggest cluster %d",aggregate_clusters.size());
      if(aggregate_clusters.size()>0){
        int max_size=aggregate_clusters.at(0).size();
        int max_index=0;
        for(int i=0;i<aggregate_clusters.size();i++)
            if(aggregate_clusters.at(i).size()>max_size){
                max_size=aggregate_clusters.at(i).size();
                max_index=i;
            }
        //filter out outliers
        //ROS_WARN("filter out outliers %d %d", max_size, max_index);
        for(int i=0;i<aggregate_clusters.at(max_index).size();i++){
             //ROS_WARN("filter out index %d %d", cloud->points.size(), max_index,aggregate_clusters.at(max_index).at(i));
            scene_points.push_back(cloud->points.at(aggregate_clusters.at(max_index).at(i)));}
        //ROS_WARN("filtering outliers ...");
      }else{
          //nothing to filter out
          //ROS_WARN("nothing to filter out");
          for(size_t i = 0; i < cloud->points.size(); ++i)
            scene_points.push_back(cloud->points.at(i));
      }
       //ROS_WARN("end of hierarchical clustering");
      /*************************************************************************************************************************************************************/


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


  /***********************************************************************************************************************/

  void computePCAAxis(pcl::PointCloud<PointT>::Ptr cloud,  Eigen::Vector4f& pcaCentroid, Eigen::Matrix3f& eigenVectorsPCA)
  {
    try{pcl::PCA<PointT> pca;
        pcl::PointCloud<PointT> cloudProjected;
        /*std::vector<PointT> scene_points;
        refinePointcloud(cloud, scene_points);
        cloud->points.clear();
        for(int i=0;i<scene_points.size();i++)
            cloud->points.push_back(scene_points.at(i));*/
        if(cloud->points.size()>=3){
            pca.setInputCloud(cloud);
            pca.project(*cloud, cloudProjected);
            //centroid
            pcl::compute3DCentroid(*cloud, pcaCentroid);
            //eigenvectors as main axes
            eigenVectorsPCA = pca.getEigenVectors();
            //ensure consistents axes with respect to right hand rule
            eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
        }
     }
     catch(Exception e){
          ROS_WARN("Failure on pose estimation: %s",e.asString().data());
     }
  }

  /***********************************************************************************************************************/



  void computeBoundingBoxPCA(pcl::PointCloud<PointT>::Ptr cloud, OrientedBoundingBox &box, Eigen::Affine3d eigenTransformobj, ObjectNameMapItem item)
  {
    try{pcl::PCA<PointT> pca;
        pcl::PointCloud<PointT> cloudProjected;
        std::vector<PointT> scene_points;
        /*refinePointcloud(cloud, scene_points);
        cloud->points.clear();
        for(int i=0;i<scene_points.size();i++)
            cloud->points.push_back(scene_points.at(i));*/
        if(cloud->points.size()>=3){
            pca.setInputCloud(cloud);
            pca.project(*cloud, cloudProjected);

            //centroid
            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*cloud, pcaCentroid);


            //rotation matrix
            // random rotation matrix


            //Eigen::Affine3f RX ( Eigen::AngleAxis<float> (M_PI/2.0, Eigen::Matrix3f::Identity().matrix().block<1,3>(0,0)));
            //Eigen::Affine3f RZ ( Eigen::AngleAxis<float> (M_PI/2.0, Eigen::Matrix3f::Identity().matrix().block<1,3>(2,0)));

            //eigenvectors
            //Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
            Eigen::Matrix3f eigenVectorsPCA;


            eigenVectorsPCA.col(0)=(abs(item.axisMap[0])*1.0/item.axisMap[0])*eigenTransformobj.cast<float>().matrix().block<3,3>(0,0).col(abs(item.axisMap[0])-1);
            eigenVectorsPCA.col(1)=(abs(item.axisMap[1])*1.0/item.axisMap[1])*eigenTransformobj.cast<float>().matrix().block<3,3>(0,0).col(abs(item.axisMap[1])-1);
            eigenVectorsPCA.col(2)=(abs(item.axisMap[2])*1.0/item.axisMap[2])*eigenTransformobj.cast<float>().matrix().block<3,3>(0,0).col(abs(item.axisMap[2])-1);

            eigenVectorsPCA.col(2)=eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
            // Transform the original cloud to the origin where the principal components correspond to the axes.
            Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
            projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
            projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, cloudProjected, projectionTransform);

            // Get the minimum and maximum points of the transformed cloud.
            PointT proj_min, proj_max;
            pcl::getMinMax3D(cloudProjected, proj_min, proj_max);
            const Eigen::Vector3f meanDiagonal = 0.5f*(proj_max.getVector3fMap() + proj_min.getVector3fMap());

            // Final transform
            const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
            const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

            PointT /*proj_min, proj_max,*/ min_pt, max_pt;
            /*pcl::getMinMax3D(cloudProjected, proj_min, proj_max);
            pca.reconstruct(proj_min, min_pt);
            pca.reconstruct(max_pt, max_pt);*/

            tf::Transform objectToWorld;

            tf::Vector3 trans;
           // Eigen::Vector3d translation = pca.getMean().head(3).cast<double>();
            tf::vectorEigenToTF(bboxTransform.cast<double>(), trans);
            objectToWorld.setOrigin(trans);

            //Eigen::Quaterniond quaternion(pca.getEigenVectors().cast<double>());
            tf::Quaternion quat;
            tf::quaternionEigenToTF(bboxQuaternion.cast<double>(), quat);
            objectToWorld.setRotation(quat);

            box.objectToWorld = objectToWorld;
            box.width = fabs(proj_max.x - proj_min.x);
            box.height = fabs(proj_max.y - proj_min.y);
            box.depth = fabs(proj_max.z - proj_min.z);
            box.volume = box.width * box.depth * box.height;
            ROS_WARN("4DBBox: %f,%f,%f",box.width,box.height, box.volume);
        }
      }catch(Exception e){
          ROS_WARN("Failure on pose estimation: %s",e.asString().data());
      }
  }


  void computeBoundingBoxPCA2(pcl::PointCloud<PointT>::Ptr cloud, OrientedBoundingBox &box)
  {
    try{pcl::PCA<PointT> pca;
        pcl::PointCloud<PointT> cloudProjected;
        std::vector<PointT> scene_points;
        /*refinePointcloud(cloud, scene_points);
        cloud->points.clear();
        for(int i=0;i<scene_points.size();i++)
            cloud->points.push_back(scene_points.at(i));*/
        if(cloud->points.size()>=3){
            pca.setInputCloud(cloud);
            pca.project(*cloud, cloudProjected);

            //centroid
            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*cloud, pcaCentroid);


            //rotation matrix
            // random rotation matrix


            //Eigen::Affine3f RX ( Eigen::AngleAxis<float> (M_PI/2.0, Eigen::Matrix3f::Identity().matrix().block<1,3>(0,0)));
            //Eigen::Affine3f RZ ( Eigen::AngleAxis<float> (M_PI/2.0, Eigen::Matrix3f::Identity().matrix().block<1,3>(2,0)));

            //eigenvectors
            Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
            /*Eigen::Matrix3f eigenVectorsPCA;


            eigenVectorsPCA.col(0)=(abs(item.axisMap[0])*1.0/item.axisMap[0])*eigenTransformobj.cast<float>().matrix().block<3,3>(0,0).col(abs(item.axisMap[0])-1);
            eigenVectorsPCA.col(1)=(abs(item.axisMap[1])*1.0/item.axisMap[1])*eigenTransformobj.cast<float>().matrix().block<3,3>(0,0).col(abs(item.axisMap[1])-1);
            eigenVectorsPCA.col(2)=(abs(item.axisMap[2])*1.0/item.axisMap[2])*eigenTransformobj.cast<float>().matrix().block<3,3>(0,0).col(abs(item.axisMap[2])-1);*/

            eigenVectorsPCA.col(2)=eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
            // Transform the original cloud to the origin where the principal components correspond to the axes.
            Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
            projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
            projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, cloudProjected, projectionTransform);

            // Get the minimum and maximum points of the transformed cloud.
            PointT proj_min, proj_max;
            pcl::getMinMax3D(cloudProjected, proj_min, proj_max);
            const Eigen::Vector3f meanDiagonal = 0.5f*(proj_max.getVector3fMap() + proj_min.getVector3fMap());

            // Final transform
            const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
            const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

            PointT /*proj_min, proj_max,*/ min_pt, max_pt;
            /*pcl::getMinMax3D(cloudProjected, proj_min, proj_max);
            pca.reconstruct(proj_min, min_pt);
            pca.reconstruct(max_pt, max_pt);*/

            tf::Transform objectToWorld;

            tf::Vector3 trans;
           // Eigen::Vector3d translation = pca.getMean().head(3).cast<double>();
            tf::vectorEigenToTF(bboxTransform.cast<double>(), trans);
            objectToWorld.setOrigin(trans);

            //Eigen::Quaterniond quaternion(pca.getEigenVectors().cast<double>());
            tf::Quaternion quat;
            tf::quaternionEigenToTF(bboxQuaternion.cast<double>(), quat);
            objectToWorld.setRotation(quat);

            box.objectToWorld = objectToWorld;
            box.width = fabs(proj_max.x - proj_min.x);
            box.height = fabs(proj_max.y - proj_min.y);
            box.depth = fabs(proj_max.z - proj_min.z);
            box.volume = box.width * box.depth * box.height;
            ROS_WARN("4DBBox: %f,%f,%f",box.width,box.height, box.volume);
        }
      }catch(Exception e){
          ROS_WARN("Failure on pose estimation: %s",e.asString().data());
      }
  }



  void computeBoundingBoxMinArea(pcl::PointCloud<PointT>::Ptr cloud, OrientedBoundingBox &box) const
  {
    cv::Point3f min, max;
    std::vector<cv::Point> points;
    ROS_WARN("Hello Franky");
    project2D(cloud, points, min, max);
    ROS_WARN("Bye Franky");
    //ROS_WARN("CLOUD = (%d,%d,%d)",cloud->size(), cloud->width, cloud->height);
     //ROS_WARN("MINMAX = (%f,%f,%f,%f,%f,%f)",min.x,min.y,min.z,max.x,max.y,max.z);
    cv::RotatedRect rect = cv::minAreaRect(points);
    if(rect.size.width < rect.size.height)
    {
      rect.angle += 90;
      rect.size = cv::Size2f(rect.size.height, rect.size.width);
    }
     //ROS_WARN("RECT = (%f,%f,%d)",rect.size.height, rect.size.width,points.size());
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
    //ROS_WARN("BOX = (%f,%f,%f)",box.width, box.height, box.depth);
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
      //ROS_WARN("BOX %d= (%f,%f,%f)",i,box.width, box.height, box.depth);
      visualizer.addCube(translation.cast<float>(), rotation.cast<float>(), box.width, box.height, box.depth, oss.str());
    }
  }
};
// This macro exports an entry point that is used to create the annotator.
MAKE_AE(Object6DPoseAnnotator)

