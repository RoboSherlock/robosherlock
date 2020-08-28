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


#include <vector>

//uima
#include <uima/api.hpp>
#include <uima/fsfilterbuilder.hpp>

//
#include <opencv2/opencv.hpp>

//pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/impl/convex_hull.hpp>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

//project
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>
#include <rs/annotation/SceneGraphAnnotator.h>
#include <rs/annotation/ObjectNameMapItem.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <mutex> // std::mutex

//#define DEBUG_OUTPUT 1;
using namespace uima;

/**
 * @brief The SceneGraphAnnotator class
 * universal annotator,
 */
class SceneGraphAnnotator : public DrawingAnnotator
{



private:
  typedef pcl::PointXYZRGBA PointT;

  struct Cluster
  {
    size_t indices_index_;
    cv::Rect roi_, roi_hires_;
    cv::Mat mask, mask_hires_;
  };

  Type cloud_type;
  cv::Mat color;
  pcl::PointCloud<PointT>::Ptr cloud_ptr;
  ros::Subscriber sub;
  std::vector<Cluster> clusters;
  double pointSize;
  ros::ServiceClient client;
  ros::NodeHandle n;
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<rs::MergedHypothesis> uimaClusters;
  std::map<std::string,cv::Rect> rois;
  sensor_msgs::Image unreal_image_msg;
  bool real_virtual_image;
  std::mutex mtx;
  bool from_real_scene;
  long start_fake_time;
  long end_fake_time;
  std::string serverName_;
  YAML::Node config;
  std::string sceneObjectNameMap_;
  std::vector<ObjectNameMapItem> object_name_map_items;
  std::vector<std::string> target_object_names;

public:

  SceneGraphAnnotator(): DrawingAnnotator(__func__)
  {
    cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    //client = n.serviceClient<rs_robotvqa_msgs::GetSceneGraph>("/get_scene_graph");
    //sub = n.subscribe("/unreal_vison/image_color", 1000, &SceneGraphAnnotator::unrealImage,this);
    real_virtual_image=true; //real Image
    from_real_scene=false;
    start_fake_time=1473860925000000000;
    end_fake_time=1473860937000000000;
    serverName_="/get_scene_graph";
    sceneObjectNameMap_="scene_object_name_map.yaml";
  }

  void  unrealImage(sensor_msgs::Image image_msg)
  {
    mtx.lock();
    outInfo("Virtual World Listener");
    unreal_image_msg=image_msg;
    mtx.unlock();
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");


    if(ctx.isParameterDefined("sceneObjectNameMap"))
    {
      ctx.extractValue("sceneObjectNameMap", sceneObjectNameMap_);
    }
    if(ctx.isParameterDefined("fromScene"))
    {
       ctx.extractValue("fromScene", from_real_scene);
    }
    if(ctx.isParameterDefined("serverName"))
    {
       ctx.extractValue("serverName", serverName_);
    }
     std::string filename=ros::package::getPath("robosherlock")+"/config/"+sceneObjectNameMap_;
    readObjectNameMapItem(filename, target_object_names, object_name_map_items);

    client = n.serviceClient<rs_robotvqa_msgs::GetSceneGraph>(serverName_);
    setAnnotatorContext(ctx);
    return UIMA_ERR_NONE;
  }

  TyErrorId reconfigure()
  {
      outInfo("Reconfiguring");
      AnnotatorContext &ctx = getAnnotatorContext();
      initialize(ctx);
      return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

private:

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {

    //reading cas
    MEASURE_TIME;
    outInfo("process begins");
    rs::StopWatch clock;
    double t = clock.getTime();

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_COLOR_IMAGE_HD, color);
    cluster_indices.clear();
    clusters.clear();
    uimaClusters.clear();
    rois.clear();
    ROS_WARN("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ TIMESTAMP: %ld $$$$$$$$$$$$$$",scene.timestamp.get());
    //set the current image as server request
    rs_robotvqa_msgs::GetSceneGraph interface;
    sensor_msgs::Image image_msg;
    cv_bridge::CvImage cv_image;

    if(scene.timestamp.get()>start_fake_time && scene.timestamp.get()<end_fake_time && from_real_scene){
        std::string extension="/images/mug.png";
        std::string filename=ros::package::getPath("rs_ue4beliefstate");
        filename+=extension;
        ROS_WARN("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ TIMESTAMP: %s $$$$$$$$$$$$$$",(filename).data());
        cv::Mat fake_image(cv::imread(filename,cv::IMREAD_COLOR));
        //cv::Mat fake_image;
        //cv::resize(fake_im,fake_image, cv::Size(0, 0), 4.0, 4.0, cv::INTER_LINEAR);

        int counter=0;
        ROS_WARN("$??????????????? ImG Pixel: %d,%d ????????",fake_image.rows, fake_image.cols);
        for(int col=0;col<fake_image.cols;col++){
            for(int row=0;row<fake_image.rows;row++){
                   color.at<cv::Vec3b>(row+740, col+600)=fake_image.at<cv::Vec3b>(row, col);

             }
        }




        cas.set(VIEW_COLOR_IMAGE_HD,color);

    }
    cv_image.image = color;
    cv_image.encoding = "bgr8";
    cv_image.toImageMsg(image_msg);
    if(real_virtual_image){
         interface.request.query=image_msg;
         real_virtual_image=true;
    }
    else{
        mtx.lock();
        interface.request.query=unreal_image_msg;
        real_virtual_image=true;
        mtx.unlock();
    }
    if(!client.call(interface)){
        ROS_ERROR("Scene Graph Server failed!!!");
        return UIMA_ERR_NONE;
    }
    ROS_WARN("+++++++++++++++++++++++++++++++++Extracting clusters: %d founds ++++++++++++++++++++++++++",interface.response.answer.objects.size());

    t = clock.getTime();

    clusters.resize(interface.response.answer.objects.size());
    cluster_indices.resize(interface.response.answer.objects.size());

    //#pragma omp parallel for schedule(dynamic)

    std::vector<std::vector<std::pair<std::string,float>>> properties;
    properties.resize(interface.response.answer.objects.size());
    for(size_t i = 0; i < interface.response.answer.objects.size(); ++i)
    {

        Cluster &cluster = clusters[i];
        std::vector<std::pair<std::string,float>> &property=properties.at(i);
        cluster.indices_index_ = i;
        ROS_WARN("+++++++++++++++++++++++++++++++++++before roi ++++++++++++++++++++++++++");
        property.resize(5);
        createImageRoi(cluster,interface,property);
        ROS_WARN("+++++++++++++++++++++++++++++++++++after roi ++++++++++++++++++++++++++");
    }
    ROS_WARN("+++++++++++++++++++++++++++++++++++Extracting clusters terminated ++++++++++++++++++++++++++");
    outDebug("conversion to image ROI took: " << clock.getTime() - t << " ms.");
    t = clock.getTime();

    for(size_t i = 0; i < cluster_indices.size(); ++i)
    {
        Cluster &cluster = clusters[i];
        const pcl::PointIndices &indices = cluster_indices[i];
        std::vector<std::pair<std::string,float>> &property=properties.at(i);

        rs::MergedHypothesis uimaCluster = rs::create<rs::MergedHypothesis>(tcas);
        rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
        rs::PointIndices uimaIndices = rs::conversion::to(tcas, indices);

        //outDebug("cluster size: " << indices.indices.size());
        rcp.indices.set(uimaIndices);
        cv::Point3f min;

        rs::ImageROI imageRoi = rs::create<rs::ImageROI>(tcas);
        imageRoi.mask(rs::conversion::to(tcas, cluster.mask));
        imageRoi.mask_hires(rs::conversion::to(tcas, cluster.mask_hires_));
        imageRoi.roi(rs::conversion::to(tcas, cluster.roi_));
        imageRoi.roi_hires(rs::conversion::to(tcas, cluster.roi_hires_));
        ROS_WARN("******************* Mask SIze: (%d,%d,%d) *********", imageRoi.mask.get().rows.get(),imageRoi.mask.get().cols.get(),rcp.indices.get().indices.get().size());
        uimaCluster.points.set(rcp);
        uimaCluster.rois.set(imageRoi);
        uimaCluster.source.set("RobotVQAClustering");

        //adding annotation to cluster
        if(!from_real_scene){

          ObjectNameMapItem item;
          bool found=false;
          for(int k=0;k<object_name_map_items.size();k++){
              item=object_name_map_items[k];
              if(property.at(0).first.find(target_object_names[k])!=std::string::npos){
                   found=true;
                   break;
              }
          }
          if(!found)
              continue;

          //Category
          rs::Classification classResult = rs::create<rs::Classification>(tcas);
          classResult.classname.set(item.category);
          classResult.classifier("RobotVQA");
          classResult.featurename("type");
          classResult.source.set("RobotVQA");
          rs::ClassConfidence confidence = rs::create<rs::ClassConfidence>(tcas);
          confidence.score.set(property.at(0).second);
          confidence.name.set(property.at(0).first);
          confidence.source.set("RobotVQA");
          classResult.confidences.set({confidence});
          uimaCluster.annotations.append(classResult);

          rs::Classification classResult1 = rs::create<rs::Classification>(tcas);
          classResult1.classname.set(property.at(0).first);
          classResult1.classifier("RobotVQA");
          classResult1.featurename("Category");
          classResult1.source.set("RobotVQA");
          rs::ClassConfidence confidence1 = rs::create<rs::ClassConfidence>(tcas);
          confidence1.score.set(property.at(0).second);
          confidence1.name.set(property.at(0).first);
          confidence1.source.set("RobotVQA");
          classResult1.confidences.set({confidence1});
          //uimaCluster.annotations.append(classResult1);

          //Color
          rs::SemanticColor colorAnnotation = rs::create<rs::SemanticColor>(tcas);
          colorAnnotation.color.set(item.color.at(0));
          colorAnnotation.ratio.set(property.at(1).second);
          colorAnnotation.source.set("RS_RobotVQA");
          //uimaCluster.annotations.append(colorAnnotation);

          rs::SemanticColor colorAnnotation1 = rs::create<rs::SemanticColor>(tcas);
          colorAnnotation1.color.set(property.at(1).first);
          colorAnnotation1.ratio.set(property.at(1).second);
          colorAnnotation1.source.set("RobotVQA");
          uimaCluster.annotations.append(colorAnnotation1);

          //Shape
          rs::Shape shapeAnnotation = rs::create<rs::Shape>(tcas);
          shapeAnnotation.shape.set(item.shape.at(0));
          shapeAnnotation.confidence.set(property.at(2).second);
          shapeAnnotation.source.set("RS_RobotVQA");
          //uimaCluster.annotations.append(shapeAnnotation);

          //Shape
          rs::Shape shapeAnnotation1 = rs::create<rs::Shape>(tcas);
          shapeAnnotation1.shape.set(property.at(2).first);
          shapeAnnotation1.confidence.set(property.at(2).second);
          shapeAnnotation1.source.set("RobotVQA");
          uimaCluster.annotations.append(shapeAnnotation1);

          //Material
          rs::SemanticMaterial materialAnnotation = rs::create<rs::SemanticMaterial>(tcas);
          materialAnnotation.material.set(property.at(3).first);
          materialAnnotation.confidence.set(property.at(3).second);
          materialAnnotation.source.set("RobotVQA");
          uimaCluster.annotations.append(materialAnnotation);


          //Openability
          rs::SemanticOpenability openabilityAnnotation = rs::create<rs::SemanticOpenability>(tcas);
          openabilityAnnotation.openability.set(property.at(4).first);
          openabilityAnnotation.confidence.set(property.at(4).second);
          openabilityAnnotation.source.set("RobotVQA");
          uimaCluster.annotations.append(openabilityAnnotation);

          //Spatial Relation
          for(size_t j = 0; j < interface.response.answer.relations.size(); ++j)
          {
               if(interface.response.answer.relations.at(j).object_id1==classResult1.classname.get()){
                   rs::SemanticSpatialRelation srAnnotation = rs::create<rs::SemanticSpatialRelation>(tcas);
                   srAnnotation.target.set(interface.response.answer.relations.at(j).object_id2);
                   srAnnotation.relation.set(interface.response.answer.relations.at(j).text_value.at(0));
                   srAnnotation.confidence.set(interface.response.answer.relations.at(j).confidence.at(0));
                   srAnnotation.source.set("RobotVQA");
                   uimaCluster.annotations.append(srAnnotation);
               }

          }
          cv::Rect rect(clusters[i].roi_hires_.x,clusters[i].roi_hires_.y,clusters[i].roi_hires_.width,clusters[i].roi_hires_.height);
          if(!from_real_scene)
            rois[classResult1.classname.get()]=rect;

        }
          cv::Rect rect(clusters[i].roi_hires_.x,clusters[i].roi_hires_.y,clusters[i].roi_hires_.width,clusters[i].roi_hires_.height);
          uimaClusters.push_back(uimaCluster);
          scene.identifiables.append(uimaCluster);
    }
    ROS_WARN("+++++++++++++++++++++++++++++++++++Merging clusters terminated ++++++++++++++++++++++++++");
    outDebug("adding clusters took: " << clock.getTime() - t << " ms.");
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
         disp = color.clone();
         for(size_t i = 0; i < clusters.size(); ++i)
         {

           cv::Rect rect(clusters[i].roi_hires_.x,clusters[i].roi_hires_.y,clusters[i].roi_hires_.width,clusters[i].roi_hires_.height);
           if(!from_real_scene){
           std::vector<rs::Classification>       classes;
           std::vector<rs::SemanticColor>        colors;
           std::vector<rs::Shape>                shapes;
           std::vector<rs::SemanticMaterial>     materials;
           std::vector<rs::SemanticOpenability>  openabilities;
           std::vector<rs::SemanticSpatialRelation> sRelations;

           uimaClusters.at(i).annotations.filter(classes);
           uimaClusters.at(i).annotations.filter(colors);
           uimaClusters.at(i).annotations.filter(shapes);
           uimaClusters.at(i).annotations.filter(materials);
           uimaClusters.at(i).annotations.filter(openabilities);
           uimaClusters.at(i).annotations.filter(sRelations);

           ROS_ERROR("Classes %d",classes.size());



           std::string label=classes.at(0).confidences.get().at(0).name.get()+" "+std::to_string(classes.at(0).confidences.get().at(0).score.get())+"\n"+
                             colors.at(0).color.get()+" "+std::to_string(colors.at(0).ratio.get())+"\n"+
                             shapes.at(0).shape.get() +" "+std::to_string(shapes.at(0).confidence.get())+"\n"+
                             materials.at(0).material.get()+" "+std::to_string(materials.at(0).confidence.get())+"\n"+
                             openabilities.at(0).openability.get()+" "+std::to_string(openabilities.at(0).confidence.get())+"\n";
           std::string ss=classes.at(0).confidences.get().at(0).name.get();
           ROS_WARN("Classes %d = %d",i,std::stoi(ss.substr(0,1)));
           for(size_t j = 0; j < sRelations.size(); ++j){
                cv::Rect rect1=rois[sRelations.at(j).target.get()];
                cv::Rect rect2((clusters[i].roi_hires_.x+clusters[i].roi_hires_.width/2),(clusters[i].roi_hires_.y+clusters[i].roi_hires_.height/2),(rect1.x+rect1.width/2),(rect1.y+rect1.height/2));
                cv::Scalar color;
                if(sRelations.at(j).relation.get()=="On")
                    color=CV_RGB(0, 0, 255);
                else
                    if(sRelations.at(j).relation.get()=="In")
                        color=CV_RGB(255, 0, 150);
                    else
                        if(sRelations.at(j).relation.get()=="Front")
                            color=CV_RGB(255, 0, 0);
                        else
                            color=CV_RGB(0, 255, 0);
                cv::arrowedLine(disp,cv::Point(rect2.x,rect2.y),cv::Point(rect2.width,rect2.height),color);
           }
           drawCluster(disp, rect, label, rs::common::cvScalarColors[i % rs::common::numberOfColors]);
          }else
           cv::rectangle(disp, rect, rs::common::cvScalarColors[i % rs::common::numberOfColors]);
         }
         if(!real_virtual_image){
             std::string l="REFINEMENT AFTER PHYSICS-BASED REASONING";
             int baseLine;
             cv::Size textSize = cv::getTextSize(l, cv::FONT_HERSHEY_PLAIN, 1.5, 2.0, &baseLine);
             cv::putText(disp, l, cv::Point(20,20), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0), 2);
         }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
         const std::string &cloudname = this->name;
         for(size_t i = 0; i < cluster_indices.size(); ++i)
         {
           const pcl::PointIndices &indices = cluster_indices[i];
           for(size_t j = 0; j < indices.indices.size(); ++j)
           {
             size_t index = indices.indices[j];
             cloud_ptr->points[index].rgba = rs::common::colors[i % rs::common::numberOfColors];
             cloud_ptr->points[index].a = 255;
           }
         }
         if(firstRun)
         {
           visualizer.addPointCloud(cloud_ptr, cloudname);
           visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
         }
         else
         {
           visualizer.updatePointCloud(cloud_ptr, cloudname);
           visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
         }
  }

  void drawCluster(cv::Mat &input, cv::Rect rect, const std::string &label, cv::Scalar color)
  {

    cv::rectangle(input, rect, color, 2);
    int offset = 25;
    int baseLine;
    std::string actual_label=label;
    int pos=actual_label.find('\n');
    int k=5;
    while(pos>0){
        std::string l=actual_label.substr(0,pos);
        cv::Size textSize = cv::getTextSize(l, cv::FONT_HERSHEY_PLAIN, 1.5, 2.0, &baseLine);
        cv::putText(input, l, cv::Point(rect.x, rect.y - offset - textSize.height*k), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
        k--;
        actual_label= actual_label.substr(pos+1, actual_label.size()-pos-1);
        pos=actual_label.find('\n');
    }
  }

  /**
   * given orignal_image and reference cluster points, compute an image containing only the cluster
   */
  void createImageRoi(Cluster &cluster, rs_robotvqa_msgs::GetSceneGraph &interface,std::vector<std::pair<std::string,float>> &property)
  {

         cluster_indices[cluster.indices_index_].indices.empty();
         std::vector<float> mask;
         std::vector<float> bbox;
         std::vector<int> indice;

         ROS_WARN("+++++++++++++++++++++++++++++++++++in roi ++++++++++++++++++++++++++");
         for(int k=0;k<interface.response.answer.objects.at(cluster.indices_index_).properties.size();k++){
             ROS_WARN("+++++++++++++++++++++++++++++++++++in before roi ++++++++++++++++++++++++++");
              if(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).name=="Mask"){
                  mask=interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).num_value;

              }else
                   if(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).name=="BBox"){
                       bbox=interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).num_value;

                   }else{
                       if(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).name=="Category"){
                           property.at(0)=std::pair<std::string,float>(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).text_value.at(0),
                                                                       interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).confidence.at(0));
                       }else{
                           if(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).name=="Color"){
                               property.at(1)=std::pair<std::string,float>(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).text_value.at(0),
                                                                           interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).confidence.at(0));
                           }else{
                               if(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).name=="Shape"){
                                   property.at(2)=std::pair<std::string,float>(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).text_value.at(0),
                                                                               interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).confidence.at(0));
                               }else{
                                   if(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).name=="Material"){
                                       property.at(3)=std::pair<std::string,float>(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).text_value.at(0),
                                                                                   interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).confidence.at(0));
                                   }else{
                                         if(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).name=="Openability"){
                                           property.at(4)=std::pair<std::string,float>(interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).text_value.at(0),
                                                                                       interface.response.answer.objects.at(cluster.indices_index_).properties.at(k).confidence.at(0));
                                         }
                                   }
                               }
                           }
                       }
                   }
              ROS_WARN("+++++++++++++++++++++++++++++++++++in after roi ++++++++++++++++++++++++++");
         }
          size_t width = cloud_ptr->width;
          size_t height = cloud_ptr->height;
          ROS_WARN("Point Cloud Size: (%d,%d)",height,width);
          cv::Mat mask_full = cv::Mat::zeros(height, width, CV_8U);

          // get min / max extents (rectangular bounding box in image (pixel) coordinates)
          //#pragma omp parallel for
          ROS_WARN("+++++++++++++++++++++++++++++++++++after loop roi ++++++++++++++++++++++++++");
            for(int x=0; x<width;x++){
                for(int y=0;y<height;y++){
                    if(mask.at(width*y+x)>0.5){
                       mask_full.at<uint8_t>(y, x) =255;
                       cluster_indices[cluster.indices_index_].indices.push_back(width*y+x);
                    }
                }
            }

          ROS_WARN("+++++++++++++++++++++++++++++++++++in last block roi: size=%d ++++++++++++++++++++++++++",mask.size());
          cluster.roi_ = cv::Rect(bbox.at(0),bbox.at(1), bbox.at(2), bbox.at(3));
          cluster.roi_hires_ = cv::Rect(cluster.roi_.x << 1, cluster.roi_.y << 1, cluster.roi_.width << 1, cluster.roi_.height << 1);
          mask_full(cluster.roi_).copyTo(cluster.mask);
          cv::resize(cluster.mask, cluster.mask_hires_, cv::Size(0, 0), 2.0, 2.0, cv::INTER_NEAREST);




  }
};

MAKE_AE(SceneGraphAnnotator)
