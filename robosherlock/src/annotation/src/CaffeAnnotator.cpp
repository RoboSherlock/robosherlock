/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
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
#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

//Caffe
#include <caffe/caffe.hpp>

//RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

#include <ros/package.h>

#include <rs/recognition/CaffeProxy.h>


using namespace uima;

class CaffeAnnotator : public DrawingAnnotator
{
private:
  std::shared_ptr<CaffeProxy> caffeProxyObj;
  bool caffe_normalize;

  cv::Mat color;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
public:

  CaffeAnnotator(): DrawingAnnotator(__func__)
  {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    std::string resourcesPath, caffe_model_file, caffe_trained_file, caffe_mean_file, caffe_label_file;

    resourcesPath = ros::package::getPath("rs_resources") + '/';

    if(ctx.isParameterDefined("caffe_annotator_model_file"))
    {
      ctx.extractValue("caffe_annotator_model_file", caffe_model_file);
    }
    if(ctx.isParameterDefined("caffe_annotator_trained_file"))
    {
      ctx.extractValue("caffe_annotator_trained_file", caffe_trained_file);
    }
    if(ctx.isParameterDefined("caffe_annotator_mean_file"))
    {
      ctx.extractValue("caffe_annotator_mean_file", caffe_mean_file);
    }
    if(ctx.isParameterDefined("caffe_annotator_label_file"))
    {
      ctx.extractValue("caffe_annotator_label_file", caffe_label_file);
    }
    if(ctx.isParameterDefined("caffe_annotator_normalize"))
    {
      ctx.extractValue("caffe_annotator_normalize", caffe_normalize);
    }

    outInfo("  model: " FG_YELLOW << caffe_model_file);
    outInfo("trained: " FG_YELLOW << caffe_trained_file);
    outInfo("   mean: " FG_YELLOW << caffe_mean_file);
    outInfo(" labels: " FG_YELLOW << caffe_label_file);

    // Check if the data has already been saved to disk
    if(!boost::filesystem::exists(resourcesPath + caffe_model_file) ||
       !boost::filesystem::exists(resourcesPath + caffe_trained_file) ||
       !boost::filesystem::exists(resourcesPath + caffe_mean_file) ||
       !boost::filesystem::exists(resourcesPath + caffe_label_file))
    {
      outError("files not found!");
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }
    caffeProxyObj = std::make_shared<CaffeProxy>(resourcesPath + caffe_model_file,
                    resourcesPath + caffe_trained_file,
                    resourcesPath + caffe_mean_file,
                    resourcesPath + caffe_label_file);

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
    outInfo("process start");
    rs::SceneCas cas(tcas);

    cas.get(VIEW_CLOUD, *cloud);
    cas.get(VIEW_COLOR_IMAGE_HD, color);

    rs::Scene scene = cas.getScene();

    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::ObjectHypothesis &cluster = clusters[i];
      if(!cluster.points.has())
      {
        continue;
      }
      cv::Rect roi;
      rs::conversion::from(cluster.rois().roi_hires(), roi);

      const cv::Mat &clusterImg = color(roi);

      std::vector<float> result = caffeProxyObj->extractFeature(clusterImg);
      cv::Mat desc(1, result.size(), CV_32F, &result[0]);

      if(caffe_normalize)
      {
        //desc /= std::sqrt(desc.dot(desc));
        cv::normalize(desc, desc, 1, 0, cv::NORM_L2);
      }

      rs::Features feat = rs::create<rs::Features>(tcas);
      feat.descriptors(rs::conversion::to(tcas, desc));
      feat.descriptorType("numerical");
      feat.source("Caffe");

      cluster.annotations.append(feat);
    }

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = color.clone();
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, bool firstRun)
  {
    const std::string name = "cloud";
    if(firstRun)
    {
      visualizer.addPointCloud(cloud, name);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, name);
    }
    else
    {
      visualizer.updatePointCloud(cloud, name);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, name);
    }
  }
};

MAKE_AE(CaffeAnnotator)
