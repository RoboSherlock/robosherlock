/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
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

// UIMA
#include <uima/api.hpp>

// RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/HueClusterComparator.h>
#include <rs/ValueClusterComparator.h>
#include <rs/utils/common.h>

// PCL
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::PointXYZRGB PointX;
typedef pcl::PointXYZHSV PointH;
typedef pcl::PointCloud<PointH> PCH;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointN> PCN;

using namespace uima;

class PointCloudColorSegmentation : public DrawingAnnotator
{

private:
  PCR::Ptr temp = PCR::Ptr(new PCR);
  PCH::Ptr cloud = PCH::Ptr(new PCH);
  PCH::Ptr cloud_b = PCH::Ptr(new PCH);
  PCN::Ptr normals = PCN::Ptr(new PCN);

  std::vector<pcl::PointIndices> clusters;
  std::vector<pcl::PointIndices> hue_i;
  std::vector<pcl::PointIndices> value_i;

  int CLUSTER_LOWER_BOUND, CLUSTER_UPPER_BOUND, HUE_THRESHOLD;
  float DISTANCE_THRESHOLD, VALUE_THRESHOLD;
  bool USE_HYPOTHESIS, DISCRETIZE;
  std::string HYPOTHESIS_DUMMY;

public:

  PointCloudColorSegmentation(): DrawingAnnotator(__func__)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("Initialize");

    ctx.extractValue("minPoints", CLUSTER_LOWER_BOUND);
    ctx.extractValue("maxPoints", CLUSTER_UPPER_BOUND);

    ctx.extractValue("hueThreshold", HUE_THRESHOLD);
    ctx.extractValue("valueThreshold", VALUE_THRESHOLD);
    ctx.extractValue("distThreshold", DISTANCE_THRESHOLD);

    ctx.extractValue("useHypothesis", USE_HYPOTHESIS);
    ctx.extractValue("discretize", DISCRETIZE);

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("Destroy");

    return UIMA_ERR_NONE;
  }

private:

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    temp->clear();
    cloud->clear();
    cloud_b->clear();
    normals->clear();

    clusters.clear();
    hue_i.clear();
    value_i.clear();

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_CLOUD, *temp);
    pcl::PointCloudXYZRGBAtoXYZHSV(*temp, *cloud);
    //hack because the method deletes xyz values
    for(int i = 0; i < cloud->size(); i++)
    {
      cloud->points[i].x = temp->points[i].x;
      cloud->points[i].y = temp->points[i].y;
      cloud->points[i].z = temp->points[i].z;
    }
    cas.get(VIEW_NORMALS, *normals);

    pcl::ExtractIndices<PointH> ex;

    if(USE_HYPOTHESIS)
    {
      std::string id;
      rs::Query q = rs::create<rs::Query>(tcas);
      //get id
      cas.getFS("QUERY", q);
      //q.asJson.set("{\"ID\":0,\"POS\":1}");

      std::string j = q.asJson.get();
      int posID = j.find("\"ID\":");
      int posBrace = j.find("}", posID);
      int posComma = j.find(",", posID);
      posID += 5;

      if(posComma < 0)
      {
        j = j.substr(posID, posBrace - posID);
      }
      else
      {
        j = j.substr(posID, posComma - posID);
      }
      int hyp = stoi(j);

      //get cluster
      std::vector<rs::Cluster> clusts;
      scene.identifiables.filter(clusts);
      rs::Cluster cluster = rs::create<rs::Cluster>(tcas);
      cluster = clusts[hyp];

      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      rs::conversion::from(((rs::ReferenceClusterPoints)cluster.points.get()).indices.get(), *indices);

      //filter cloud
      ex.setKeepOrganized(true);
      ex.setIndices(indices);
      ex.setInputCloud(cloud);
      ex.filter(*cloud);
    }

    pcl::PointCloud<pcl::Label>::Ptr output_labels(new pcl::PointCloud<pcl::Label>);

    pcl::HueClusterComparator<PointH, PointN, pcl::Label>::Ptr hcc(new pcl::HueClusterComparator<PointH, PointN, pcl::Label>());
    hcc->setInputCloud(cloud);
    hcc->setDistanceThreshold(DISTANCE_THRESHOLD, true);
    hcc->setInputNormals(normals);
    hcc->setHueThreshold(HUE_THRESHOLD);
    hcc->setDiscretization(DISCRETIZE);

    hue_i.clear();
    pcl::OrganizedConnectedComponentSegmentation<PointH, pcl::Label> segmenter(hcc);
    segmenter.setInputCloud(cloud);
    segmenter.segment(*output_labels, hue_i);

    outInfo("Found " << hue_i.size() << " clusters for " << cloud->size() << " points.");
    clusters.clear();
    for(size_t i = 0; i < hue_i.size(); i++)
    {
      if(hue_i.at(i).indices.size() > CLUSTER_LOWER_BOUND && hue_i.at(i).indices.size() < CLUSTER_UPPER_BOUND)
      {
        clusters.push_back(hue_i.at(i));
      }
    }
    int hue_counter = clusters.size();
    outInfo("Found " << hue_counter << " hue clusters.");

    pcl::copyPointCloud(*cloud, *cloud_b);

    ex.setNegative(true);
    ex.setKeepOrganized(true);
    pcl::PointIndices::Ptr clust(new pcl::PointIndices());

    for(size_t i = 0; i < clusters.size(); i++)
    {
      clust->indices = clusters[i].indices;
      ex.setInputCloud(cloud_b);
      ex.setIndices(clust);
      ex.filterDirectly(cloud_b);
    }

    pcl::ValueClusterComparator<PointH, pcl::Normal>::Ptr vcc(new pcl::ValueClusterComparator<PointH, pcl::Normal>());
    vcc->setInputCloud(cloud_b);
    vcc->setDistanceThreshold(DISTANCE_THRESHOLD, true);
    vcc->setInputNormals(normals);
    vcc->setValueThreshold(VALUE_THRESHOLD);
    vcc->setDiscretization(DISCRETIZE);

    value_i.clear();
    pcl::PointCloud<pcl::Label>::Ptr output_l(new pcl::PointCloud<pcl::Label>);
    pcl::OrganizedConnectedComponentSegmentation<PointH, pcl::Label> segV(vcc);

    segV.setInputCloud(cloud_b);
    segV.segment(*output_l, value_i);

    for(size_t i = 0; i < value_i.size(); i++)
    {
      if(value_i.at(i).indices.size() > CLUSTER_LOWER_BOUND && value_i.at(i).indices.size() < CLUSTER_UPPER_BOUND)
      {
        clusters.push_back(value_i.at(i));
      }
    }

    outInfo("Found " << clusters.size() - hue_counter << " value clusters.");

    for(size_t i = 0; i < clusters.size(); ++i)
    {
      const pcl::PointIndices &indices = clusters[i];

      rs::Cluster uimaCluster = rs::create<rs::Cluster>(tcas);
      rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
      rs::PointIndices uimaIndices = rs::conversion::to(tcas, indices);

      rcp.indices.set(uimaIndices);

      uimaCluster.points.set(rcp);
      uimaCluster.rois.set(createImageRoi(tcas, *cloud, indices));
      if(i < hue_counter)
      {
        uimaCluster.source.set("HueClustering");
      }
      else
      {
        uimaCluster.source.set("ValueClustering");
      }

      rs::SemanticColor col = rs::create<rs::SemanticColor>(tcas);
      col.avHue.set(getAverageHue(cloud, indices));
      col.avValue.set(getAverageValue(cloud, indices));
      uimaCluster.annotations.append(col);

      scene.identifiables.append(uimaCluster);
    }

    return UIMA_ERR_NONE;
  }

  int getAverageHue(PCH::Ptr cloud, const pcl::PointIndices &indices)
  {
    float hue = 0;
    int size = indices.indices.size();

    for(size_t i = 0; i < size; i++)
    {
      hue += cloud->points[indices.indices[i]].h / size;
    }

    return (int) hue;
  }


  float getAverageValue(PCH::Ptr cloud, const pcl::PointIndices &indices)
  {
    float value = 0;
    int size = indices.indices.size();

    for(size_t i = 0; i < size; i++)
    {
      value += cloud->points[indices.indices[i]].v / size;
    }

    return value;
  }
  /**
   * given orignal_image and reference cluster points, compute an image containing only the cluster
   */
  rs::ImageROI createImageRoi(CAS &tcas, const pcl::PointCloud<PointH> &cloud, const pcl::PointIndices &indices)
  {
    size_t width = cloud.width,
           height = cloud.height;

    int min_x = width,
        max_x = -1,
        min_y = height,
        max_y = -1;

    cv::Mat mask_full = cv::Mat::zeros(height, width, CV_8U);

    // get min / max extents (rectangular bounding box in image (pixel) coordinates)
    #pragma omp parallel for
    for(size_t i = 0; i < indices.indices.size(); ++i)
    {
      const int idx = indices.indices[i],
                x = idx % width,
                y = idx / width;

      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);

      mask_full.at<uint8_t>(y, x) = 255;
    }

    cv::Rect roi(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    cv::Mat mask;
    mask = mask_full(roi);

    rs::ImageROI imageROI = rs::create<rs::ImageROI>(tcas);
    imageROI.mask(rs::conversion::to(tcas, mask));
    imageROI.roi(rs::conversion::to(tcas, roi));
    return imageROI;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = cv::Mat::zeros(240, 320, CV_8UC3);
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;
    for(size_t i = 0; i < clusters.size(); ++i)
    {
      const pcl::PointIndices &indices = clusters[i];
      for(size_t j = 0; j < indices.indices.size(); ++j)
      {
        size_t index = indices.indices[j];
        temp->points[index].rgba = rs::common::colors[i % rs::common::numberOfColors];
      }
    }

    double pointSize = 1;

    if(firstRun)
    {
      visualizer.addPointCloud(temp, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(temp, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
};

MAKE_AE(PointCloudColorSegmentation)
