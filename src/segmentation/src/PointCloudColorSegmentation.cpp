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
#include <rs/utils/common.h>

// PCL
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/point_types_conversion.h>

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
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
	PCN::Ptr normals = PCN::Ptr(new PCN);

	std::vector<pcl::PointIndices> clusters;

	int CLUSTER_LOWER_BOUND, CLUSTER_UPPER_BOUND, HUE_THRESHOLD;
	float DISTANCE_THRESHOLD;

public:

  PointCloudColorSegmentation(): DrawingAnnotator(__func__) {
	}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("Initialize");

		ctx.extractValue("minPoints", CLUSTER_LOWER_BOUND);
		ctx.extractValue("maxPoints", CLUSTER_UPPER_BOUND);
		ctx.extractValue("hueThreshold", HUE_THRESHOLD);
		ctx.extractValue("distThreshold", DISTANCE_THRESHOLD);

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
		rs::SceneCas cas(tcas);
		rs::Scene scene = cas.getScene();

    cas.get(VIEW_CLOUD, *temp);
		pcl::PointCloudXYZRGBAtoXYZHSV(*temp, *cloud);
		//hack because the method deletes xyz values
		for (int i = 0; i < cloud->size(); i++) {
			cloud->points[i].x = temp->points[i].x;
			cloud->points[i].y = temp->points[i].y;
			cloud->points[i].z = temp->points[i].z;
		}
		cas.get(VIEW_NORMALS, *normals);

		pcl::PointCloud<pcl::Label>::Ptr output_labels(new pcl::PointCloud<pcl::Label>);

		pcl::HueClusterComparator<PointH, PointN, pcl::Label>::Ptr hcc(new pcl::HueClusterComparator<PointH, PointN, pcl::Label>());
		hcc->setInputCloud(cloud);
		hcc->setDistanceThreshold(DISTANCE_THRESHOLD, true);
		hcc->setInputNormals(normals);
		hcc->setHueThreshold(HUE_THRESHOLD);

		std::vector<pcl::PointIndices> cluster_i;
		pcl::OrganizedConnectedComponentSegmentation<PointH, pcl::Label> segmenter(hcc);
		segmenter.setInputCloud(cloud);
		segmenter.segment(*output_labels, cluster_i);
		
		outInfo("Found " << cluster_i.size() << " clusters for " << cloud->size() << " points." );
		for (size_t i = 0; i < cluster_i.size(); i++) {
			if (cluster_i.at(i).indices.size() > CLUSTER_LOWER_BOUND && cluster_i.at(i).indices.size() < CLUSTER_UPPER_BOUND) {
				clusters.push_back(cluster_i.at(i));
			}
		}

		outInfo("Found " << clusters.size() << " clusters.");

		for (size_t i = 0; i < clusters.size(); ++i) {
			const pcl::PointIndices &indices = clusters[i];

			rs::Cluster uimaCluster = rs::create<rs::Cluster>(tcas);
			rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
			rs::PointIndices uimaIndices = rs::conversion::to(tcas, indices);

			rcp.indices.set(uimaIndices);
			
			uimaCluster.points.set(rcp);
			uimaCluster.rois.set(createImageRoi(tcas, *cloud, indices));
			uimaCluster.source.set("HueClustering: " + getAverageHue(cloud, indices));
			scene.identifiables.append(uimaCluster);
		}

    return UIMA_ERR_NONE;
  }
	
	std::string getAverageHue(PCH::Ptr cloud, const pcl::PointIndices &indices) {
		float hue = 0;
		int size = indices.indices.size();

		for (size_t i = 0; i < size; i++) {
			hue += cloud->points[indices.indices[i]].h / size;
		}
		
		std::stringstream ss;
		ss << std::fixed << std::setprecision(2) << hue;
		return ss.str();
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
		for (size_t i = 0; i < clusters.size(); ++i) {
			const pcl::PointIndices &indices = clusters[i];
			for (size_t j = 0; j < indices.indices.size(); ++j) {
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
