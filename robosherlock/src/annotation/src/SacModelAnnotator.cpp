/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
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

#include <uima/api.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>

#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>


#define DEBUG_OUTPUT 0
using namespace uima;

class SacModelAnnotator : public Annotator
{

private:
    bool apply_to_clusters;
    pcl::SacModel sacModel;
    std::string name;

    TyErrorId readParameters(AnnotatorContext &ctx) {
        if(ctx.isParameterDefined("sacModel"))
        {
            ctx.extractValue("sacModel", name);
            std::transform(name.begin(), name.end(), name.begin(), ::toupper);

            if(name == "CYLINDER"){
                sacModel = pcl::SACMODEL_CYLINDER;
            }
            else if(name == "SPHERE"){
                sacModel = pcl::SACMODEL_NORMAL_SPHERE;
            }
            else{
                return UIMA_ERR_NOT_YET_IMPLEMENTED;
            }

            outInfo("Selected " << name << " (" << sacModel << ")..");
            return UIMA_ERR_NONE;
        }

        return UIMA_ERR_ANNOTATOR_COULD_NOT_LOAD;
    }

public:
    TyErrorId initialize(AnnotatorContext &ctx)
    {
        outInfo("initialize");

        return readParameters(ctx);
    }

    TyErrorId reconfigure()
    {
        outInfo("Reconfiguring");

        AnnotatorContext &ctx = getAnnotatorContext();
        return readParameters(ctx);
    }

    TyErrorId destroy()
    {
        outInfo("destroy");
        return UIMA_ERR_NONE;
    }

    TyErrorId process(CAS &tcas, ResultSpecification const &resSpec)
    {
        MEASURE_TIME;
        outInfo("process begins");

        rs::SceneCas cas(tcas);
        rs::Scene scene = cas.getScene();

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);
        cas.get(VIEW_CLOUD, *cloud_ptr);
        cas.get(VIEW_NORMALS, *normals_ptr);

        std::vector<rs::ObjectHypothesis> clusters;
        scene.identifiables.filter(clusters);
        outInfo("Processing " << clusters.size() << " point clusters");
        for(std::vector<rs::ObjectHypothesis>::iterator it = clusters.begin(); it != clusters.end(); ++it)
        {
            pcl::PointIndicesPtr clusterIndices(new pcl::PointIndices());
            rs::conversion::from(((rs::ReferenceClusterPoints)it->points.get()).indices(), *clusterIndices);


            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
            pcl::PointCloud<pcl::Normal>::Ptr clusterNormal(new pcl::PointCloud<pcl::Normal>);

            pcl::ModelCoefficients::Ptr coefficientsModel(new pcl::ModelCoefficients);

            pcl::PointIndices::Ptr modelInliers(new pcl::PointIndices);

            for(std::vector<int>::const_iterator pit = clusterIndices->indices.begin();
                pit != clusterIndices->indices.end(); pit++)
            {
                clusterCloud->points.push_back(cloud_ptr->points[*pit]);
                clusterNormal->points.push_back(normals_ptr->points[*pit]);
            }

            clusterCloud->width = clusterCloud->points.size();
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;
            clusterNormal->width = normals_ptr->points.size();
            clusterNormal->height = 1;
            clusterNormal->is_dense = true;
            outInfo("Fitting the " << name << " SAC model");

            pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(sacModel);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight(0.1);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.03);
            seg.setRadiusLimits(0, 0.1);
            seg.setInputCloud(clusterCloud);
            seg.setInputNormals(clusterNormal);

            // Obtain the cylinder inliers and coefficients
            seg.segment(*modelInliers, *coefficientsModel);

            outDebug("Size of cluster " << it - clusters.begin() << ": " << clusterCloud->width);
            outInfo("Number of " << name << " inliers: " << modelInliers->indices.size());
            if((float)modelInliers->indices.size() / clusterCloud->width > 0.5)
            {
                rs::Shape shapeAnnotation = rs::create<rs::Shape>(tcas);
                shapeAnnotation.shape.set(name);
                it->annotations.append(shapeAnnotation);
            }

#if DEBUG_OUTPUT
            pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
      extract.setInputCloud(clusterCloud);
      extract.setIndices(cylinderInliers);
      extract.setNegative(false);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZRGBA> ());
      extract.filter(*cloud_cylinder);
      pcl::PCDWriter writer;
      if(cloud_cylinder->points.empty())
      {
        std::cerr << "Can't find the cylindrical component." << std::endl;
      }
      else
      {
        std::stringstream fn;
        fn << "cylinder" << it - clusters.begin() << ".pcd";
        writer.write(fn.str(), *cloud_cylinder, false);
      }
#endif
        }
        return UIMA_ERR_NONE;
    }
};

MAKE_AE(SacModelAnnotator)
