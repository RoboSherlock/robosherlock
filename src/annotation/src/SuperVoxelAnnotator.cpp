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

// PCL
#include <pcl/segmentation/supervoxel_clustering.h>
#include <vtkPolyLine.h>

// RS
#include <rs/DrawingAnnotator.h>
#include <rs/scene_cas.h>
#include <rs/utils/output.h>
#include <rs/utils/time.h>

using namespace uima;

class Supervoxels : public DrawingAnnotator
{
private:
  pcl::SupervoxelClustering<pcl::PointXYZRGBA> *supervoxels;
  std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;
  bool use_transform;
  float voxel_resolution, seed_resolution, color_importance, spatial_importance, normal_importance;
  double normalsColor[3];
  double pointSize;
  cv::Mat out;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr supervoxelcloud;

  enum
  {
    ALL,
    W_VOXELS,
    W_VA,
    W_VN,
    W_AN,
    W_ADJACENCY,
    W_NORMALS,
    ADJACENCY,
    NONE,
    TEST
  } dispMode;

public:

  Supervoxels() : DrawingAnnotator(__func__), use_transform(false), voxel_resolution(0.008f), seed_resolution(0.1f), color_importance(0.2f),
    spatial_importance(0.4f), normal_importance(1.0f), normalsColor {1.0, 0.0, 0.0}, pointSize(3.0), dispMode(ALL) {}

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("Initialize");

    if(ctx.isParameterDefined("use_transform"))
    {
      ctx.extractValue("use_transform", use_transform);
    }
    if(ctx.isParameterDefined("voxel_resolution"))
    {
      ctx.extractValue("voxel_resolution", voxel_resolution);
    }
    if(ctx.isParameterDefined("seed_resolution"))
    {
      ctx.extractValue("seed_resolution", seed_resolution);
    }
    if(ctx.isParameterDefined("color_importance"))
    {
      ctx.extractValue("color_importance", color_importance);
    }
    if(ctx.isParameterDefined("spatial_importance"))
    {
      ctx.extractValue("spatial_importance", spatial_importance);
    }
    if(ctx.isParameterDefined("normal_importance"))
    {
      ctx.extractValue("normal_importance", normal_importance);
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("Destroy");
    delete supervoxels;
    return UIMA_ERR_NONE;
  }

private:
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("Process begins");
    rs::SceneCas cas(tcas);
    cas.get(VIEW_THERMAL_COLOR_IMAGE, out);

    supervoxelcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PointCloud< pcl::PointXYZRGBA>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_THERMAL_CLOUD, *cloudPtr);
    supervoxels = new pcl::SupervoxelClustering<pcl::PointXYZRGBA>(voxel_resolution, seed_resolution, use_transform);
    supervoxels->setColorImportance(color_importance);
    supervoxels->setSpatialImportance(spatial_importance);
    supervoxels->setNormalImportance(normal_importance);
    supervoxels->setInputCloud(cloudPtr);

    if(!supervoxel_clusters.empty())
    {
      supervoxel_clusters.clear();
    }
    supervoxels->extract(supervoxel_clusters);

    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr >::iterator it = supervoxel_clusters.begin();
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr >::iterator itEnd = supervoxel_clusters.end();

    supervoxelcloud->height = 1;
    supervoxelcloud->width = supervoxel_clusters.size();
    supervoxelcloud->points.resize(supervoxelcloud->width);
    pcl::PointXYZRGBA *pPt = &supervoxelcloud->points[0];
    for(; it != itEnd; ++it, ++pPt)
    {
      pcl::PointXYZRGBA centroid;
      it->second->getCentroidPoint(centroid);
      *pPt = centroid;
    }

    //    pcl::PointCloud< pcl::PointXYZRGBA>::Ptr voxelCentroidCloud = supervoxels->getVoxelCentroidCloud();
    cas.set(VIEW_CLOUD_SUPERVOXEL, *supervoxelcloud);

    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    supervoxels->getSupervoxelAdjacency(supervoxel_adjacency);

    return UIMA_ERR_NONE;
  }

  void addSupervoxelConnectionsToViewer(pcl::PointXYZRGBA &supervoxel_center,
                                        pcl::PointCloud< pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
                                        std::string supervoxel_name,
                                        pcl::visualization::PCLVisualizer &viewer)
  {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

    // Setup colors
    unsigned char red[3] = {255, 0, 0};
    unsigned char green[3] = {0, 255, 0};
    unsigned char blue[3] = {0, 0, 255};

    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");
    colors->InsertNextTupleValue(red);
    colors->InsertNextTupleValue(green);
    colors->InsertNextTupleValue(blue);


    //Iterate through all adjacent points, and add a center point to adjacent point pair
    pcl::PointCloud< pcl::PointXYZRGBA>::iterator adjacent_itr = adjacent_supervoxel_centers.begin();
    for(; adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr)
    {
      points->InsertNextPoint(supervoxel_center.data);
      points->InsertNextPoint(adjacent_itr->data);
    }
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    // Add the points to the dataset
    polyData->SetPoints(points);
    polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
    for(unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
    {
      polyLine->GetPointIds()->SetId(i, i);
    }
    cells->InsertNextCell(polyLine);
    // Add the lines to the dataset
    polyData->SetLines(cells);
    //    polyData->GetPointData()->SetScalars(colors);
    viewer.addModelFromPolyData(polyData, supervoxel_name);
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string centroidname = this->name + "_centroids";
    const std::string coloredvoxelname = this->name + "_voxels_colored";
    const std::string normalsname = this->name + "_supervoxel_normals";

    if(!firstRun)
    {
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, centroidname);
    }
    visualizer.removeAllPointClouds();
    visualizer.removeAllShapes();
    addCentroids(visualizer, centroidname);
    switch(dispMode)
    {
    case ALL:
      addVoxels(visualizer, coloredvoxelname);
      addAdjacency(visualizer);
      addNormals(visualizer, normalsname);
      break;
    case W_VOXELS:
      addVoxels(visualizer, coloredvoxelname);
      break;
    case W_VA:
      addVoxels(visualizer, coloredvoxelname);
      addAdjacency(visualizer);
      break;
    case W_VN:
      addVoxels(visualizer, coloredvoxelname);
      addNormals(visualizer, normalsname);
      break;
    case W_AN:
      addAdjacency(visualizer);
      addNormals(visualizer, normalsname);
      break;
    case W_ADJACENCY:
      addAdjacency(visualizer);
      break;
    case W_NORMALS:
      addNormals(visualizer, normalsname);
      break;
    case ADJACENCY:
      visualizer.removeAllPointClouds();
      addAdjacency(visualizer);
      break;
    case NONE:
      break;
    case TEST:
      filterAdjacency(visualizer);
      break;
    }
  }

  void addCentroids(pcl::visualization::PCLVisualizer &visualizer, const std::string &name)
  {
    //    pcl::PointCloud< pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud = supervoxels->getVoxelCentroidCloud();
    visualizer.addPointCloud(supervoxelcloud, name);
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, name);
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, name);
  }
  void addVoxels(pcl::visualization::PCLVisualizer &visualizer, const std::string &name)
  {
    pcl::PointCloud< pcl::PointXYZRGBA>::Ptr colored_voxel_cloud = supervoxels->getColoredVoxelCloud();
    visualizer.addPointCloud(colored_voxel_cloud, name);
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, name);
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, name);
  }
  void addNormals(pcl::visualization::PCLVisualizer &visualizer, const std::string &name)
  {
    pcl::PointCloud< pcl::PointNormal>::Ptr sv_normal_cloud = supervoxels->makeSupervoxelNormalCloud(supervoxel_clusters);
    visualizer.addPointCloudNormals<pcl::PointNormal> (sv_normal_cloud, 1, 0.05f, name);
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, normalsColor[0], normalsColor[1], normalsColor[2], name);
  }

  //TODO use boost adjaceny list for visualization. far from beeing done
  void addAdjacencyBoost()
  {
    pcl::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelAdjacencyList adjacencyList;
    supervoxels->getSupervoxelAdjacencyList(adjacencyList);

    boost::graph_traits<pcl::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelAdjacencyList>::vertex_iterator vi, viEnd, next;
    boost::tie(vi, viEnd) = boost::vertices(adjacencyList);
    for(next = vi; vi != viEnd; vi = next)
    {
      ++next;
    }
  }

  void addAdjacency(pcl::visualization::PCLVisualizer &visualizer)
  {
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    supervoxels->getSupervoxelAdjacency(supervoxel_adjacency);
    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    for(std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin(); label_itr != supervoxel_adjacency.end();)
    {
      //First get the label
      uint32_t supervoxel_label = label_itr->first;
      //Now get the supervoxel corresponding to the label
      pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

      //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
      pcl::PointCloud< pcl::PointXYZRGBA> adjacent_supervoxel_centers;
      std::multimap<uint32_t, uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
      for(; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
      {
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);
        adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
      }
      //Now we make a name for this polygon
      std::stringstream ss;
      ss << "supervoxel_" << supervoxel_label;
      //This function generates a "star" polygon mesh from the points given
      addSupervoxelConnectionsToViewer(supervoxel->centroid_, adjacent_supervoxel_centers, ss.str(), visualizer);
      label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
    }
  }

  void filterAdjacency(pcl::visualization::PCLVisualizer &visualizer)
  {
    visualizer.removeAllPointClouds();
    visualizer.removeAllShapes();
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    supervoxels->getSupervoxelAdjacency(supervoxel_adjacency);

    pcl::PointCloud< pcl::PointXYZRGBA>::Ptr filtered_supervoxel(new pcl::PointCloud< pcl::PointXYZRGBA>);
    for(std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin(); label_itr != supervoxel_adjacency.end();)
    {
      //First get the label
      uint32_t supervoxel_label = label_itr->first;
      //Now get the supervoxel corresponding to the label
      pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

      int threshold = 7 - supervoxel->centroid_.z * 0.375;

      int count = supervoxel_adjacency.count(supervoxel_label);
      if(count >= threshold)
      {
        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        pcl::PointCloud< pcl::PointXYZRGBA>::Ptr adjacent_supervoxel_centers(new pcl::PointCloud< pcl::PointXYZRGBA>);

        std::multimap<uint32_t, uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
        for(; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
        {

          uint32_t snd_label = adjacent_itr->second;
          int snd_count = supervoxel_adjacency.count(snd_label);
          pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = supervoxel_clusters.at(snd_label);
          int snd_threshold = 7 - neighbor_supervoxel->centroid_.z * 0.375;
          if(snd_count >= snd_threshold)
          {
            adjacent_supervoxel_centers->push_back(neighbor_supervoxel->centroid_);
            filtered_supervoxel->push_back(neighbor_supervoxel->centroid_);
          }
        }
        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        //This function generates a "star" polygon mesh from the points given
        addSupervoxelConnectionsToViewer(supervoxel->centroid_, *adjacent_supervoxel_centers, ss.str(), visualizer);

      }
      label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
    }
    const std::string foo = "fassdfasdf";
    visualizer.addPointCloud(filtered_supervoxel, foo);
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, foo);
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = out;
  }

  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case '1':
      dispMode = ALL;
      break;
    case '2':
      dispMode = W_VOXELS;
      break;
    case '3':
      dispMode = W_VA;
      break;
    case '4':
      dispMode = W_VN;
      break;
    case '5':
      dispMode = W_AN;
      break;
    case '6':
      dispMode = W_ADJACENCY;
      break;
    case '7':
      dispMode = W_NORMALS;
      break;
    case '8':
      dispMode = ADJACENCY;
      break;
    case '9':
      dispMode = NONE;
      break;
    case '0':
      dispMode = TEST;
      break;

    }
    return true;
  }
};

MAKE_AE(Supervoxels)
