/************************************************************
NOTE: This method is still in considering
************************************************************/

#ifndef OCCUPANCY_MAP_HPP
#define OCCUPANCY_MAP_HPP

#include <omp.h>
#include <mutex>

#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <rs/segmentation/array_utils.hpp>
#include <rs/segmentation/Geometry.hpp>
#include <rs/utils/output.h>

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

class OccupancyMap{
private:
  octomap::OcTree occupancy_tree;
  bool*** occupancy_map;
  uint16_t depth;
  uint16_t voxel_size;

  DynamicEDT3D* distance_map;
  float maxDist;

  octomap::point3d bounding_box_min;
  octomap::point3d bounding_box_max;

  octomap::OcTreeKey treeToDistMapOffset;

public:
  OccupancyMap() : occupancy_tree(0.0), occupancy_map(NULL), distance_map(NULL) {}
  ~OccupancyMap() { if (distance_map) delete distance_map; }

  template <typename PointT>
  bool setInputCloud(typename pcl::PointCloud<PointT>::Ptr &cloud){
    if(cloud->points.size() < 3){
      outError("Insufficient points in cloud!");
      return false;
    }

    octomap::PointCloud temp;
    for(size_t it = 0; it < cloud->points.size(); it++)
      temp.push_back(cloud->points[it].x, cloud->points[it].y, cloud->points[it].z);

    occupancy_tree.insertPointCloud(temp, octomap::point3d(0.0, 0.0, 0.0));
    occupancy_tree.updateInnerOccupancy();
    depth = occupancy_tree.getTreeDepth();
    voxel_size = occupancy_tree.getResolution();

    if(distance_map){
      distance_map->~DynamicEDT3D();
      delete distance_map;
      distance_map = NULL;
    }

    return true;
  }

  unsigned int getTreeDepth() {
    return depth;
  }

  float getTreeResolution() {
    return voxel_size;
  }

  octomap::OcTreeKey treeKeyToDistMapKey(octomap::OcTreeKey& key_oct){
    octomap::OcTreeKey key_dm;
    key_dm[0] = (key_oct[0] - treeKeyToDistMapKey[0]) / voxel_size;
    key_dm[1] = (key_oct[1] - treeKeyToDistMapKey[1]) / voxel_size;
    key_dm[2] = (key_oct[2] - treeKeyToDistMapKey[2]) / voxel_size;
    return key_dm;
  }

  octomap::OcTreeKey distMapKeyToTreeKey(octomap::OcTreeKey& key_dm){
    octomap::OcTreeKey key_oct;
    key_oct[0] = key_dm[0] * voxel_size + treeKeyToDistMapKey[0];
    key_oct[1] = key_dm[1] * voxel_size + treeKeyToDistMapKey[1];
    key_oct[2] = key_dm[2] * voxel_size + treeKeyToDistMapKey[2];
    return key_oct;
  }
};


#endif
