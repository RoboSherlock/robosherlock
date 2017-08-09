#ifndef DOWNSAMPLE_MAP_HPP
#define DOWNSAMPLE_MAP_HPP

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>

#include <rs/utils/output.h>

enum DownsampleMethod{
  AVERAGE,
  NEAREST_NEIGHBOR
};

template<typename PointT>
class DownsampleMap : public pcl::VoxelGrid<PointT>{
private:
  DownsampleMethod downsampleMethod;

  typename pcl::PointCloud<PointT>::Ptr dsCloud;
  std::vector< std::vector<int> > downsampleMap; // from downsample indices to original indices
  std::vector<int> reversedMap; // from original indices to downsample indices

  std::vector<int> nearestIndices; // store nearest index of each point for NEAREST NEIGHBOR method

  inline void reset(){
    dsCloud.reset(new pcl::PointCloud<PointT>);
    downsampleMap.clear();
    nearestIndices.clear();
  }

  //use when already set indices
  inline bool initDownsampleMap(){
    if(downsampleMap.empty()){
      if(dsCloud->points.empty()){
        outError("No downsampled cloud, please run filter first!");
        return false;
      }

      downsampleMap.resize(dsCloud->points.size());
      for(size_t it = 0; it < this->indices_->size(); it++){
        int pointId = this->indices_->at(it);
        int dsPointId;
        try
        {
          dsPointId = this->getCentroidIndex(this->input_->points[pointId]);
        }
        catch(...)
        {
          Eigen::Vector3f coordinate = this->input_->points[pointId].getVector3fMap();
          dsPointId = this->getCentroidIndexAt(this->getGridCoordinates(coordinate[0], coordinate[1], coordinate[2]));
          if(dsPointId == -1)
          {
            continue;
          }
        }

        downsampleMap[dsPointId].push_back(pointId);
      }
    }
    return true;
  }

  inline void initReversedMap(){
    if(reversedMap.empty()){
      initDownsampleMap();

      reversedMap.resize(this->indices_->size());
      for(size_t dsPointId = 0; dsPointId < downsampleMap.size(); dsPointId++)
      {
        for(size_t pointIdIt = 0; pointIdIt < downsampleMap[dsPointId].size(); pointIdIt++)
        {
          int pointId = downsampleMap[dsPointId][pointIdIt];
          reversedMap[pointId] = dsPointId;
        }
      }
    }
  }

  inline void initNearestPointIndices(){
    if(nearestIndices.empty()){
      initDownsampleMap();

      pcl::search::KdTree<PointT> search;
      std::vector<int> indices(1);
      std::vector<float> dists(1);

      for(size_t it = 0; it < dsCloud->points.size(); it++){
        search.setInputCloud(this->input_, boost::make_shared< std::vector<int> >(downsampleMap[it]));
        search.nearestKSearch(dsCloud->points[it], 1, indices, dists);
        nearestIndices.push_back(indices[0]);
      }
    }
  }

  virtual void applyFilter(pcl::PointCloud<PointT>& output){
    if(dsCloud->points.empty()){
      pcl::VoxelGrid<PointT>::applyFilter(*dsCloud);
    }

    if(downsampleMethod == AVERAGE){
      output = *dsCloud;
    }
    else if(downsampleMethod == NEAREST_NEIGHBOR){
      initNearestPointIndices();
      pcl::copyPointCloud(*this->input_, nearestIndices, output);
    }
  }

public:

  DownsampleMap() : downsampleMethod(AVERAGE), dsCloud(new pcl::PointCloud<PointT>), downsampleMap(), nearestIndices(){
    this->setSaveLeafLayout(true);
  }
  ~DownsampleMap() {}

  virtual void setInputCloud(typename pcl::PointCloud<PointT>::Ptr& cloud){
    pcl::VoxelGrid<PointT>::setInputCloud(cloud);
    this->reset();
  }

  virtual void setIndices(pcl::PointIndices::Ptr& indices){
    pcl::VoxelGrid<PointT>::setIndices(indices);
    this->reset();
  }

  inline void setDownsampleMethod(DownsampleMethod dsMethod){
    downsampleMethod = dsMethod;
  }

  inline void setLeafSize(const float leafSize){
    pcl::VoxelGrid<PointT>::setLeafSize(leafSize, leafSize, leafSize);
    this->reset();
  }

  inline void getDownsampleMap(std::vector< std::vector<int> >& dsMap){
    initDownsampleMap();
    dsMap = downsampleMap;
  }
  inline void getReversedMap(std::vector<int> &reversed_map){
    initReversedMap();
    reversed_map = reversedMap;
  }
  inline void getNearestNeighborMap(std::vector< int >& dsNearestMap){
    initNearestPointIndices();
    dsNearestMap = nearestIndices;
  }
};

//NOTE: this normal downsample method uses AVERAGE method
inline bool computeDownsampleNormals(pcl::PointCloud<pcl::Normal>::Ptr& in_normals,
                                     std::vector< std::vector<int> >& dsMap,
                                     std::vector<int>& dsNearestMap,
                                     DownsampleMethod method,
                                     pcl::PointCloud<pcl::Normal>::Ptr& out_normals)
{
  if(dsNearestMap.size() != dsMap.size()){
    outError("Inconsistent downsample map and nearest neighbors map!");
    return false;
  }
  out_normals.reset(new pcl::PointCloud<pcl::Normal>(dsMap.size(), 1));
  out_normals->points.resize(dsMap.size());

  if(method == AVERAGE){
      for(size_t dsPointId = 0; dsPointId < dsMap.size(); dsPointId++){
        int denom = dsMap[dsPointId].size();
        float dsNormX = 0.0f, dsNormY = 0.0f, dsNormZ = 0.0f;

        for(size_t pointIdIt = 0; pointIdIt < dsMap[dsPointId].size(); pointIdIt++){
          int pointId = dsMap[dsPointId][pointIdIt];
          if(pcl_isfinite(in_normals->points[pointId].normal_x) &&
             pcl_isfinite(in_normals->points[pointId].normal_y) &&
             pcl_isfinite(in_normals->points[pointId].normal_z))
          {
            dsNormX += in_normals->points[pointId].normal_x;
            dsNormY += in_normals->points[pointId].normal_y;
            dsNormZ += in_normals->points[pointId].normal_z;
          }
          else
            denom--;
        }

        if(denom != 0){
          dsNormX /= denom;
          dsNormY /= denom;
          dsNormZ /= denom;
          out_normals->points[dsPointId] = pcl::Normal(dsNormX, dsNormY, dsNormZ);
        }
        else{
          outWarn("Normal cloud is corrupted! Too many NaN normals");
        }
      }
  }
  else if(method == NEAREST_NEIGHBOR){
    for(size_t dsPointId = 0; dsPointId < dsNearestMap.size(); dsPointId++){
      int pointId = dsNearestMap[dsPointId];
      out_normals->points[dsPointId] = in_normals->points[pointId];
    }
  }
  return true;
}

inline bool upsample_cloud(const std::vector<int>& dsIndices,
                           const std::vector< std::vector<int> >& dsMap,
                           std::vector<int>& usIndices)
{
  if(dsMap.empty()){
    outError("Downsample map is null!");
    return false;
  }

  for(size_t it = 0; it < dsIndices.size(); it++)
    usIndices.insert(usIndices.end(), dsMap[dsIndices[it]].begin(), dsMap[dsIndices[it]].end());

  return true;
}

#endif
