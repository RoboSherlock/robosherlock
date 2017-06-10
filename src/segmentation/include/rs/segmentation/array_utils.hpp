#ifndef ARRAY_UTILS_HPP
#define ARRAY_UTILS_HPP

#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

// return a vector with unique elements that are both in v1 and v2
inline std::vector<int> Intersection(const std::vector<int>& v1, const std::vector<int>& v2){
  std::vector<int> intersect;
  std::vector<int> sortedV1(v1);
  std::vector<int> sortedV2(v2);

  std::sort(sortedV1.begin(), sortedV1.end());
  std::sort(sortedV2.begin(), sortedV2.end());

  std::set_intersection(sortedV1.begin(), sortedV1.end(), sortedV2.begin(), sortedV2.end(), std::back_inserter(intersect));

  return intersect;
}

// return a vector with all unique elements in v1 and v2
inline std::vector<int> Union(const std::vector<int>& v1, const std::vector<int>& v2){
  std::vector<int> union_vec;
  std::vector<int> sortedV1(v1);
  std::vector<int> sortedV2(v2);

  std::sort(sortedV1.begin(), sortedV1.end());
  std::sort(sortedV2.begin(), sortedV2.end());

  sortedV1.erase(std::unique(sortedV1.begin(), sortedV1.end()), sortedV1.end());
  sortedV2.erase(std::unique(sortedV2.begin(), sortedV2.end()), sortedV2.end());

  std::set_union(sortedV1.begin(), sortedV1.end(), sortedV2.begin(), sortedV2.end(), std::back_inserter(union_vec));

  return union_vec;
}

// get linear subscript from 2D array
template<typename Type>
inline int matrixToLinear(std::vector< std::vector<Type> >& v, int row, int col){
  int result = 0;
  for(size_t it = 0; it < row; it++)
    result += v[it].size();
  result += col;
  return result;
}

template<typename Type>
inline bool linearToMatrix(std::vector< std::vector<Type> >& v, int linear_id, int& row, int& col){
  int offset = 0;
  col = 0;
  for(row = 0; row < v.size(); row++){
    offset += v[row].size();
    if(offset > linear_id){
      col = linear_id - (offset - v[row].size());
      return true;
    }
  }

  return false;
}

#endif
