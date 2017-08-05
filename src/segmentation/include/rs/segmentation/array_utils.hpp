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


inline std::vector<int> Difference(const std::vector<int> &v1, const std::vector<int> &v2)
{
  std::vector<int> v_difference;
  std::vector<int> v1_sorted(v1);
  std::vector<int> v2_sorted(v2);

  std::sort(v1_sorted.begin(), v1_sorted.end());
  std::sort(v2_sorted.begin(), v2_sorted.end());

  v1_sorted.erase(std::unique(v1_sorted.begin(), v1_sorted.end()), v1_sorted.end());
  v2_sorted.erase(std::unique(v2_sorted.begin(), v2_sorted.end()), v2_sorted.end());

  std::set_difference(v1_sorted.begin(), v1_sorted.end(), v2_sorted.begin(), v2_sorted.end(), std::back_inserter(v_difference));
      
  return v_difference;
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

template<typename Type>
inline float mean(std::vector<Type>& v){
  Type sum = std::accumulate(v.begin(), v.end(), 0.0);
  return static_cast<float> (sum) / static_cast<float>(v.size());
}

template<typename Type>
inline Type median(std::vector<Type>& v){
  std::vector<Type> copied(v);
  std::nth_element(copied.begin(), copied.begin() + copied.size()/2, copied.end());
  return copied[copied.size()/2];
}

template<typename Type>
ostream& operator<<(ostream& output, std::vector< std::vector<Type> >& arr){
  for(size_t it = 0; it < arr.size(); it++){
    output << "ID: ";
    for(size_t subIt = 0; subIt < arr[it].size(); subIt++){
      output << arr[it][subIt] << ' ';
    }
    output << '\n';
  }
  return output;
}

template<typename Type>
ostream& operator<<(ostream& output, std::vector<Type>& arr){
  output << "Data: ";
  for(size_t it = 0; it < arr.size(); it++){
    output << arr[it] << " ";
  }
  output << '\n';
  return output;
}

template<typename Type>
inline int vectorSearch(std::vector<Type>& arr, const Type target, std::vector<int>& searchIndices){
  searchIndices.clear();
  auto it = arr.begin();

  while(true){
    it = std::find(it, arr.end(), target);
    if(it == arr.end())
      break;
    else{
      searchIndices.push_back(it - arr.end());
      it++;
    }
  }
  return searchIndices.size();
}

#endif
