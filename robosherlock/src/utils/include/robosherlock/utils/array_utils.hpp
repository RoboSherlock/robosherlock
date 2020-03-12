/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
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

#ifndef __ARRAY_UTILS_HPP__
#define __ARRAY_UTILS_HPP__

#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <numeric>

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

/** \brief Find an intersection between two vectors. The return vector contains unique values that are present in both vectors.
 *  \param[in] v1 first vector
 *  \param[in] v2 second vector
 *  \return vector intersection
 */
inline std::vector<int> Intersection(const std::vector<int> &v1, const std::vector<int> &v2)
{
  std::vector<int> intersect;
  std::vector<int> sortedV1(v1);
  std::vector<int> sortedV2(v2);

  std::sort(sortedV1.begin(), sortedV1.end());
  std::sort(sortedV2.begin(), sortedV2.end());

  std::set_intersection(sortedV1.begin(), sortedV1.end(), sortedV2.begin(), sortedV2.end(), std::back_inserter(intersect));

  return intersect;
}

/** \brief Find a union of two vectors. The return vector contains unique values that are present in both vectors.
 *  \param[in] v1 first vector
 *  \param[in] v2 second vector
 *  \return vector union
 */
inline std::vector<int> Union(const std::vector<int> &v1, const std::vector<int> &v2)
{
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

/** \brief Find a different of two vectors. The return vector contains unique values that are present in first vector but not second vector.
 *  \param[in] v1 first vector
 *  \param[in] v2 second vector
 *  \return vector union
 */
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

/** \brief get linear subscript from 2D array from 2D subscript
 *  \param[in] v    2D array
 *  \param[in] row
 *  \param[in] col
 *  \return linear subscript of 2D array (row = 1, col = 1 in 2x2 => subscript = 3)
 */
template<typename Type>
inline int matrixToLinear(std::vector< std::vector<Type> > &v, int row, int col)
{
  int result = 0;
  for(size_t it = 0; it < row; it++)
  {
    result += v[it].size();
  }
  result += col;

  return result;
}

/** \brief get 2D subscript from 2D array from linear subscript
 *  \param[in]   v           2D array
 *  \param[in]   linear_id   linear subscript
 *  \param[out]  row
 *  \param[out]  col
 *  \return false linear subscript is out of bound
 */
template<typename Type>
inline bool linearToMatrix(std::vector< std::vector<Type> > &v, int linear_id, int &row, int &col){
  int offset = 0;
  col = 0;
  for(row = 0; row < v.size(); row++)
  {
    offset += v[row].size();
    if(offset > linear_id)
    {
      col = linear_id - (offset - v[row].size());
      return true;
    }
  }

  return false;
}

/** \brief get mean value from vector
 *  \param[in]   v    vector
 *  \return mean value of vector as float
 */
template<typename Type>
inline float mean(std::vector<Type> &v)
{
  Type sum = std::accumulate(v.begin(), v.end(), 0.0);
  return static_cast<float> (sum) / static_cast<float>(v.size());
}

/** \brief get median value from vector
 *  \param[in]   v    vector
 *  \return median value of vector as float
 */
template<typename Type>
inline Type median(std::vector<Type> &v)
{
  std::vector<Type> copied(v);
  std::nth_element(copied.begin(), copied.begin() + copied.size()/2, copied.end());
  return copied[copied.size()/2];
}

/** \brief overrided function to output 2D array */
template<typename Type>
std::ostream& operator<<(std::ostream &output, std::vector< std::vector<Type> > &arr)
{
  for(size_t it = 0; it < arr.size(); it++)
  {
    output << "ID: ";
    for(size_t subIt = 0; subIt < arr[it].size(); subIt++)
    {
      output << arr[it][subIt] << ' ';
    }
    output << '\n';
  }

  return output;
}

/** \brief overrided function to output array */
template<typename Type>
std::ostream& operator<<(std::ostream &output, std::vector<Type> &arr)
{
  output << "Data: ";
  for(size_t it = 0; it < arr.size(); it++)
  {
    output << arr[it] << " ";
  }
  output << '\n';

  return output;
}

/** \brief Search all instances of target in vector and output as vector of indices
 *  \param[in]    arr              input vector
 *  \param[in]    target           target value
 *  \param[out]   searchIndices    instances indices
 *  \return number of found instances
 */
template<typename Type>
inline int vectorSearch(std::vector<Type> &arr, const Type target, std::vector<int> &searchIndices){
  searchIndices.clear();
  auto it = arr.begin();

  while(true)
  {
    it = std::find(it, arr.end(), target);
    if(it == arr.end())
    {
      break;
    }
    else
    {
      searchIndices.push_back(it - arr.end());
      it++;
    }
  }

  return searchIndices.size();
}

/** \brief Convert 2D array to linear array based on indices vector
 *  \param[in]     segmentDataIn           input 2D vector
 *  \param[out]    segmentDataOut          output linear vector
 *  \param[in]     indices                 indices
 */
template<typename Type>
inline void linearizeSegmentData(typename std::vector< std::vector<Type> > &segmentDataIn, typename std::vector<Type> &segmentDataOut, std::vector< std::vector<int> > indices = std::vector<int>(0))
{ // for both scores and Symmetries
  segmentDataOut.clear();

  for(size_t segmentIt = 0; segmentIt < segmentDataIn.size(); segmentIt++)
  {
    if(indices.size() != 0)
    {
      int dataId;
      for(size_t it = 0; it < indices[segmentIt].size(); it++)
      {
        dataId = indices[segmentIt][it];
        if(dataId >= 0 && dataId < segmentDataIn[segmentIt].size())
        {
          segmentDataOut.push_back(segmentDataIn[segmentIt][dataId]);
        }
      }
    }
    else
    {
      for(size_t it = 0; it < segmentDataIn[segmentIt].size(); it++)
      {
        segmentDataOut.push_back(segmentDataIn[segmentIt][it]);
      }
    }
  }
}

inline bool readGroundTruth (const std::string &filename, std::vector< std::vector<int> > &segmentation)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    std::cout << "[readGroundTruth] Could not open file for reading ('" << filename << "')." << std::endl;
    return false;
  }

  segmentation.resize(0);

  int numSegments;
  std::string line;
  file >> numSegments;

  if (numSegments < 0)
  {
    std::cout << "[readGroundTruth] Number of segments is smaller than 0, segmentation file is corrupted." << std::endl;
    std::cout << "[readGroundTruth] In file: " << filename << std::endl;
    return false;
  }
  else if (numSegments == 0)
  {
    std::cout << "[readGroundTruth] File contains 0 segments." << std::endl;
    std::cout << "[readGroundTruth] In file: " << filename << std::endl;
    return true;
  }

  file >> line;

  // Read segment point indices
  int numSegmentsRead = 0;
  bool done = false;
  file >> line;

  while (!done)
  {
    // Read segment header
    int segIdRead;
    file >> segIdRead;

    if (segIdRead != numSegmentsRead)
    {
      std::cout << "[readGroundTruth] Unexpected segment id. Expected " << numSegmentsRead << ", got " << segIdRead << std::endl;
      std::cout << "[readGroundTruth] Segmentation file is corrupted." << std::endl;
      std::cout << "[readGroundTruth] In file: " << filename << std::endl;
      return false;
    }

    numSegmentsRead++;
    segmentation.push_back(std::vector<int> ());
    file >> line;

    // Read point indices
    while (!file.eof())
    {
      file >> line;
      if (line == "--------------------" || line == "")
      {
        break;
      }
      else
      {
        int pointId = std::stoi(line);
        segmentation[segIdRead].push_back(pointId);
        line.clear();
      }
    }

    if (file.eof())
    {
      done = true;
    }
  }

  // Check that number of segments read is the same as number of segments in the header
  if (numSegments != numSegmentsRead)
  {
    std::cout << "[readGroundTruth] Expected " << numSegments << " segments, was able to read " << numSegmentsRead << " segments." << std::endl;
    std::cout << "[readGroundTruth] Segmentation file is corrupted." << std::endl;
    std::cout << "[readGroundTruth] In file: " << filename << std::endl;
  }

  return true;
}

#endif // __ARRAY_UTILS_HPP__
