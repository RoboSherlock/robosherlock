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

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>

// RS
#include <robosherlock/conversion/conversion.h>
#include <robosherlock/types/pcl_types.h>

namespace rs
{
namespace conversion
{

template<typename PointT>
void from_aux(const uima::FeatureStructure &fs, pcl::PointCloud<PointT> &output)
{
  rs::PointCloud cloud(fs);
  assertWithMsg(std::string(TypeTrait<PointT>::name()) == cloud.point_type(), "Incompatible point type!");
  std_msgs::Header header;
  from(cloud.header(), header);
  output = pcl::PointCloud<PointT>();
  output.width = cloud.width();
  output.height = cloud.height();
  output.resize(output.width * output.height);
  pcl_conversions::toPCL(header, output.header);

  const uima::Feature &feature = fs.getType().getFeatureByBaseName("points");
  const uima::ByteArrayFS &arrayFS = fs.getByteArrayFSValue(feature);
  arrayFS.copyToArray(0, (char *)output.points.data(), 0, arrayFS.size());
  //memcpy(&output.points[0], &cloud.points()[0], cloud.points.size());

  output.is_dense = cloud.is_dense();
}

//in order to difffferentiate between normals and reglar cloud...not needed, but if we define a type for Normals we might as well use it


template<typename PointT>
uima::FeatureStructure to_aux(uima::CAS &cas, const pcl::PointCloud<PointT> &input)
{
  rs::PointCloud cloud = rs::create<rs::PointCloud >(cas);

  std_msgs::Header header;
  pcl_conversions::fromPCL(input.header, header);
  cloud.header(to(cas, header));
  cloud.width(input.width);
  cloud.height(input.height);
  cloud.is_dense(input.is_dense);
  cloud.point_type(TypeTrait<PointT>::name()); // make sure all possible PointT get a TypeTrait

  size_t size = input.size() * sizeof(PointT);
  //std::vector<char> data(size);
  //std::memcpy(&data[0], &input.points[0], size);
  //cloud.points(data);
  uima::FeatureStructure &fs = cloud;
  const uima::Feature &feature = fs.getType().getFeatureByBaseName("points");
  uima::ByteArrayFS arrayFS = cas.createByteArrayFS(size);
  arrayFS.copyFromArray((char *)input.points.data(), 0, size, 0);
  fs.setFSValue(feature, arrayFS);
  return cloud;
}


template<>
void from(const uima::FeatureStructure &fs, pcl::PointCloud<pcl::Normal> &output)
{
  rs::NormalsCloud cloud(fs);
  assertWithMsg(std::string("Normal") == cloud.point_type(), "Incompatible point type!");
  std_msgs::Header header;
  from(cloud.header(), header);
  output = pcl::PointCloud<pcl::Normal>();
  output.width = cloud.width();
  output.height = cloud.height();
  output.resize(output.width * output.height);
  pcl_conversions::toPCL(header, output.header);

  const uima::Feature &feature = fs.getType().getFeatureByBaseName("points");
  const uima::ByteArrayFS &arrayFS = fs.getByteArrayFSValue(feature);
  arrayFS.copyToArray(0, (char *)output.points.data(), 0, arrayFS.size());
  //memcpy(&output.points[0], &cloud.points()[0], cloud.points.size());

  output.is_dense = cloud.is_dense();
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const pcl::PointCloud<pcl::Normal> &input)
{
  rs::NormalsCloud cloud = rs::create<rs::NormalsCloud >(cas);

  std_msgs::Header header;
  pcl_conversions::fromPCL(input.header, header);
  cloud.header(to(cas, header));
  cloud.width(input.width);
  cloud.height(input.height);
  cloud.is_dense(input.is_dense);
  cloud.point_type("Normal"); // make sure all possible PointT get a TypeTrait

  size_t size = input.size() * sizeof(pcl::Normal);
  //std::vector<char> data(size);
  //std::memcpy(&data[0], &input.points[0], size);
  //cloud.points(data);
  uima::FeatureStructure &fs = cloud;
  const uima::Feature &feature = fs.getType().getFeatureByBaseName("points");
  uima::ByteArrayFS arrayFS = cas.createByteArrayFS(size);
  arrayFS.copyFromArray((char *)input.points.data(), 0, size, 0);
  fs.setFSValue(feature, arrayFS);
  return cloud;
}

template<>
void from(const uima::FeatureStructure &fs, pcl::PointCloud<pcl::PointXYZ> &output)
{
  from_aux(fs, output);
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const pcl::PointCloud<pcl::PointXYZ> &input)
{
  return to_aux(cas, input);
}

template<>
void from(const uima::FeatureStructure &fs, pcl::PointCloud<pcl::RGB> &output)
{
  from_aux(fs, output);
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const pcl::PointCloud<pcl::RGB> &input)
{
  return to_aux(cas, input);
}

template<>
void from(const uima::FeatureStructure &fs, pcl::PointCloud<pcl::PointXYZRGB> &output)
{
  from_aux(fs, output);
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const pcl::PointCloud<pcl::PointXYZRGB> &input)
{
  return to_aux(cas, input);
}

template<>
void from(const uima::FeatureStructure &fs, pcl::PointCloud<pcl::PointXYZRGBA> &output)
{
  from_aux(fs, output);
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const pcl::PointCloud<pcl::PointXYZRGBA> &input)
{
  return to_aux(cas, input);
}

//template<>
//void from(const uima::FeatureStructure &fs, pcl::PointCloud<pcl::Normal> &output)
//{
//  from_aux(fs, output);
//}

//template<>
//uima::FeatureStructure to(uima::CAS &cas, const pcl::PointCloud<pcl::Normal> &input)
//{
//  return to_aux(cas, input);
//}

template<>
void from(const uima::FeatureStructure &fs, pcl::PointCloud<pcl::VFHSignature308> &output)
{
  from_aux(fs, output);
}

template<>
uima::FeatureStructure to(uima::CAS &cas, const pcl::PointCloud<pcl::VFHSignature308> &input)
{
  return to_aux(cas, input);
}

template<>
void from(const uima::FeatureStructure &fs, pcl::PointIndices &output)
{
  rs::PointIndices pi(fs);
  std_msgs::Header header;
  from(pi.header(), header);
  pcl_conversions::toPCL(header, output.header);

  const uima::Feature &feature = fs.getType().getFeatureByBaseName("indices");
  const uima::IntArrayFS &arrayFS = fs.getIntArrayFSValue(feature);
  try
  {
    const size_t size = arrayFS.size();
    output.indices.resize(size);
    arrayFS.copyToArray(0, output.indices.data(), 0, size);
  }
  catch(uima::InvalidFSObjectException ex)
  {
    //this is fine. We can have clusters that don't have 3D points
    outDebug("No point indices to convert");
  }

}

template<>
uima::FeatureStructure to(uima::CAS &cas, const pcl::PointIndices &input)
{
  rs::PointIndices pi = rs::create<rs::PointIndices>(cas);
  std_msgs::Header header;
  pcl_conversions::fromPCL(input.header, header);

  pi.header(to(cas, header));
  //pi.indices.set(input.indices);

  uima::FeatureStructure &fs = pi;
  const uima::Feature &feature = fs.getType().getFeatureByBaseName("indices");
  const size_t size = input.indices.size();
  uima::IntArrayFS arrayFS = cas.createIntArrayFS(size);
  if(input.indices.size()>0)
  {
    arrayFS.copyFromArray(input.indices.data(), 0, size, 0);
    fs.setFSValue(feature, arrayFS);
  }
  return pi;
}

}
}
