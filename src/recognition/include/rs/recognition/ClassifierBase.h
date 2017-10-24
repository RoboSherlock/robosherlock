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

// UIMA
#include <uima/api.hpp>

#include <opencv2/opencv.hpp>

// RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>

/**
 *Base class for annotators that use Opencv ML module for classification
 */
template<class T>
class Classifier : public uima::Annotator
{
protected:
  cv::Ptr<T> model;
  std::string modelFile;
  const std::string classifierName;
  std::vector<std::string> classNames;

public:
  Classifier(const std::string &classifierName) : Annotator(), classifierName(classifierName)
  {
#if CV_MAJOR_VERSION == 3
   outInfo("Something needs to happen here");
   model = T::create();
#elif
   model = cv::makePtr<T>();
#endif
  }

  uima::TyErrorId initialize(uima::AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("modelFile", modelFile);

    cv::Mat descriptors, responses;

    cv::FileStorage file(modelFile, cv::FileStorage::READ);
    cv::FileNode node = file["classnames"];
    for(auto it = node.begin();it!=node.end();++it)
    {
      classNames.push_back(*it);
    }
    file["descriptors"] >> descriptors;
    file["responses"] >> responses;
    file.release();

    return train(ctx, descriptors, responses);
  }

  uima::TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  uima::TyErrorId process(uima::CAS &tcas, uima::ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    for(size_t i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster &cluster = clusters[i];
      std::vector<rs::Features> features;
      cluster.annotations.filter(features);
      for(size_t j = 0; j < features.size(); ++j)
      {
        rs::Features &feats = features[j];
        cv::Mat descriptors, responses, distances;

        rs::conversion::from(feats.descriptors(), descriptors);
        classify(descriptors, feats.descriptorType(), responses, distances);
        if(!responses.empty())
        {
          rs::Response response = rs::create<rs::Response>(tcas);
          response.classifier(classifierName);
          response.classNames(classNames);
          response.response(rs::conversion::to(tcas, responses));
          response.distances(rs::conversion::to(tcas, distances));
          feats.response.append(response);
        }
      }
    }

    return UIMA_ERR_NONE;
  }

protected:
  virtual uima::TyErrorId train(uima::AnnotatorContext &ctx, const cv::Mat &descriptors, const cv::Mat &responses) = 0;
  virtual void classify(const cv::Mat &descriptors, const std::string &descriptorType, cv::Mat &responses, cv::Mat &distances) = 0;
};
