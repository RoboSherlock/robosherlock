/* Copyright (c) 2013, Thiemo Wiedemeyer  <wiedemeyer@informatik.uni-bremen.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
  Classifier(const std::string &classifierName) : Annotator(), model(new T()), classifierName(classifierName)
  {
  }

  uima::TyErrorId initialize(uima::AnnotatorContext &ctx)
  {
    outInfo("initialize");

    ctx.extractValue("modelFile", modelFile);

    cv::Mat descriptors, responses;

    cv::FileStorage file(modelFile, cv::FileStorage::READ);
    file["classnames"] >> classNames;
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
