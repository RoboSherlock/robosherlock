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
#include <rs/recognition/ClassifierBase.h>

using namespace uima;

#if CV_MAJOR_VERSION == 3
class KNNClassifier : public Classifier<cv::ml::KNearest>
#elif CV_MAJOR_VERSION == 2
class KNNClassifier : public Classifier<cv::KNearest>
#endif
{
private:
  int kNN;
  int maxK;
  bool isRegression;

public:
  KNNClassifier() : Classifier(__func__), kNN(1), maxK(32), isRegression(false)
  {
  }

protected:
  virtual TyErrorId train(AnnotatorContext &ctx, const cv::Mat &descriptors, const cv::Mat &responses)
  {
    if(ctx.isParameterDefined("kNN"))
    {
      ctx.extractValue("kNN", kNN);
    }
    if(ctx.isParameterDefined("maxK"))
    {
      ctx.extractValue("maxK", maxK);
    }
    if(ctx.isParameterDefined("isRegression"))
    {
      ctx.extractValue("isRegression", isRegression);
    }
#if CV_MAJOR_VERSION == 3
    cv::Ptr<cv::ml::TrainData> data;
    data->create(descriptors,cv::ml::ROW_SAMPLE,responses);
    model->train(data);
#elif CV_MAJOR_VERSION ==2
    model->train(descriptors, responses, cv::Mat(), isRegression, maxK);
#endif
    return UIMA_ERR_NONE;
  }

  virtual void classify(const cv::Mat &descriptors, const std::string &descriptorType, cv::Mat &responses, cv::Mat &distances)
  {
    MEASURE_TIME;
    if(descriptorType != "numerical")
    {
      return;
    }
    cv::Mat neighborResponses, bestResponse;
#if CV_MAJOR_VERSION == 2
    model->find_nearest(descriptors, kNN, bestResponse, neighborResponses, distances);
#elif CV_MAJOR_VERSION ==3
    model->findNearest(descriptors,kNN,bestResponse,neighborResponses,distances);
#endif
    responses = neighborResponses;
  }
};

MAKE_AE(KNNClassifier)
