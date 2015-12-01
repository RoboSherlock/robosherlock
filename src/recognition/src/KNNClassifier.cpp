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
#include <rs/recognition/ClassifierBase.h>

using namespace uima;

class KNNClassifier : public Classifier<cv::KNearest>
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

    model->train(descriptors, responses, cv::Mat(), isRegression, maxK);
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
    model->find_nearest(descriptors, kNN, bestResponse, neighborResponses, distances);
    responses = neighborResponses;
  }
};

MAKE_AE(KNNClassifier)
