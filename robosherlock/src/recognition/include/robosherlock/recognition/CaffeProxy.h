/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
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
 *
 * based on the caffe cpp tutorial from
 * http://caffe.berkeleyvision.org/gathered/examples/cpp_classification.html
 */

#ifndef CAFFE_CLASSIFIER_HEADER
#define CAFFE_CLASSIFIER_HEADER

#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace caffe;
using std::string;

/* Return the indices of the top N values of vector v. */
std::vector<int> Argmax(const std::vector<float> &v, int N);

/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> Prediction;

class CaffeProxy
{
public:
  CaffeProxy(const string &model_file,
             const string &trained_file,
             const string &mean_file,
             const string &label_file);

  std::vector<Prediction> Classify(const cv::Mat &img, int N = 5);

  std::vector<float> extractFeature(const cv::Mat &img, std::string layer = "fc7");

private:
  void SetMean(const string &mean_file);

  std::vector<float> Predict(const cv::Mat &img);

  void WrapInputLayer(std::vector<cv::Mat> *input_channels);

  void Preprocess(const cv::Mat &img,
                  std::vector<cv::Mat> *input_channels);


private:
  shared_ptr<Net<float> > net_;
  cv::Size input_geometry_;
  int num_channels_;
  cv::Mat mean_;
  std::vector<string> labels_;
};

#endif
