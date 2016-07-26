/*
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
