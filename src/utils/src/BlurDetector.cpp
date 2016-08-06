/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
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

// RS
#include <rs/utils/BlurDetector.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>

#define BLUR_FUNC funcSobelStdDevOptimized

std::list<double> BlurDetector::results;
std::list<bool> BlurDetector::isBlurred;

BlurDetector::BlurDetector() : threshold(0.01), maxHist(10), minHist(3), waitBlur(5), maxStatSize(500)
{
  sum = 0;
  avg = 0;
  wasBlurred = 0;
}

double BlurDetector::funcLaplaceSum(const cv::Mat &img)
{
  cv::Mat lap;
  cv::Laplacian(img, lap, CV_32F);
  return cv::sum(lap)[0];
}

double BlurDetector::funcLaplaceMean(const cv::Mat &img)
{
  cv::Mat lap;
  cv::Laplacian(img, lap, CV_32F);
  return cv::mean(lap)[0];
}

double BlurDetector::funcLaplaceStdDev(const cv::Mat &img)
{
  cv::Mat lap;
  cv::Laplacian(img, lap, CV_32F);
  cv::Scalar mu, sigma;
  cv::meanStdDev(lap, mu, sigma);

  return (sigma.val[0] * sigma.val[0]);
}

double BlurDetector::funcSobelSum(const cv::Mat &img)
{
  cv::Mat dx, dy;
  cv::Sobel(img, dx, CV_32F, 1, 0, 3);
  cv::Sobel(img, dy, CV_32F, 0, 1, 3);
  cv::magnitude(dx, dy, dx);
  return cv::sum(dx)[0];
}

double BlurDetector::funcSobelMean(const cv::Mat &img)
{
  cv::Mat dx, dy;
  cv::Sobel(img, dx, CV_32F, 1, 0, 3);
  cv::Sobel(img, dy, CV_32F, 0, 1, 3);
  cv::magnitude(dx, dy, dx);
  return cv::mean(dx)[0];
}

double BlurDetector::funcSobelStdDev(const cv::Mat &img)
{
  cv::Mat dx, dy;
  cv::Sobel(img, dx, CV_32F, 1, 0, 3);
  cv::Sobel(img, dy, CV_32F, 0, 1, 3);
  cv::magnitude(dx, dy, dx);
  cv::Scalar mu, sigma;
  cv::meanStdDev(dx, mu, sigma);

  return (sigma.val[0] * sigma.val[0]) / mu.val[0];
}

double BlurDetector::funcSobelStdDevOptimized(const cv::Mat &img)
{
  const size_t height = img.rows - 2;
  const size_t width = img.cols - 2;
  const size_t n = height * width;
  float mean = 0;
  float stdDev = 0;
  cv::Mat mag(height, width, CV_32F);

  #pragma omp parallel for
  for(size_t r = 0; r < height; ++r)
  {
    const uint8_t *it11, *it12, *it13, *it21, *it23, *it31, *it32, *it33;
    it11 = img.ptr<uint8_t>(r);
    it12 = it11 + 1;
    it13 = it11 + 2;
    it21 = img.ptr<uint8_t>(r + 1);
    it23 = it21 + 2;
    it31 = img.ptr<uint8_t>(r + 2);
    it32 = it31 + 1;
    it33 = it31 + 2;
    float *itO = mag.ptr<float>(r);
    float localMean = 0;

    for(size_t c = 0; c < width; ++c, ++it11, ++it12, ++it13, ++it21, ++it23, ++it31, ++it32, ++it33, ++itO)
    {
      const int t1 = *it11 - *it33;
      const int t2 = *it13 - *it31;
      const int dx = (t1 - t2 + ((*it21 - *it23) << 1));
      const int dy = (t1 + t2 + ((*it12 - *it32) << 1));
      const float mag = sqrtf(dx * dx + dy * dy);
      localMean += mag;
      *itO = mag;
    }

    #pragma omp atomic
    mean += localMean;
  }
  mean /= n;

  for(size_t r = 0; r < height; ++r)
  {
    const float *it = mag.ptr<float>(r);

    for(size_t c = 0; c < width; ++c, ++it)
    {
      const float dev = *it - mean;
      stdDev += (dev * dev);
    }
  }
  stdDev /= n;

  return stdDev / mean;
}

double BlurDetector::funcModifiedLaplace(const cv::Mat &img)
{
  cv::Mat M = (cv::Mat_<double>(3, 1) << -1, 2, -1);
  cv::Mat G = cv::getGaussianKernel(3, -1, CV_64F);

  cv::Mat Lx;
  cv::sepFilter2D(img, Lx, CV_64F, M, G);

  cv::Mat Ly;
  cv::sepFilter2D(img, Ly, CV_64F, G, M);

  cv::Mat FM = cv::abs(Lx) + cv::abs(Ly);

  return cv::mean(FM).val[0];
}

double BlurDetector::funcTenengrad(const cv::Mat &img)
{
  cv::Mat Gx, Gy;
  cv::Sobel(img, Gx, CV_64F, 1, 0, 3);
  cv::Sobel(img, Gy, CV_64F, 0, 1, 3);

  cv::Mat FM = Gx.mul(Gx) + Gy.mul(Gy);

  return cv::mean(FM).val[0];
}

double BlurDetector::funcNormalizedGraylevelVariance(const cv::Mat &img)
{
  cv::Scalar mu, sigma;
  cv::meanStdDev(img, mu, sigma);

  return (sigma.val[0] * sigma.val[0]) / mu.val[0];
}

bool BlurDetector::detectBlur(const cv::Mat &image)
{
  cv::Mat grey;
  cv::cvtColor(image, grey, CV_BGR2GRAY);

  double result = BLUR_FUNC(grey);
  bool blurred = detectBlur(result);

  results.push_back(result);
  isBlurred.push_back(blurred);

  if(results.size() == maxStatSize)
  {
    results.pop_front();
    isBlurred.pop_front();
  }

  return blurred;
}

bool BlurDetector::detectBlur(const double result)
{
  bool blurred = history.size() < minHist;

  double diff = avg - result;
  if(diff > threshold * avg)
  {
    wasBlurred = waitBlur;
    blurred = true;
  }

  if(history.size() == maxHist)
  {
    sum -= history.front();
    history.pop_front();
  }
  history.push_back(result);
  sum += result;
  avg = sum / history.size();

  if(!blurred && wasBlurred)
  {
    blurred = true;
    --wasBlurred;
  }

  return blurred;
}

