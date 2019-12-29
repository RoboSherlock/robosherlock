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

#include <robosherlock/utils/DepthImageProcessing.h>
#include <robosherlock/utils/output.h>

void selectiveBinomialFiltering(const cv::Mat &input, cv::Mat &output, const uint16_t minValue, const uint16_t maxValue)
{
  output.create(input.rows, input.cols, input.type());

  int rowsEnd = input.rows - 1;
  int colsEnd = input.cols - 1;

  for(int y = 1; y < rowsEnd; y++)
  {
    const uint16_t *predecessor = input.ptr<uint16_t>(y - 1);
    const uint16_t *center = input.ptr<uint16_t>(y);
    const uint16_t *successor = input.ptr<uint16_t>(y + 1);

    uint16_t *centerOutput = output.ptr<uint16_t>(y);
    for(int x = 1; x < colsEnd; x++)
    {
      // 3x3-Umgebung
      // v0 v1 v2
      // v3 v4 v5
      // v6 v7 v8

      uint16_t sum = 0;
      int count = 0;

      const uint16_t v0 = predecessor[x - 1];
      if(v0 >= minValue && v0 <= maxValue)
      {
        sum += v0;
        ++count;
      }
      const uint16_t v2 = predecessor[x + 1];
      if(v2 >= minValue && v2 <= maxValue)
      {
        sum += v2;
        ++count;
      }
      const uint16_t v6 = successor[x - 1];
      if(v6 >= minValue && v6 <= maxValue)
      {
        sum += v6;
        ++count;
      }
      const uint16_t v8 = successor[x + 1];
      if(v8 >= minValue && v8 <= maxValue)
      {
        sum += v8;
        ++count;
      }

      // Die Seiten der Filtermaske
      const uint16_t v1 = predecessor[x];
      if(v1 >= minValue && v1 <= maxValue)
      {
        sum += v1 + v1;
        count += 2;
      }
      const uint16_t v7 = successor[x];
      if(v7 >= minValue && v7 <= maxValue)
      {
        sum += v7 + v7;
        count += 2;
      }
      const uint16_t v3 = center[x - 1];
      if(v3 >= minValue && v3 <= maxValue)
      {
        sum += v3 + v3;
        count += 2;
      }
      const uint16_t v5 = center[x + 1];
      if(v5 >= minValue && v5 <= maxValue)
      {
        sum += v5 + v5;
        count += 2;
      }

      // Das Zentrum der Filtermaske
      const uint16_t v4 = center[x];
      if(v4 >= minValue && v4 <= maxValue)
      {
        sum += 4 * v4;
        count += 4;
      }

      // Ergebnis setzen
      if(count)
      {
        centerOutput[x] = sum / count;
      }
      else
      {
        centerOutput[x] = v4;
      }
    }
    // linken und rechten Bildrand nicht filtern
    centerOutput[0] = center[0];
    centerOutput[input.cols - 1] = center[input.cols - 1];
  }
}

void selectiveDownsampling(const cv::Mat &input, cv::Mat &output, const uint16_t minValue, const uint16_t maxValue)
{
  int cols = input.cols, rows = input.rows;

  // Ausgabegröße
  const int resultWidth = cols / 2;
  const int resultHeight = rows / 2;

  output.create(resultHeight, resultWidth, input.type());

  for(int y = 1, yOut = 0; y < rows; y += 2, ++yOut)
  {

    const uint16_t *predecessor = input.ptr<uint16_t>(y - 1);
    const uint16_t *center = input.ptr<uint16_t>(y);

    uint16_t *centerOut = output.ptr<uint16_t>(yOut);
    for(int x = 1, xOut = 0; x < cols; x += 2, ++xOut)
    {
      float sum = 0.f;
      int count = 0;

      const uint16_t v0 = predecessor[x - 1];
      if(v0 >= minValue && v0 <= maxValue)
      {
        sum += v0;
        ++count;
      }

      const uint16_t v1 = predecessor[x];
      if(v1 >= minValue && v1 <= maxValue)
      {
        sum += v1;
        ++count;
      }

      const uint16_t v2 = center[x - 1];
      if(v2 >= minValue && v2 <= maxValue)
      {
        sum += v2;
        ++count;
      }

      const uint16_t v3 = center[x];
      if(v3 >= minValue && v3 <= maxValue)
      {
        sum += v3;
        ++count;
      }

      if(count > 0)
      {
        centerOut[xOut] = sum / count;
      }
      else
      {
        centerOut[xOut] = v0;
      }
    }
  }
}

namespace rs
{
namespace DepthImageProcessing
{

void fillHoles(cv::Mat &image)
{
  cv::Mat output(image.rows, image.cols, image.type());
  std::vector<cv::Mat> pyramid;

  int rows = image.rows;
  int cols = image.cols;

  for(int y = 0; y < rows; ++y)
  {
    const uint16_t *center = image.ptr<uint16_t>(y);
    uint16_t *output_center = output.ptr<uint16_t>(y);
    for(int x = 0; x < cols; ++x)
    {
      uint16_t depth = center[x];
      if(depth == 0)
      {
        int pyramidLayer = -1;
        uint16_t replacement = 0;
        int xPyr = x;
        int yPyr = y;
        int pyramidTop = 0;

        do
        {
          ++pyramidLayer;
          xPyr /= 2;
          yPyr /= 2;

          if(pyramidLayer >= pyramid.size())
          {
            cv::Mat lowerPyrLevel = image.clone();
            if(pyramidLayer > 0)
            {
              lowerPyrLevel = pyramid[pyramidLayer - 1];
            }

            cv::Mat smoothed;
            selectiveBinomialFiltering(lowerPyrLevel, smoothed, 1, 10000);

            cv::Mat upperPyrLevel;
            selectiveDownsampling(smoothed, upperPyrLevel, 1, 10000);
            pyramid.push_back(upperPyrLevel);

          }
          cv::Mat pyrLevel = pyramid[pyramidLayer];
          if(MIN(pyrLevel.cols, pyrLevel.rows) < 10)
          {
            pyramidTop = 1;
          }

          if(xPyr < pyrLevel.cols && yPyr < pyrLevel.rows)
          {
            replacement = image.at<uint16_t>(yPyr, xPyr);
          }
          else
          {
            pyramidTop = 1;
          }
        }
        while(!replacement && !pyramidTop);
        output_center[x] = replacement;
      }
    }
  }
  image = output;
}
void project(const cv::Mat &depth, const cv::Mat &color, const cv::Mat &alpha, const cv::Mat &lookupX, const cv::Mat &lookupY, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  cloud->height = depth.rows;
  cloud->width = depth.cols;
  cloud->is_dense = false;
  cloud->points.resize(cloud->height * cloud->width);

  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  #pragma omp parallel for
  for(size_t r = 0; r < (size_t)depth.rows; ++r)
  {
    pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
    const uint16_t *itD = depth.ptr<uint16_t>(r);
    const uint8_t *itA = alpha.ptr<uint8_t>(r);
    const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
    const float y = lookupY.at<float>(0, r);
    const float *itX = lookupX.ptr<float>();

    for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itA, ++itX)
    {
      register const float depthValue = *itD / 1000.0f;
      // Check for invalid measurements
      if(depthValue == 0.0f)
      {
        // not valid
        itP->x = itP->y = itP->z = badPoint;
        itP->rgba = 0;
        continue;
      }
      itP->z = depthValue;
      itP->x = *itX * depthValue;
      itP->y = y * depthValue;
      itP->b = itC->val[0];
      itP->g = itC->val[1];
      itP->r = itC->val[2];
      itP->a = 255 - *itA;
    }
  }
}

}
}
