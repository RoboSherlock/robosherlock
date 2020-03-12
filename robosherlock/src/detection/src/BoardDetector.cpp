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

// UIMA
#include <uima/api.hpp>

#include <opencv2/opencv.hpp>

// RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/DrawingAnnotator.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/utils/output.h>

using namespace uima;

class BoardDetector : public DrawingAnnotator
{
private:
  enum
  {
    CHESS,
    CIRCLE
  } boardType;
  cv::Mat pointsImage;
  cv::Mat pointsWorld;
  size_t boardRows;
  size_t boardCols;
  float boardDistX;
  float boardDistY;
  cv::Size boardSize;
  bool foundBoard;
  cv::Mat image;

public:
  BoardDetector() : DrawingAnnotator(__func__), boardType(CIRCLE), boardRows(7), boardCols(7), boardDistX(0.02), boardDistY(0.02), foundBoard(false)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    if(ctx.isParameterDefined("boardRows"))
    {
      ctx.extractValue("boardRows", boardRows);
    }
    if(ctx.isParameterDefined("boardCols"))
    {
      ctx.extractValue("boardCols", boardCols);
    }
    if(ctx.isParameterDefined("boardDistX"))
    {
      ctx.extractValue("boardDistX", boardDistX);
    }
    if(ctx.isParameterDefined("boardDistY"))
    {
      ctx.extractValue("boardDistY", boardDistY);
    }
    if(ctx.isParameterDefined("boardType"))
    {
      std::string sType;
      ctx.extractValue("boardType", sType);
      if(sType == "circle")
      {
        boardType = CIRCLE;
      }
      else if(sType == "chess")
      {
        boardType = CHESS;
      }
    }

    boardSize = cv::Size(boardCols, boardRows);

    // Create world points
    pointsWorld = cv::Mat(boardRows * boardCols, 3, CV_32F);
    for(size_t r = 0; r < pointsWorld.rows; ++r)
    {
      float *it = pointsWorld.ptr<float>(r);
      *it++ = boardDistX * (r % boardCols);
      *it++ = boardDistY * (r / boardCols);
      *it++ = 0;
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

private:
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_COLOR_IMAGE_HD, image);

    foundBoard = false;
    switch(boardType)
    {
    case CIRCLE:
      foundBoard = cv::findCirclesGrid(image, boardSize, pointsImage);
      break;
    case CHESS:
      foundBoard = cv::findChessboardCorners(image, boardSize, pointsImage, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

      if(foundBoard)
      {
        cv::cornerSubPix(image, pointsImage, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      }
      break;
    }

    if(foundBoard)
    {
      rs::Board board = rs::create<rs::Board>(tcas);
      board.pointsImage(rs::conversion::to(tcas, pointsImage));
      board.pointsWorld(rs::conversion::to(tcas, pointsWorld));
      scene.annotations.append(board);
    }
    else
    {
      outInfo("no board found!");
    }

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp = image.clone();
    if(foundBoard)
    {
      cv::drawChessboardCorners(disp, boardSize, pointsImage, foundBoard);
    }
  }
};

MAKE_AE(BoardDetector)
