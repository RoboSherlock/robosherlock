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
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>

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
