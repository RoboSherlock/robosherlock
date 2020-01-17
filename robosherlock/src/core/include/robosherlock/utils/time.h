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

#ifndef RS_UTIL_TIME_H_
#define RS_UTIL_TIME_H_

#include <cmath>
#include <string>
#include <chrono>
#include <robosherlock/utils/output.h>

namespace rs
{

class StopWatch
{
protected:
  std::chrono::high_resolution_clock::time_point start_time;

public:
  /** \brief Constructor. */
  StopWatch() : start_time(std::chrono::high_resolution_clock::now())
  {
  }

  /** \brief Destructor. */
  virtual ~StopWatch() {}

  /** \brief Retrieve the time in milliseconds spent since the last call to \a reset(). */
  inline double
  getTime()
  {
    return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count() * 1000.0;
  }

  /** \brief Retrieve the time in seconds spent since the last call to \a reset(). */
  inline double
  getTimeSeconds()
  {
    return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();
  }

  /** \brief Reset the stopwatch to 0. */
  inline void
  reset()
  {
    start_time = std::chrono::high_resolution_clock::now();
  }
};

class ScopeTime : private StopWatch
{
private:
  const char *file, *function;
  const int line;
public:
  inline ScopeTime(const char *file, const char *function, const int line) :
    StopWatch(), file(file), function(function), line(line)
  {
  }

  inline ~ScopeTime()
  {
    ROS_INFO_STREAM(FG_GREEN<<file<<FG_BLUE<<"("<<line<<")"<<FG_MAGENTA<<"["<<function<<"]"<<FG_WHITE<<this->getTime()<< " ms.");
  }
};

#ifndef MEASURE_TIME
#define MEASURE_TIME rs::ScopeTime scopeTime(OUT_FILENAME, __FUNCTION__, __LINE__)
#endif

}  // end namespace

#endif
