#ifndef RS_UTIL_TIME_H_
#define RS_UTIL_TIME_H_

#include <cmath>
#include <string>
#include <chrono>
#include <rs/utils/output.h>

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
    OUT_AUX_INT(FG_BLUE, NO_COLOR, OUT_LEVEL_DEBUG, OUT_STD_STREAM, this->getTime() << " ms.", file, line, function);
  }
};

#ifndef MEASURE_TIME
#define MEASURE_TIME rs::ScopeTime scopeTime(OUT_FILENAME, __FUNCTION__, __LINE__)
#endif

}  // end namespace

#endif
