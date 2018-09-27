#ifndef UIMA_TIMEDATETOOLS_HPP_
#define UIMA_TIMEDATETOOLS_HPP_
/** \file timedatetools.hpp .
-------------------------------------------------------------------------------



 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.

-------------------------------------------------------------------------------

    \brief  Contains Timer and timing tool functions/macros

-------------------------------------------------------------------------------
*/

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <time.h>
#include <limits.h>
#include <float.h>
#include <string>
#include <uima/types.h>
#include <uima/strtools.hpp>


namespace uima {

#ifdef DEBUG_TIMING
#  ifdef UIMA_TIMING
#     undef UIMA_TIMING
#  endif
  /** A macro in which our timing statements are hidden,
     if DEBUG_TIMING is <EM>not</EM> defined.
     Using a timer <TT>clTimer</TT> of type <TT>Timer</TT> the macro can be used
     in statements like:
     <TT>UIMA_TIMING(clTimer.start)</TT> and <TT>UIMA_TIMING(clTimer.start)</TT>
  */
#  define UIMA_TIMING( expr ) ( expr )
#else
#  ifdef UIMA_TIMING
#     undef UIMA_TIMING
#  endif
#  define UIMA_TIMING( ignore ) ( ( void )0 )
#endif

  /**
   * Function <TT>currTimeString</TT> is a help function to return the current
   * time/data as a string.
   * Note: This should be made more flexible in the output format
   *
   * @return a string with the current time/date
   *
   */
  inline std::string
  tafCurrTimeDateString() {
    time_t ltime;
    (void)time(&ltime);   //get time, ignore return value
    return asctime(localtime(&ltime)); //time string
  }


  /**
   * Class <TT>ClTimer</TT> is a tool class to help with timing.
   * It includes some useful string output for timers.
   *
   */
//class UIMA_LINK_IMPORTSPEC Timer : public CosClTiming
  class UIMA_LINK_IMPORTSPEC Timer {
  public:
    ///Constructor
//   Timer(const std::string & clstrDescription = ""):
//      CosClTiming(),
//      iv_strDecscription(clstrDescription)
//      {}
    Timer(const std::string & clstrDescription = "") {}

    void start(void) {}
    void stop(void) {}
    void reset(void) {}

    /** Retrieve the accumulated time
       To avoid division by zero errors we never return zero
       but FLT_MIN instead.
     */
    double getAccumulatedTime(void) const {
//         return ((CosClTiming*)this)->getAccumulatedTime() > 0 ?
//                  ((CosClTiming*)this)->getAccumulatedTime() :
//                  FLT_MIN;
      return 0;
    }

    ///Returns timed time up to now
    double getTimeSoFar() const {
      // Not implemented
      return((double)0.0);
    }

    /** @name Formated string output for timer data */
    /*@{*/
    ///Returns the description from the constructor (used by follwing functions)
    std::string getDescription(size_t padWidth = 35) const;
    ///Returns the description from the constructor (used by follwing functions)
    void setDescription(const std::string & crstrDesc) {
      iv_strDecscription = crstrDesc;
    }
    ///Returns &lt;DESCRIPTION>:      &lt;accumulatedTime> seconds
    std::string timeString() const;
    ///Returns &lt;DESCRIPTION>:      &lt;items/accumulatedTime> &lt;itemname> / second
    std::string relativeThroughputString(size_t items, const std::string & itemsname = "") const;
    ///Returns &lt;DESCRIPTION>:      &lt;accumulatedTime/relative_to_seconds>%
    std::string percentString(double relative_to_seconds) const;
    ///Returns &lt;DESCRIPTION>:      &lt;accumulatedTime/relative_to_seconds>%
    std::string percentString(const Timer& relative_to_timer) const;
    ///Returns absolute (timeString) and relative (percentString) as one string
    std::string timeAndPercentString(double relative_to_seconds) const;
    ///Returns absolute (timeString) and relative (percentString) as one string
    std::string timeAndPercentString(const Timer& relative_to_timer) const;
    ///Returns combination of <TT>timeAndPercentString</TT> plus <TT>relativeThroughputString</TT>
    std::string timeAndPercentAndThroughputString(double relative_to_seconds, size_t relative_to_items, const std::string & itemsname = "") const;
    static std::string timeString(double dTime);
    /*@}*/
    /** @name Operators to combine timers */
    /*@{*/
    /// Addition of timers
    Timer
    operator+(const Timer & crclOther) const {
      Timer clTmp(iv_strDecscription);
      if (crclOther.iv_strDecscription.length() > 0) {
        clTmp.setDescription(iv_strDecscription + " + " + crclOther.iv_strDecscription);
      }
//      clTmp.iv_dAccumulatedTime = iv_dAccumulatedTime + crclOther.iv_dAccumulatedTime;
      return clTmp;
    }
    /// Subtraction of timers
    Timer
    operator-(const Timer & crclOther) const {
      Timer clTmp(iv_strDecscription);
      if (crclOther.iv_strDecscription.length() > 0) {
        clTmp.setDescription(iv_strDecscription + " - " + crclOther.iv_strDecscription);
      }
//      if (iv_dAccumulatedTime > crclOther.iv_dAccumulatedTime) {
//         clTmp.iv_dAccumulatedTime = iv_dAccumulatedTime - crclOther.iv_dAccumulatedTime;
//      }
      return clTmp;
    }
    /*@}*/
  private:
    std::string iv_strDecscription;
  }
  ; //lint !e1754: Expected symbol 'operator-=' to be declared for class 'Timer' !e1754: Expected symbol 'operator-=' to be declared for class 'Timer'

} // namespace uima

#endif //UIMA_TIMEDATETOOLS_HPP_
/* <EOF> */

