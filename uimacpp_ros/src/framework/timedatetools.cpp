/*
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
*/

#include <uima/timedatetools.hpp>
#include <uima/strtools.hpp>
#include <uima/strconvert.hpp>
#include <uima/text.h>
using namespace std;
namespace uima {


  const size_t cuiSecDigits_After_Period = 3;
  const size_t cuiPctDigits_After_Period = 1;


  string Timer::getDescription(size_t padWidth) const {
    if (iv_strDecscription.length() == 0) {
      return "";
    }
    return (strpad_r_copy(iv_strDecscription, padWidth, ' ') + ": ");  //lint !e1024: No function has same argument count as 'strpad_r_copy(const std::basic_string<char,std::char_traits<char>,std::allocator<char>>, unsigned int, char)', 2 candidates found
  }

  string Timer::timeString(double dTime) {
    string s;
    if (dTime <= FLT_MIN) {
      s = "0";
    } else {
      double2String(dTime, cuiSecDigits_After_Period, s);
    }
    return strpad_l_copy(s, (size_t)6, ' ') + " seconds";
  }

  string Timer::timeString() const {
    return getDescription() + timeString(getAccumulatedTime());
  }

  string
  Timer::timeAndPercentString(double dRelativeToSeconds) const {
    string strAccumulatedTime("0");
    double dRelativeTime = 0.0;
    if (getAccumulatedTime() > FLT_MIN) {
      double2String(getAccumulatedTime(), cuiSecDigits_After_Period, strAccumulatedTime);
      dRelativeTime = (getAccumulatedTime() / dRelativeToSeconds);
    }
    string strRelativePct;
    double2String(100.0 * dRelativeTime, cuiPctDigits_After_Period , strRelativePct);

    return (getDescription() +
            strpad_l_copy(strAccumulatedTime, (size_t)6) +
            _TEXT(" seconds (") +
            strpad_l_copy(strRelativePct, (size_t)cuiPctDigits_After_Period+3) +
            _TEXT("%)"));
  }

  string
  Timer::timeAndPercentAndThroughputString(
    double            dRelativeToSeconds,
    size_t            uiRelativeToItems,
    const string &   strItemsname
  ) const {
    string s;
    string s2;
    return getDescription() +
           strpad_l_copy(double2String(getAccumulatedTime(), cuiSecDigits_After_Period, s), (size_t)6) +
           _TEXT(" seconds (") +
           strpad_l_copy(double2String(100.0 * (getAccumulatedTime() / dRelativeToSeconds), cuiPctDigits_After_Period , s), (size_t)cuiPctDigits_After_Period+3) +
           _TEXT("% ") +
           double2String(uiRelativeToItems / getAccumulatedTime(), cuiPctDigits_After_Period, s2)  +
           _TEXT(" ") + (strItemsname.length() == 0 ? (string)_TEXT("items") : strItemsname) +
           _TEXT("/sec)");
  }

  string
  Timer::timeAndPercentString(const Timer& crclRelativeToTimer) const {
    return timeAndPercentString(crclRelativeToTimer.getAccumulatedTime());
  }

  string
  Timer::relativeThroughputString(size_t uiRelativeToItems, const string & strItemsname) const {
    string s;
    return getDescription() +
           double2String(uiRelativeToItems / getAccumulatedTime(), cuiSecDigits_After_Period, s)  +
           " " + (strItemsname.length() == 0 ? (string)_TEXT("items") : strItemsname) + _TEXT("/second");
  }

  string
  Timer::percentString(double dRelativeToSeconds) const {
    string s;
    return getDescription() +
           double2String(100.0 * (getAccumulatedTime() / dRelativeToSeconds), cuiPctDigits_After_Period, s)  +
           _TEXT("%");
  }

  string
  Timer::percentString(const Timer& crclRelativeToTimer) const {
    return percentString(crclRelativeToTimer.getAccumulatedTime());
  }

} //namespace uima

