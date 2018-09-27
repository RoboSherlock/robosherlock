/** @name conui.cpp
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

-----------------------------------------------------------------------------

      Contents: functions to let UIMA types interact with util::ConsoleUI

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include "uima/conui.hpp"

#include "uima/consoleui.hpp"
#include "uima/engine.hpp"
#include "uima/api.hpp"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {


  void uimaToolDisplayException(util::ConsoleUI & rclConsole, const Exception & crclException)
  /* ----------------------------------------------------------------------- */
  {
    rclConsole.formatHeader(_TEXT("Exception"));
    rclConsole.format(_TEXT("Exception error id"), crclException.getErrorInfo().getErrorId());
    rclConsole.format(_TEXT("Exception name"), crclException.getName());
    rclConsole.format(_TEXT("Exception what"), crclException.what());
    rclConsole.format(_TEXT("Exception message"), crclException.getErrorInfo().getMessage().asString().c_str());
    rclConsole.formatBool(_TEXT("Exception recoverable"), crclException.getErrorInfo().isRecoverable());
    const TCHAR * cpszSavePrefix = ErrorInfo::getGlobalErrorInfoIndent();
    ErrorInfo::setGlobalErrorInfoIndent("  ");
    rclConsole.getOutputStream() << crclException.getErrorInfo() << endl;
    ErrorInfo::setGlobalErrorInfoIndent(cpszSavePrefix);
  }

  void uimaToolDisplayErrorId(util::ConsoleUI const & rclConsole, const TyErrorId utErrorId, const TCHAR * cpszLastErrorMsg)
  /* ----------------------------------------------------------------------- */
  {
    rclConsole.formatHeader(_TEXT("TyErrorId"));
    rclConsole.format(_TEXT("Error id"), utErrorId);
    rclConsole.format(_TEXT("Error id"), TextAnalysisEngine::getErrorIdAsCString(utErrorId));
    if (EXISTS(cpszLastErrorMsg)) {
      rclConsole.format(_TEXT("Last logged error"), cpszLastErrorMsg);
    }
  }

  void uimaToolHandleErrorId(util::ConsoleUI & rclConsole, const TyErrorId utErrorId, const TCHAR * cpszLastErrorMsg, const TCHAR * cpszFunction, TyErrorId utErrorIdExpected)
  /* ----------------------------------------------------------------------- */
  {
    if (EXISTS(cpszFunction)) {
      rclConsole.format(_TEXT("Function"), cpszFunction);
    }
    if (utErrorIdExpected != UIMA_ERR_NONE) {
      rclConsole.format(_TEXT("Expecting Error id"), utErrorIdExpected);
      rclConsole.format(_TEXT("Expecting Error id"), TextAnalysisEngine::getErrorIdAsCString(utErrorIdExpected));
    }
    if (utErrorId != utErrorIdExpected) {
      uimaToolDisplayErrorId(rclConsole, utErrorId, cpszLastErrorMsg);
      UIMA_EXC_THROW_NEW(ConsoleAbortExc,
                         utErrorId,
                         UIMA_MSG_ID_EXC_UNEXPECTED_ERROR,
                         ErrorMessage(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT),
                         ErrorInfo::unrecoverable);
    }
    if (EXISTS(cpszFunction)) {
      rclConsole.formatBool(_TEXT("   Function OK"), true);
    }
  }

} // namespace uima

/* <EOF> */

