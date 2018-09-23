/** \file conui.hpp .

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

   \brief  functions to let UIMA types interact with util::ConsoleUI

-------------------------------------------------------------------------- */

#ifndef UIMA_CONUI_HPP
#define UIMA_CONUI_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include "uima/pragmas.hpp" // must be first file to be included to get pragmas
#include "uima/exceptions.hpp"
#include "uima/err_ids.h"
#include "uima/consoleui.hpp"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {

  /** display the specified UIMA exception on the console object */
  void uimaToolDisplayException(uima::util::ConsoleUI & rclConsole, const uima::Exception & crclException);

  /** display the specified UIMA error id on the console object */
  void uimaToolDisplayErrorId(uima::util::ConsoleUI const & rclConsole, const uima::TyErrorId utErrorId, const TCHAR * cpszLastErrorMsg);

  /** display the specified UIMA error id on the console object and
      call uima::util::ConsoleUI::fatal() if the error id is not UIMA_ERR_NONE */
  void uimaToolHandleErrorId(uima::util::ConsoleUI & rclConsole, const uima::TyErrorId utErrorId, const TCHAR * cpszLastErrorMsg, const TCHAR * cpszFunction, uima::TyErrorId utErrorIdExpected = 0);

  /* ----------------------------------------------------------------------- */
  /*       Types / Classes                                                   */
  /* ----------------------------------------------------------------------- */

  /* ----------------------------------------------------------------------- */
  /*       Implementation                                                    */
  /* ----------------------------------------------------------------------- */

  /* ----------------------------------------------------------------------- */

} //namespace uima

#endif /* UIMA_CONUI_HPP */

/* <EOF> */

