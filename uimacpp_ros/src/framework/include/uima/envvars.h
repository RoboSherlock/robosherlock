/** \file envvars.h .
-----------------------------------------------------------------------------



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

    \brief  All UIMACPP Environment Variables

-------------------------------------------------------------------------- */

#ifndef UIMA_ENVVARS_H
#define UIMA_ENVVARS_H

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */


/** The UIMACPP environment variable determines the location of the XSD files  */
#define UIMA_ENVVAR_HOME               ("UIMACPP_HOME")
/** The UIMACPP environment variable determines the location of the log file
  * If a UIMACPP annotator is called from JAVA, we switch to using the Java
  * logger.
  */
#define UIMA_ENVVAR_LOG_FILE     ("UIMACPP_LOGFILE")

/** The UIMACPP environment variable determines the location of the data files
 *  This is used to resolve relative paths.
 */
#define UIMA_ENVVAR_SUFFIX_DATAPATH             ("UIMACPP_DATAPATH")

/** @deprecated
 * The UIMACPP environment variable determines where the temporary working
 * files are created
 */
#define UIMA_ENVVAR_SUFFIX_WORKPATH    ("UIMACPP_WORKPATH")

/** The UIMACPP environment variable to register URI scheme handlers
 *
 */
#define UIMA_ENVVAR_SOFA_STREAM_HANDLERS  ("UIMACPP_STREAMHANDLERS")

#endif /* UIMA_ENVVARS_H */

/* <EOF> */

