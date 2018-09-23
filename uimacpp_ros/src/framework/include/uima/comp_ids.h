/** \file comp_ids.h .
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

    \brief  Component Ids for UIMACPP trace facility

-------------------------------------------------------------------------- */

#ifndef UIMA_COMP_IDS_H
#define UIMA_COMP_IDS_H

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/types.h>

// first arg: a util::Trace object, second arg: a "stream expression" like: "message: " << us
#include <sstream>
#define UIMA_TRACE_STREAM(trace, x) \
     if (trace.isEnabled()) {   \
        std::stringstream _s;    \
        _s << x; _s.flush();    \
        trace.dump( _s.str().c_str(), "" ); \
     }

#define UIMA_TRACE_STREAM_ARG(trace, x, y) \
     if (trace.isEnabled()) {   \
        std::stringstream _s1;    \
        _s1 << x; _s1.flush(); \
        std::stringstream _s2;    \
        _s2 << y; _s2.flush(); \
        trace.dump( _s1.str().c_str(), _s2.str().c_str() ); \
     }

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/** @name Trace Component Ids
   user-defined component ids must be in the range
   UIMA_TRACE_COMPONENT_ID_FIRST .. UIMA_TRACE_COMPONENT_ID_LAST */
/*@{*/
/** \internal Internal Use Only! do not use! */
#define UIMA_TRACE_COMPONENT_ID_UNDEFINED   ((uima::TyComponentId) 0x00)
/** first trace component id available to user */
#define UIMA_TRACE_COMPONENT_ID_FIRST       ((uima::TyComponentId) 0x01)
/** last trace component id available to user */
#define UIMA_TRACE_COMPONENT_ID_LAST        ((uima::TyComponentId) 0xFF)
/*@}*/

/* NOTE: UIMA_TRACE_COMPONENT_ID_FIRST is equal to 1 */
#define UIMA_TRACE_COMPID_EXCEPTIONS       ((uima::TyComponentId) (  0 + UIMA_TRACE_COMPONENT_ID_FIRST))
#define UIMA_TRACE_COMPID_ENGINE           ((uima::TyComponentId) (  1 + UIMA_TRACE_COMPONENT_ID_FIRST))
#define UIMA_TRACE_COMPID_RESOURCE_MGR     ((uima::TyComponentId) (  2 + UIMA_TRACE_COMPONENT_ID_FIRST))
#define UIMA_TRACE_COMPID_ANNOTATOR_MGR       ((uima::TyComponentId) (  3 + UIMA_TRACE_COMPONENT_ID_FIRST))
#define UIMA_TRACE_COMPID_LOG_FACILITY     ((uima::TyComponentId) ( 4 + UIMA_TRACE_COMPONENT_ID_FIRST))
#define UIMA_TRACE_COMPID_TIMING           ((uima::TyComponentId) ( 5 + UIMA_TRACE_COMPONENT_ID_FIRST))

/* UIMACPP provided Annotators should use reserved area 151..199 */
#define UIMA_TRACE_COMPID_ANNOTATOR_DEFAULT   ((uima::TyComponentId) (150 + UIMA_TRACE_COMPONENT_ID_FIRST))

/* User Annotators should use reserved area 201..255 */
/* Note: comp ids for user annotator must be user configurable through configuration */
#define UIMA_TRACE_COMPID_USER_FIRST       ((uima::TyComponentId) (200 + UIMA_TRACE_COMPONENT_ID_FIRST))
#define UIMA_TRACE_COMPID_USER_LAST        ((uima::TyComponentId) (254 + UIMA_TRACE_COMPONENT_ID_FIRST))

#endif /* UIMA_COMP_IDS_H */

/* <EOF> */

