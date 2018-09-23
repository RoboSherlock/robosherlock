/** \file trace.hpp .
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

   \brief A Trace class

-------------------------------------------------------------------------- */

#ifndef __UIMA_TRACE_HPP
#define __UIMA_TRACE_HPP

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#include <uima/types.h>
#include <uima/text.h>
#include <uima/comp_ids.h>

// Copy enough from cos .h files to keep compiler happy.

#ifndef UIMA_TRACE_ORIGIN
#define UIMA_TRACE_ORIGIN ""
#endif

/* ----------------------------------------------------------------------- */
/*       Types                                                             */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace util {

    /** the following trace levels of details can be distinguished */
    typedef enum EnTraceDetail {
      enTraceDetailOff      = 0,
      enTraceDetailLow      = 1,
      enTraceDetailMedium   = 2,
      enTraceDetailHigh     = 3
                              /* NOTE: all EnTraceDetail values must be < 4 (0x4) !!! */
    } EnTraceDetail;

    /* ----------------------------------------------------------------------- */
    /*       Classes                                                           */
    /* ----------------------------------------------------------------------- */

    /**
     * The class <tt> util::Trace</tt> is used to send information about function
     * execution and data (trace events) to a trace buffer.
     * Information will be sent only if the trace is enabled.
     *
     * \note An object of class util::Trace must be instantiated with
     * UIMA_TRACE_ORIGIN as an argument!
     * \code
       foo(unsigned int uiAValue, const char * cpszBValue)
       {
          util::Trace              clTrace(util::EnTraceDetailLow, UIMA_TRACE_ORIGIN, COMPONENT_FOO);

          // your code here ...

          clTrace.dump(EnTraceDetailHigh, "AValue [before]", uiAValue);
          clTrace.dump(EnTraceDetailHigh, "BValue [before]", cpszBValue);

          // your code here ...

          clTrace.dump("AValue", uiAValue);
          clTrace.dump("BValue", cpszBValue);

          // your code here ...

       }
       \endcode
     */
    class Trace {
    public:
      /** @name Constructors */
      /*@{*/
      /** create a trace with the specified level of details and an optional
          component id.
          \note Do not provide arguments for <tt> cpszFile</tt> directly, use the
          macro <tt> UIMA_TRACE_ORIGIN</tt> instead. */

      Trace(EnTraceDetail enDetail,
            const char * cpszOrigin,
            uima::TyComponentId utCompId = UIMA_TRACE_COMPONENT_ID_UNDEFINED);
      /*@}*/
      /** @name Properties */
      /*@{*/
      /** return TRUE if the trace is enabled */
      bool                       isEnabled(void) const;
      /*@}*/
      /** @name Dump Methods */
      /*@{*/
      /** dump data into the trace using the detail level set as a default by constructor.
          Several different data types are available: */
      void                       dump(const TCHAR * cpszMessage);
      void                       dump(const TCHAR * cpszMessage, const TCHAR * cpszValue);
#ifdef UNICODE
      void                       dump(const TCHAR * cpszMessage, const char * cpszValue);
#endif
      void                       dump(const TCHAR * cpszMessage, const void * cpvValue, size_t uiSize);
      void                       dump(const TCHAR * cpszMessage, bool bValue);
      void                       dump(const TCHAR * cpszMessage, char cValue);
      void                       dump(const TCHAR * cpszMessage, unsigned char ucValue);
      void                       dump(const TCHAR * cpszMessage, short sValue);
      void                       dump(const TCHAR * cpszMessage, unsigned short usValue);
      void                       dump(const TCHAR * cpszMessage, int iValue);
      void                       dump(const TCHAR * cpszMessage, unsigned int uiValue);
      void                       dump(const TCHAR * cpszMessage, long lValue);
      void                       dump(const TCHAR * cpszMessage, unsigned long ulValue);
//    void                       dump(const TCHAR * cpszMessage, INT64 lValue);
//    void                       dump(const TCHAR * cpszMessage, WORD64 ulValue);
      void                       dump(const TCHAR * cpszMessage, float fValue);
      void                       dump(const TCHAR * cpszMessage, double dValue);
      void                       dump(const TCHAR * cpszMessage, long double ldValue);
      /** dump the address into the trace using the detail level set as a default
          by constructor. */
      void                       dumpAdrs(const TCHAR * cpszMessage, const void * cpvValue);
      /** dump the specified data as UNICODE data into the trace using the detail
          level set as a default by constructor. */
      void                       dumpUnicode(const char * cpszMessage, const void * cpvValue, size_t uiLength);
      /** dump boolean data into the trace using a specific detail level. */
      void                       dumpBool(const TCHAR * cpszMessage, bool bValue);
      /** dump data into the trace using a specific detail level.
          Several different data types are available: */
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, const TCHAR * cpszValue);
#ifdef UNICODE
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, const char * cpszValue);
#endif
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, const void * cpvValue, size_t uiSize);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, bool bValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, char cValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, unsigned char ucValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, short sValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, unsigned short usValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, int iValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, unsigned int uiValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, long lValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, unsigned long ulValue);
//    void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, INT64 lValue);
//    void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, WORD64 ulValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, float fValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, double dValue);
      void                       dump(EnTraceDetail enDetail, const TCHAR * cpszMessage, long double ldValue);
      /** dump the address into the trace using the specified detail level */
      void                       dumpAdrs(EnTraceDetail enDetail, const TCHAR * cpszMessage, const void * cpvValue);
      /** dump the specified data as UNICODE data into the trace using the
          specified detail level */
      void                       dumpUnicode(EnTraceDetail enDetail, const char * cpszMessage, const void * cpvValue, size_t uiLength);
      /*@}*/
      /** @name Miscelleanous */
      /*@{*/
      /** flush the trace entries so that they are written to the trace file */
      void                       flush(void);
      /*@}*/
    protected:
      /* --- functions --- */
    private:
      /* --- functions --- */
      /* BASE CONSTRUCTOR NOT SUPPORTED */
      Trace(void); //lint !e1704
      /* COPY CONSTRUCTOR NOT SUPPORTED */
      Trace(const Trace & ); //lint !e1704
      /* ASSIGNMENT OPERATOR NOT SUPPORTED */
      Trace & operator=(const Trace & crclObject);
    }
    ; /* Trace */

    /* ----------------------------------------------------------------------- */
    /*       Globals                                                           */
    /* ----------------------------------------------------------------------- */

    /* ----------------------------------------------------------------------- */
    /*       Function declarations                                             */
    /* ----------------------------------------------------------------------- */

    /* ----------------------------------------------------------------------- */
    /*       Macro definitions                                                 */
    /* ----------------------------------------------------------------------- */

    /* ----------------------------------------------------------------------- */
    /*       Public implementation                                             */
    /* ----------------------------------------------------------------------- */

    inline Trace::Trace(EnTraceDetail, const char *, uima::TyComponentId) {
      ;
    }

    inline bool Trace::isEnabled(void) const {
      return(false);
    }

    inline void Trace::dump(const TCHAR *) {}

    inline void Trace::dump(const TCHAR *, const TCHAR *) {
      return;
    }

    inline void Trace::dump(const TCHAR *, long) {
      return;
    }

    inline void Trace::dump(const TCHAR *, unsigned long) {
      return;
    }

    inline void Trace::dump(const TCHAR *, int) {
      return;
    }

    inline void Trace::dump(const TCHAR *, unsigned int) {
      return;
    }

    inline void Trace::dumpAdrs(const TCHAR *, const void *) {
      return;
    }

    inline void Trace::dump(EnTraceDetail, const TCHAR *) {
      return;
    }
    inline void Trace::dump(EnTraceDetail, const TCHAR *, unsigned short) {
      return;
    }
    inline void Trace::dumpAdrs(EnTraceDetail, const TCHAR *, const void *) {
      return;
    }

  }  // namespace util
}  // namespace uima

#endif /* __UIMA_TRACE_HPP */

/* <EOF> */
