/** \file consoleui.hpp .
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

   \brief A Console User Interface class

-------------------------------------------------------------------------- */
#ifndef __UIMA_CONSOLEUI_HPP
#define __UIMA_CONSOLEUI_HPP

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

#include <uima/types.h>   // Has UIMA_LINK_IMPORTSPEC macro
#include <iostream>
#include "apr_pools.h"
#include "apr_getopt.h"
#include <uima/filename.hpp>

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace util {

    typedef struct found_opt_t {
      int            index;     // Index of arg in array of possible options (1-based)
      const char   * value;     // Value of arg or NULL
    }
    found_opt_t;
  }
}

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define BEEP                     '\a'          /* beep character */

/* Filler betwen formatted field and value .. always print last two characters
  field...............................: value
*/
static const char * gs_cpszFiller = "....................................: ";
static int  gi_maxlen = strlen(gs_cpszFiller) - 2;

static const char * gs_cpszLine = "--------------------------------------------------------------------------\n";

static const char * gs_progressChars[4] = { "\r | ",
    "\r / ",
    "\r - ",
    "\r \\ "
                                          };

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace util {

    /**
     * The class <tt> util::ConsoleUI</tt> displays msgs on the "console"
     * \code
       ???
      \endcode
     */

    class UIMA_LINK_IMPORTSPEC ConsoleUI {
    public:
      /** @name Constructors */
      /*@{*/
      /** Instantiate a console user interface object.
         <tt> argc</tt> and <tt> argv</tt> must be passed to the <tt> ConsoleUI</tt> object. */
      ConsoleUI(int argc, char * argv[],
                const char * cpszTitle = 0, const char * cpszCopyright = 0);

      /** destructor */
      ~ConsoleUI(void);
      /*@}*/

	  /*@{*/
      void                       setQuietMode(bool bQuiet)                 {
        iv_bQuiet = bQuiet;
      }

      std::ostream &                  getOutputStream(void) const               {
        return std::cout;
      }

      /** display a field and a value with padding so value starts at col 40
          First column is <tt> field</tt>, second column is <tt> value</tt> */
      void                       format(const char * cpszField, const char * cpszValue) const;
      void                       format(const char * cpszField, long lValue) const;
      void                       format(const char * cpszField, unsigned long ulValue) const;
      void                       formatBool(const char * cpszField, bool bValue) const;
      void                       format(const char * cpszField, bool bValue) const {
        formatBool(cpszField, bValue);
      }
      void                       format(const char * cpszField, int iValue) const {
        format(cpszField, (long)iValue);
      }
//   void                       format(const char * cpszField, size_t szValue) const
//                                                                   { format(cpszField, (unsigned long)szValue); }

      /** display a header */
      void                       formatHeader(const char * cpszMsg) const;

      /** display a header */
      void                       header(const char * cpszMsg) const;

      /** display a line of dashes */
      void                       horizontalBar(void) const;

      /** display a new-line */
      void                       newline(void) const;

      /** display an informational msg */
      void                       info(const char * cpszMsg) const;

      /** display a warning msg on stderr */
      void                       warning(const char * cpszMsg1, const char * cpszMsg2 = NULL) const;

      /** display an error msg on stderr */
      void                       error(const char * cpszMsg) const;

      /** display an error msg on stderr and exit */
      void                       fatal(int iErrcode, const char * cpszMsg1, const char * cpszMsg2 = NULL) const;

      /*@}*/

      /** @name Usage and Help methods */

      /** process usage msg and check command-line arguments */
      void                       handleUsageHelp(const char * cpszUsage, const char * cpszHelp,
          const char * cpszHelpFlags = NULL);

      /** display usage or help and then exit! */
      void                       displayUsage(void) const;
      void                       displayHelp(void) const;
      /*@}*/

      /** @name Process optional arguments */
      bool                       hasArgString(const char * cpszArgument, const char *& cpszrValue) const;
      bool                       hasArgNumval(const char * cpszArgument, long & rlValue) const;
      bool                       hasArgSwitch(const char * cpszArgument) const;
      /*@}*/

      /** @name Process positional arguments */

      /** set cursor to the first command line argument and return TRUE if there is one */
      bool                       setToFirst(void);
      /** set cursor to the next command line argument and return TRUE if there is one */
      bool                       setToNext(void);
      /** return TRUE if the current command line argument is valid */
      bool                       isValid(void) const;

      /** return current command line argument as C string pointer */
      operator const char* (void) const            {
        return(getAsCString());
      }
      /** return current command line argument as C string pointer */
      const char *               getAsCString(void) const;
      /*@}*/

      /** @name Display progress indication */
      void                       progressStart(void)                          {
        iv_uiProgressInd = 0;
      }
      void                       progressStep(void);
      void                       progressStop(void) const;
      /*@}*/


    protected:
      /* --- functions --- */
      void                       format(const char * cpszMag) const;
      bool                       hasOption(const char * cpszArgument, const char *& cpszrValue) const;
      void                       debugDisplayOptions(int numOpts);

    private:
      apr_pool_t               * consPool;
      int                        iv_argc;
      const char              ** iv_argv;
      const char               * iv_cpszHelp;
      const char               * iv_cpszUsage[2];
      char                     * iv_szProcessName;
      bool                       iv_bQuiet;
      apr_getopt_option_t      * iv_pOpts;                 // Array of possible options
      int                        iv_maxOptIndex;
      apr_getopt_t             * iv_hGetopt;               // Handle for apr_getopt...
      found_opt_t              * iv_pFoundOpts;
      int                        iv_nFoundOpts;
      int                        iv_currentArg;
      int                        iv_uiProgressInd;

    }
    ; /* ConsoleUI */

    /* ----------------------------------------------------------------------- */
    /*       Implementation                                                    */
    /* ----------------------------------------------------------------------- */

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::formatHeader(const char * cpszMsg) const {
      if (iv_bQuiet) return;
      std::cout << std::endl << "  " << cpszMsg << "\n  " << gs_cpszLine+2;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::header(const char * cpszMsg) const {
      if (iv_bQuiet) return;
      std::cout << std::endl << "  " << gs_cpszLine+2 << iv_szProcessName << ": " << cpszMsg << std::endl;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::format(const char * cpszMsg) const {
      if (iv_bQuiet) return;
      int len = strlen(cpszMsg);
      if ( len > gi_maxlen ) len = gi_maxlen;
      std::cout << "  " << cpszMsg << gs_cpszFiller+len;      // Pad out to col 40
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::format(const char * cpszField, const char * cpszValue) const {
      if (iv_bQuiet) return;
      format(cpszField);
      std::cout << cpszValue << std::endl;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::format(const char * cpszField, long lValue) const {
      if (iv_bQuiet) return;
      format(cpszField);
      std::cout << lValue << std::endl;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::format(const char * cpszField, unsigned long ulValue) const {
      if (iv_bQuiet) return;
      format(cpszField);
      std::cout << ulValue << std::endl;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::formatBool(const char * cpszField, bool bValue) const {
      if (iv_bQuiet) return;
      format(cpszField);
      std::cout << (bValue ? "YES" : "NO") << std::endl;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::horizontalBar(void) const {
      if (iv_bQuiet) return;
      std::cout << gs_cpszLine << std::endl;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::newline(void) const {
      if (iv_bQuiet) return;
      std::cout << std::endl;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::displayUsage(void) const {
      std::cout << std::endl << "Usage: " << iv_szProcessName << std::endl
      << iv_cpszUsage[0] << iv_cpszUsage[1] << BEEP;
      ::exit(1);
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::displayHelp(void) const {
      std::cout << std::endl << "Help: " << iv_cpszHelp;
      ::exit(1);
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::info(const char * cpszMsg) const {
      if (iv_bQuiet) return;
      std::cout << std::endl << gs_cpszLine << iv_szProcessName << ": " << cpszMsg << std::endl;

    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::warning(const char * cpszMsg1, const char * cpszMsg2) const {
      std::cerr << "WARNING: " << cpszMsg1;
      if ( cpszMsg2 != NULL )
	std::cerr << cpszMsg2;
      std::cerr << std::endl;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::error(const char * cpszMsg) const {
      std::cerr << "ERROR: " << cpszMsg << std::endl;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::fatal(int iErrcode, const char * cpszMsg1 , const char * cpszMsg2) const {
      std::cerr << "FATAL ERROR: " << iErrcode << " " << cpszMsg1;
      if ( cpszMsg2 != NULL )
	std::cerr << cpszMsg2;
      std::cerr << std::endl;
      ::exit((int) iErrcode);
    }

    /* ----------------------------------------------------------------------- */
    inline bool ConsoleUI::hasArgString(const char * cpszArgument, const char *& cpszrValue) const {
      // Check if have this option and it does have a value
      return hasOption( cpszArgument,cpszrValue ) && (cpszrValue != NULL);
    }

    /* ----------------------------------------------------------------------- */
    inline bool ConsoleUI::hasArgNumval(const char * cpszArgument, long & rlValue) const {
      // Check if have this option and that it has a value
      const char* value;
      if ( !hasOption( cpszArgument,value ) || (value == NULL) )
        return false;
      if (*value == '\'')
        ++value;
      if ( isdigit(*value) || (*value == '-') ) {
        rlValue = atol(value);
        return true;
      }
      // Type mismatch .. should display warning?
      return false;
    }

    /* ----------------------------------------------------------------------- */
    inline bool ConsoleUI::hasArgSwitch(const char * cpszArgument) const {
      // Check if have this option and that it doesn't have a value
      const char* value;
      return hasOption( cpszArgument,value ) && (value == NULL);
    }

    /* ----------------------------------------------------------------------- */
    inline bool ConsoleUI::setToFirst(void) {
      iv_currentArg = iv_hGetopt->skip_start;
      return iv_currentArg < iv_hGetopt->skip_end;
    }

    /* ----------------------------------------------------------------------- */
    inline bool ConsoleUI::setToNext(void) {
      return ++iv_currentArg < iv_hGetopt->skip_end;
    }

    /* ----------------------------------------------------------------------- */
    inline bool ConsoleUI::isValid(void) const {
      return iv_currentArg < iv_hGetopt->skip_end;
    }

    /* ----------------------------------------------------------------------- */
    inline const char * ConsoleUI::getAsCString(void) const {
      if (iv_currentArg < iv_hGetopt->skip_end)
        return iv_hGetopt->argv[iv_currentArg];
      else
        return NULL;
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::progressStep(void) {
      if (iv_bQuiet) return;
      std::cout << gs_progressChars[iv_uiProgressInd++ % 4];
    }

    /* ----------------------------------------------------------------------- */
    inline void ConsoleUI::progressStop(void) const {
      if (iv_bQuiet) return;
      std::cout << "\r                                                                                \r";
    }


  }  // namespace util
}  // namespace uima

#endif /* __UIMA_CONSOLEUI_HPP */

/* <EOF> */
