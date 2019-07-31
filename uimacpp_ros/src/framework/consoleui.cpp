/** \file consoleui.cpp .
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

  Options used to be of the form "-opt value" or "-switch"
  APR follows standard convention of short and long options, e.g.
        "-c -x"   "-cx"   "--opt value"   "--opt=value"   "--switch"

-------------------------------------------------------------------------- */

#ifndef __UIMA_CONSOLEUI_CPP
#define __UIMA_CONSOLEUI_CPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <vector>
#include <uima/pragmas.hpp> //must be first include to surpress warnings
#include <uima/consoleui.hpp>
#include "apr_strings.h"
#include "apr_version.h"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {
  namespace util {

    /* ----------------------------------------------------------------------- */
    /*       Implementation                                                    */
    /* ----------------------------------------------------------------------- */

// -----------------------------------------------------------------------
//  Conctructor & Destructor
// -----------------------------------------------------------------------
    ConsoleUI::ConsoleUI(int argc, char * argv[],
                         const char * cpszTitle, const char * cpszCopyright)
        : iv_bQuiet(false),
        iv_cpszHelp(NULL),
        iv_szProcessName(""),
        iv_currentArg(99999) {
      // Catch any APR errors here in case caller does not expect this constructor to fail
      try {

        apr_status_t rv = apr_initialize();
        if (rv != APR_SUCCESS) {
          char errBuf[256];
          apr_strerror(rv, errBuf, sizeof(errBuf));
          UIMA_EXC_THROW_NEW(AprFailureException,
                             UIMA_ERR_APR_FAILURE,
                             ErrorMessage(UIMA_MSG_ID_EXC_APR_ERROR,errBuf),
                             ErrorMessage(UIMA_MSG_ID_EXCON_APR_FUNCTION,"apr_initialize"),
                             ErrorInfo::unrecoverable);
        }

        rv = apr_pool_create(&consPool, NULL);
        if (rv != APR_SUCCESS) {
          UIMA_EXC_THROW_NEW(ExcOutOfMemory,
                             UIMA_ERR_ENGINE_OUT_OF_MEMORY,
                             UIMA_MSG_ID_EXC_OUT_OF_MEMORY,
                             ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_POOL_FOR_CLASS,"uima::util::ConsoleUI"),
                             ErrorInfo::unrecoverable);
        }

        iv_cpszUsage[0] = iv_cpszUsage[1] = "";
        iv_argc = argc;
        iv_argv = (const char**) argv;
        // Get program name from first arg
        if ( cpszTitle != NULL ) {
          util::Filename progname(argv[0]);
          iv_szProcessName = (char *)apr_palloc(consPool, 1+strlen(progname.getName()));
          progname.extractBaseName(iv_szProcessName);
          cout << endl << iv_szProcessName << " - " << cpszTitle;
          if ( cpszCopyright != NULL )
            cout << "   " << cpszCopyright;
          cout << "  [APR version " << apr_version_string() << "]" << endl;
        }

        // Log and rethrow and APR failures
      } catch (Exception & rclException) {
        cerr << rclException;
        UIMA_EXC_RETHROW(rclException,
                         ErrorMessage(UIMA_MSG_ID_EXCON_CONSTRUCTING_CLASS,"uima::util::ConsoleUI"));
      }
    }

    /* ----------------------------------------------------------------------- */
    ConsoleUI::~ConsoleUI(void) {
      if (consPool != NULL)
        apr_pool_destroy(consPool);
      apr_terminate();
    }

// ------------------------------------------------------------------------
// Save the help string and check that all options are legal
// ------------------------------------------------------------------------

    void ConsoleUI::handleUsageHelp(const char * cpszUsage, const char * cpszHelp, const char * cpszHelpFlags) {
      std::vector<const char *>     opts;        // Vector of possible option (start points)
      unsigned int                  i;
      apr_status_t                  rv;
      unsigned int                  nHelpFlags;

      if (cpszHelpFlags == NULL)
        cpszHelpFlags = "[--help | -?]\t\n";
      iv_cpszUsage[0] = cpszHelpFlags;
      iv_cpszUsage[1] = cpszUsage;
      iv_cpszHelp     = cpszHelp;

      // Step thru the usage msg to and create a vector addressing the start of each option
      // Options can be switches or take a value, and be marked as optional (most options are
      // are optional [!] but taftest_app requires one and only one of --config & --configs)

      //      --opt      --opt <val>      --opt=<val>
      //     [--opt]    [--opt <val>]    [--opt=<val>]  [-c] [-f foo] [-x | -y]

      // Assume a long option starts with -- preceeded by [ or whitespace
      // Assume a short option starts with - preceeded by [ or space and is followed by ] or space

      // First process help flags then the remainder of the usage message.
      for ( i = 0; i < 2; ++i ) {
        const char * str = iv_cpszUsage[i];
        if ( *str == '-' && *++str == '-' )           // Check for initial -- where str[-1] would be bad!
          opts.push_back(str);
        while ((str = strchr(str+1, '-')) != NULL) {
          if ( ( str[1]=='-' && (str[-1]=='[' || isspace(str[-1])) ) ||
               ( str[1]!='-' && isgraph(str[1]) && (str[-1]=='[' || str[-1]==' ') && (str[2]==']' || str[2]==' ') ) )
            opts.push_back(++str);
        }
        if (i == 0)
          nHelpFlags = opts.size();
      }

      iv_pOpts = (apr_getopt_option_t *)apr_palloc( consPool, (opts.size()+1) * sizeof(apr_getopt_option_t) );

      // Process short and long options in two passes as must use a wierd value for optch to
      // locate long options, so can have no more than 47 long options (1 - '0')

      int j = 0;
      const char * opt, * str;
      int k = 0;
      char helpChars[20];

      for ( i = 0; i < opts.size(); ++i ) {
        opt = opts[i];
        if ( *opt == '-' ) {                        // Long option
          str = strpbrk( ++opt, "]= \n" );
          if (str == NULL)                          // Must be last option!
            str = opt + strlen(opt);
          iv_pOpts[j].name = apr_pstrndup(consPool, opt, str - opt);
          iv_pOpts[j].has_arg = (*str == '=' || (*str == ' ' && *(str+1) == '<'));
          iv_pOpts[j].optch = j+1;                  // Use 1-based index as "short" option
          if (i < nHelpFlags)
            helpChars[k++] = j+1;
          ++j;
        }
      }
      iv_maxOptIndex = j;                           // Largest wierd value marking a long option
      // NOTE: should check that this is < any alphanumeric char, i.e. < 48 ('0')

      for ( i = 0; i < opts.size(); ++i ) {
        opt = opts[i];
        if ( *opt != '-' ) {                        // Short option
          iv_pOpts[j].optch = *opt;                 // Use real character
          iv_pOpts[j].name = NULL;
          iv_pOpts[j++].has_arg = (opt[1] == ' ' && opt[2] != '|' );
          if (i < nHelpFlags)
            helpChars[k++] = *opt;
        }
      }
      iv_pOpts[j].optch = 0;                        // Mark end of list
      helpChars[k] = '\0';

#ifndef NDEBUG
      // Dump all legal options when called with just 1 arg of "--"
      if ( iv_argc == 2 && strcmp(iv_argv[1],"--")==0 )
        debugDisplayOptions(j);
#endif

      // Set up processing of interleaved options and arguments
      apr_getopt_init(&iv_hGetopt, consPool, iv_argc, iv_argv);
      iv_hGetopt->interleave = 1;

      // Can't have more options than args!
      iv_pFoundOpts = (found_opt_t *)apr_palloc(consPool, iv_argc * sizeof(found_opt_t));
      iv_nFoundOpts = 0;

      // Check that provided options are valid and save them.
      int          index;
      const char * optarg;
      bool         needHelp = false;

      while ((rv = apr_getopt_long(iv_hGetopt, iv_pOpts, &index, &optarg)) == APR_SUCCESS) {
        iv_pFoundOpts[iv_nFoundOpts].index = index;
        iv_pFoundOpts[iv_nFoundOpts++].value = optarg;
        if (strchr(helpChars,index) != NULL)
          needHelp = true;
      }

      // Two errors are:  Invalid option & missing argument
      // getopt prints these on error streamwith fprintf(stderr but can override
      // via iv_hGetopt->errfn        typedef void( apr_getopt_err_fn_t)(void *arg, const char *err,...)

      if ( rv != APR_EOF )
        displayUsage();

      // Args are valid ... now check for the help flags
      else
        if (needHelp)
          displayHelp();
    }

// ------------------------------------------------------------------------
// Search for a specified option and return its value
// ------------------------------------------------------------------------

    bool ConsoleUI::hasOption(const char * cpszArgument, const char *& cpszrValue) const {
      for ( int i = 0; i < iv_nFoundOpts; ++i ) {
        int index = iv_pFoundOpts[i].index;               // Index if a long option or a single char
        if ( index > iv_maxOptIndex ) {
          if ( cpszArgument[0] == index && cpszArgument[1] == '\0' ) {
            cpszrValue = iv_pFoundOpts[i].value;
            return true;
          }
        } else
          if ( strcmp(cpszArgument, iv_pOpts[index-1].name) == 0 ) {
            cpszrValue = iv_pFoundOpts[i].value;
            return true;
          }
      }
      return false;
    }

// ------------------------------------------------------------------------
//  Routine to help debug problems with parsing Usage msg
// ------------------------------------------------------------------------

#ifndef NDEBUG
    void ConsoleUI::debugDisplayOptions(int numOpts) {
      int i;
      const char * valMsg[] = { "\n", " <value>\n"
                              };

      cout << "DEBUG: Found " << numOpts << " options in Help message" << endl;
      for ( i = 0; i < iv_maxOptIndex; ++i )
        cout << "  --" << iv_pOpts[i].name << valMsg[iv_pOpts[i].has_arg];
      for ( i = iv_maxOptIndex; i < numOpts; ++i )
        cout << "   -" << (char)iv_pOpts[i].optch << valMsg[iv_pOpts[i].has_arg];
    }
#endif

  }  // namespace util
}  // namespace uima

#endif /* __UIMA_CONSOLEUI_CPP */

/* <EOF> */
