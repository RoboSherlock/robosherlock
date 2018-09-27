/** \file envvar.hpp .
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

   \brief a class to access (but not modify) environment variables

-------------------------------------------------------------------------- */

#ifndef __UIMA_ENVVAR_HPP
#define __UIMA_ENVVAR_HPP

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

#include "apr_env.h"

#include <uima/exceptions.hpp>
#include <uima/msg.h>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace util {

    /**
     * The class <tt> EnvironmentVariableQueryOnly</tt> is used to manage
     * environment variables which can be queried only.
     * The value for this variable cannot be changed.
     * \code
       foo(void)
       {
          EnvironmentVariableQueryOnly var("TMP");

          if(var.hasValue())
             cout << "TMP directory: " << var.getValue() << endl;
          else
             cout << "No TMP directory defined" << endl;
       }
      \endcode
     */
    class EnvironmentVariableQueryOnly {
    public:
      /** @name Constructors */
      /*@{*/
      /** instantiate a new environment variable handler */
      EnvironmentVariableQueryOnly(const char * cpszVar);
 
      ~EnvironmentVariableQueryOnly(void);
      /*@}*/
      /* --- selectors --- */
      /** return TRUE if the environment variable has a value else FALSE */
      bool                       hasValue(void) const                         {
        return (envValue != NULL);
      }
      /** return the value for the environment variable or NULL if it does not have a value */
      const char *               getValue(void) const                         {
        return envValue;
      }
      /** return TRUE if the value for the environment variable
          is "ON" or "YES" or "TRUE" or "1" */
      bool                       hasValueEnabled(void) const;
      /** return TRUE if the value for the environment variable */
      /* --- operators --- */
      /* --- functors --- */
    protected:
      /* --- functions --- */
    private:
      apr_pool_t               * envPool;
      char                     * envValue;

      /* BASE CONSTRUCTOR NOT SUPPORTED */
      EnvironmentVariableQueryOnly(void);
      /* COPY CONSTRUCTOR NOT SUPPORTED */
      EnvironmentVariableQueryOnly(const EnvironmentVariableQueryOnly & crclVar);
      /* ASSIGNMENT OPERATOR NOT SUPPORTED */
      EnvironmentVariableQueryOnly & operator=(const EnvironmentVariableQueryOnly & crclObject);

    }
    ; /* EnvironmentVariableQueryOnly */

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
    /*       Implementation                                                    */
    /* ----------------------------------------------------------------------- */

    inline EnvironmentVariableQueryOnly::EnvironmentVariableQueryOnly(const char * cpszVar)
    /* ----------------------------------------------------------------------- */
    {
      envPool = NULL;
      envValue = NULL;

      if ( apr_pool_create( &envPool,NULL ) != APR_SUCCESS ) {
        UIMA_EXC_THROW_NEW(ExcOutOfMemory,
                           UIMA_ERR_ENGINE_OUT_OF_MEMORY,
                           UIMA_MSG_ID_EXC_OUT_OF_MEMORY,
                           ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_POOL_FOR_CLASS,
                                        "uima::util::EnvironmentVariableQueryOnly"),
                           ErrorInfo::unrecoverable);
      }
      apr_env_get( &envValue,cpszVar,envPool );
    }

    inline EnvironmentVariableQueryOnly::~EnvironmentVariableQueryOnly(void)
    /* ----------------------------------------------------------------------- */
    {
      if ( envPool != NULL )
        apr_pool_destroy ( envPool );
    }

    inline bool EnvironmentVariableQueryOnly::hasValueEnabled(void) const
    /* ----------------------------------------------------------------------- */
    {
      return ( envValue != NULL &&
               ( strcasecmp(envValue,"ON")==0 ||
                 strcasecmp(envValue,"YES")==0 ||
                 strcasecmp(envValue,"TRUE")==0 ||
                 strcmp(envValue,"1")==0
               ) );
    }

  }  // namespace util
}  // namespace uima

#endif /* __UIMA_ENVVAR_HPP */

/* <EOF> */
