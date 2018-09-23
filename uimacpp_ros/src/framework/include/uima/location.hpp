/** \file location.hpp .
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

   \brief A filename class

-------------------------------------------------------------------------- */

#ifndef __UIMA_LOCATION_HPP
#define __UIMA_LOCATION_HPP

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

#include "apr_file_info.h"
#include "apr_strings.h"
#include "apr_lib.h"
#include "apr_file_io.h"
#include <uima/envvar.hpp>

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
     * The class <tt> util::Location</tt> holds a path specification for a filesystem
     * \code
       ???
      \endcode
     */

    class Location {
    public:
      /** @name Constructors */
      /*@{*/
      /** create a location based on a path given as a C string */
      Location(const char * path);

      /** create a location based on the TMP directory (or current) */
      Location(void);

      /** destructor */
      ~Location(void);
      /*@}*/
      /** @name Assignment operations */
      /*@{*/
      /** assign new complete filename */
      Location &                 operator = (const Location & location);

      /** @name Properties */
      /*@{*/
      /** determine whether a path exists on file system */
      bool                       isExistent(void) const;

      /** determine whether path has an absolute path specification */
      bool                       isAbsolute(void) const;


      /** create a directory */
      void                       makeDirectory(void) const;

      /** @name Parts */
      /*@{*/
      /** return full pathename as a C string pointer */
      const char *               getAsCString(void) const                     {
        return path;
      }

      operator const char* (void) const            {
        return path;
      }
      /*@}*/
    protected:
      /* --- functions --- */
    private:
      void                       createPool(void);

      apr_pool_t               * locPool;
      char                     * path;

    }
    ; /* Location */

    /* ----------------------------------------------------------------------- */
    /*       Implementation                                                    */
    /* ----------------------------------------------------------------------- */

    /* ----------------------------------------------------------------------- */
    inline Location::Location(const char* pathname) {
      createPool();

      // Blindly append a Unix directory separator ... Windows doesn't appear to care
      // and can be cleaned up with util::Filename::normalizeAbsolute()
      path = apr_pstrcat(locPool, pathname, "/", NULL );
    }

    /* ----------------------------------------------------------------------- */
    inline Location::Location(void) {
      createPool();

      // Get temp directory from env var or use current directory.
      EnvironmentVariableQueryOnly envVar("TMP");
      if (envVar.hasValue())
        path = apr_pstrcat(locPool, envVar.getValue(), "/", NULL );
      else
        path = (char*)"./";
    }

    /* ----------------------------------------------------------------------- */
    inline Location::~Location(void) {
      if (locPool != NULL)
        apr_pool_destroy(locPool);
    }

    /* ----------------------------------------------------------------------- */
    inline void Location::createPool(void) {
      if (apr_pool_create(&locPool, NULL) != APR_SUCCESS) {
        UIMA_EXC_THROW_NEW(ExcOutOfMemory,
                           UIMA_ERR_ENGINE_OUT_OF_MEMORY,
                           UIMA_MSG_ID_EXC_OUT_OF_MEMORY,
                           ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_POOL_FOR_CLASS,"uima::util::Location"),
                           ErrorInfo::unrecoverable);
      }
    }

    /* ----------------------------------------------------------------------- */
    inline Location & Location::operator = (const Location & location) {
      // Must make a copy since RHS & its pool may disappear
      path = apr_pstrdup(locPool, location.getAsCString());
      return *this;
    }

    /* ----------------------------------------------------------------------- */
    inline bool Location::isExistent(void) const {
      apr_finfo_t  finfo;
      apr_status_t rv;

      // Check if can determine type (file, directory, pipe ...)
      rv = apr_stat(&finfo, path, APR_FINFO_TYPE, locPool);
      return rv == APR_SUCCESS && finfo.filetype == APR_DIR;
    }


    /* ----------------------------------------------------------------------- */
    inline void Location::makeDirectory(void) const {
      if (!isExistent()) {
        apr_status_t rv;
        rv = apr_dir_make(path, APR_OS_DEFAULT , locPool);
        if (rv != APR_SUCCESS) {
          std::cout << "Error creating dir " << path << std::endl;
          //TODO throw exception
          UIMA_EXC_THROW_NEW(Exception,
                             UIMA_ERR_APR_FAILURE ,
                             UIMA_MSG_ID_EXC_APR_ERROR,
                             ErrorMessage(UIMA_MSG_ID_EXC_APR_ERROR,"uima::util::Location::makeDirectory"),
                             ErrorInfo::unrecoverable);
        }
      }

    }



    /* ----------------------------------------------------------------------- */
    inline bool Location::isAbsolute(void) const {
      apr_status_t rv;
      const char * root;
      const char * pth = path;

      // Modifies arg pth ... returns success if finds the root (?:\ on Windows)
      rv = apr_filepath_root(&root, &pth, 0, locPool);
      return rv == APR_SUCCESS;
    }

  }  // namespace util
}  // namespace uima

#endif /* __UIMA_LOCATION_HPP */

/* <EOF> */
