/** \file dirwalk.hpp .
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

   \brief a classs to iterate the entries of a directory in the file system

-------------------------------------------------------------------------- */

#ifndef __UIMA_DIRWALK_HPP
#define __UIMA_DIRWALK_HPP

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

#include "apr_file_info.h"
#include "apr_fnmatch.h"

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
     * The class <tt> DirectoryWalk</tt> is used to iterate the entries of a
     * directory in the file system.
     * \code
       foo(const Location & crclLocation, bool bRecurseSubdirs)
       {
          DirectoryWalk  clDirWalk(crclLocation);

          while(clDirWalk.isValid())
             {
             if(clDirWalk.isDirectory())
                cout << "Directory entry: " << clDirWalk.getNameWithoutPath();
             else
                if(clDirWalk.isFile())
                   cout << "File: " << clDirWalk.getNameWithoutPath();
         else
                   cout << "Weird? " << clDirWalk.getNameWithoutPath();
             if(clDirWalk.matchesWildcardPattern("*.cpp"))
                cout << " C++ source file;";
             cout << endl
             clDirWalk.setToNext();
             }
       }
       \endcode
     */
    class DirectoryWalk {
    public:
      /** @name Constructors */
      /*@{*/
      /** create a new instance of a directory walker based on a directory */
      DirectoryWalk(const char* crclDirectory);
      /*@}*/
      ~DirectoryWalk(void);
      /** @name Properties */
      /*@{*/
      /** return TRUE if the current entry is a valid directory entry */
      bool                       isValid(void) const                     {
        return haveEntry;
      }
      /** return TRUE if the current entry represents a regular file entry */
      bool                       isFile(void) const;
      /** return TRUE if the current entry represents a directory entry */
      bool                       isDirectory(void) const;
      /** return the name part of the filename of the current entry */
      const char *               getNameWithoutPath(void) const;
      /** return TRUE if the current entry matches the specified wildcard pattern */
      bool                       matchesWildcardPattern(const char * cpszPattern) const;
      /*@}*/
      /** @name Miscellaneous */
      /*@{*/
      /** walk to the next directory entry and return TRUE if there is one */
      bool                       setToNext(void);
      /*@}*/
    protected:
      /* --- functions --- */
    private:
      apr_pool_t               * dirPool;
      apr_dir_t                * aprDir;
      apr_finfo_t                aprInfo;
      bool                       haveEntry;

      /* --- functions --- */
      /* BASE CONSTRUCTOR NOT SUPPORTED */
      DirectoryWalk(void); //lint !e1704
      /* COPY CONSTRUCTOR NOT SUPPORTED */
      DirectoryWalk(const DirectoryWalk & ); //lint !e1704
      /* ASSIGNMENT OPERATOR NOT SUPPORTED */
      DirectoryWalk & operator=(const DirectoryWalk & crclObject);
    }
    ;                                                 /* DirectoryWalk */

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

    inline DirectoryWalk::DirectoryWalk(const char* crclDirectory)
    /* ----------------------------------------------------------------------- */
    {
      apr_status_t rv;

      dirPool = NULL;
      aprDir = NULL;
      haveEntry = FALSE;

      rv = apr_pool_create( &dirPool,NULL );
      if ( rv == APR_SUCCESS ) {
        rv = apr_dir_open( &aprDir,crclDirectory,dirPool );
        // If open succeeds get name of first file
        if ( rv == APR_SUCCESS ) {
          haveEntry = TRUE;
          setToNext();
        }
      } else {
        UIMA_EXC_THROW_NEW(ExcOutOfMemory,
                           UIMA_ERR_ENGINE_OUT_OF_MEMORY,
                           UIMA_MSG_ID_EXC_OUT_OF_MEMORY,
                           ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_POOL_FOR_CLASS,"uima::util::DirectoryWalk"),
                           ErrorInfo::unrecoverable);
      }
    }

    inline DirectoryWalk::~DirectoryWalk(void)
    /* ----------------------------------------------------------------------- */
    {
      if ( aprDir != NULL ) {
        apr_dir_close( aprDir );
        aprDir = NULL;
      }
      if ( dirPool != NULL )
        apr_pool_destroy ( dirPool );
    }

    inline bool DirectoryWalk::setToNext(void)
    /* ----------------------------------------------------------------------- */
    {
      apr_status_t rv;

      if ( haveEntry ) {
        rv = apr_dir_read( &aprInfo,APR_FINFO_TYPE|APR_FINFO_NAME,aprDir );
        if ( APR_STATUS_IS_ENOENT(rv) )
          haveEntry = FALSE;
      }
      return haveEntry;
    }

    inline bool DirectoryWalk::isFile(void) const
    /* ----------------------------------------------------------------------- */
    {
      return ( haveEntry && aprInfo.filetype==APR_REG );
    }

    inline bool DirectoryWalk::isDirectory(void) const
    /* ----------------------------------------------------------------------- */
    {
      return ( haveEntry && aprInfo.filetype==APR_DIR );
    }

    inline const char * DirectoryWalk::getNameWithoutPath(void) const
    /* ----------------------------------------------------------------------- */
    {
      return ( haveEntry ? aprInfo.name : NULL );
    }

// APR does not fill fname field so cannot easily return name with path

    inline bool DirectoryWalk::matchesWildcardPattern(const char * cpszPattern) const
    /* ----------------------------------------------------------------------- */
    {
      return ( haveEntry && (apr_fnmatch( cpszPattern,aprInfo.name,0 ) == APR_SUCCESS) );
    }

  }  // namespace util
}  // namespace uima

#endif /* __UIMA_DIRWALK_HPP */

/* <EOF> */
