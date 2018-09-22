/** \file filename.hpp .
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

#ifndef __UIMA_FILENAME_HPP
#define __UIMA_FILENAME_HPP

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

#include "apr_file_info.h"
#include "apr_strings.h"
#include "apr_lib.h"
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
     * The class <tt> FilenameCl</tt> is used to maintain filenames with all of its
     * (operating system specific) constituents: drive, path, base name and extension.
     * \code
       ???
      \endcode
     */

    class Filename {
    public:
      /** @name Constructors */
      /*@{*/
      /** create an empty filename */
      Filename(void);
      /** create a filename based on a filename given as a C string */
      Filename(const char * filename);

      /** create a filename based on a path, a filename and an optional extension,
          all given as C strings */
      Filename(const char * cpszPath,
               const char * cpszFilename,
               const char * cpszExtension = 0);
      /** copy constructor */
      Filename(const Filename & filename);
      /** destructor */
      ~Filename(void);
      /*@}*/
      /** @name Assignment operations */
      /*@{*/
      /** assign new complete filename */
      Filename &                 operator = (const Filename & filename);


      /** @name Properties */
      /*@{*/
      /** determine whether a file exists for filename on file system */
      bool                       isExistent(void) const;

      /** determine whether path has an absolute path specification */
      bool                       isAbsolute(void) const;

      /** return the size of this file in bytes.
          \note If the file does not exist a size of 0 bytes is returned. */
      unsigned long              getFileSize(void) const;

      /** @name Parts */
      /*@{*/
      /** return full filename as a C string pointer */
      const char *               getAsCString(void) const                     {
        return fname;
      }

      operator const char* (void) const            {
        return fname;
      }

      /** return the name part of this filename without the path but with the extension.
          \note If the filename part is empty, the function returns a pointer
          to an empty string, <em> not</em> a NULL pointer. */
      const char  *              getName(void) const;

      /** return the extension only, starting with the dot (e.g. ".so")
          \note If there is no extension, the function returns a pointer
          to an empty string, <em> not</em> a NULL pointer. */
      const char  *              getExtension(void) const;

      /** return the length of the complete filename */
      size_t                     getLength(void) const;

      /** assign a new entry from a path, optional filename and optional extension. */
      void                       setNew(const char * cpszPath,
                                        const char * cpszName = 0,
                                        const char * cpszExtension = 0);

      /** assign a new filename - keep current path.
          filename may include extension or not */
      void                       setNewName(const char * cpszName);

      /** assign a new extension - keep current path and filename.
          Specified extension must include the extension dot (".") */
      void                       setNewExtension(const char * cpszExtension);

      /** convert to an absolute name in native format with appropriate directory separators. */
      void                       normalizeAbsolute(void);

      /** convert to a name in native format with appropriate directory separators. */
      void                       normalize(void);

      /** copy path value to buffer pointed to by <tt> pszPath</tt>.
          If a path does not exist for this object, <tt> pszPath</tt> is set to the empty string.
          The path is returned with a terminating path separator character. */
      void                       extractPath(char * pszPath) const;

      /** copy base name (without path or extension) to buffer pointed to by <tt> pszBaseName</tt>. */
      void                        extractBaseName(char * pszBaseName) const;

      /** return TRUE if base names match (basic name part witout extension) */
      bool                       matchesBase(const Filename & crclFilename) const;

      /** search for file in a list of search paths and return TRUE if found */
      bool                       determinePath(const char* searchPaths);

      /*@}*/
    protected:
      /* --- functions --- */
    private:
      void                       createPool(void);

      apr_pool_t               * fnPool;
      char                     * fname;
    }
    ; /* Filename */

    /* ----------------------------------------------------------------------- */
    /*       Implementation                                                    */
    /* ----------------------------------------------------------------------- */

    /* ----------------------------------------------------------------------- */
    inline Filename::Filename(void) {
      createPool();
      fname = NULL;
    }

    /* ----------------------------------------------------------------------- */
    inline Filename::Filename(const char* filename) {
      createPool();
      fname = apr_pstrdup(fnPool, filename);
    }

    /* ----------------------------------------------------------------------- */
    inline Filename::Filename(const char * cpszPath, const char * cpszName,
                              const char * cpszExtension) {
      const char * fn;
      apr_status_t rv;

      createPool();
      if ( cpszExtension != NULL )
        fn = apr_pstrcat( fnPool,cpszName,cpszExtension,NULL );
      else
        fn = cpszName;
      // Add directory separator if necessary & normalize etc.
      rv = apr_filepath_merge(&fname, cpszPath, fn, APR_FILEPATH_NATIVE, fnPool);
      if (rv != APR_SUCCESS)
        fname = NULL;
    }

    /* ----------------------------------------------------------------------- */
    inline Filename::Filename(const Filename & filename) {
      createPool();
      fname = apr_pstrdup(fnPool, filename.getAsCString());
    }

    /* ----------------------------------------------------------------------- */
    inline Filename::~Filename(void) {
      if (fnPool != NULL)
        apr_pool_destroy(fnPool);
    }

    /* ----------------------------------------------------------------------- */
    inline void Filename::createPool(void) {
      if (apr_pool_create(&fnPool, NULL) != APR_SUCCESS) {
        fnPool = NULL;
        UIMA_EXC_THROW_NEW(ExcOutOfMemory,
                           UIMA_ERR_ENGINE_OUT_OF_MEMORY,
                           UIMA_MSG_ID_EXC_OUT_OF_MEMORY,
                           ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_POOL_FOR_CLASS,"uima::util::Filename"),
                           ErrorInfo::unrecoverable);
      }
    }

    /* ----------------------------------------------------------------------- */
    inline Filename & Filename::operator = (const Filename & filename) {
      // Must make a copy since RHS & its pool may disappear
      fname = apr_pstrdup(fnPool, filename.getAsCString());
      return *this;
    }

    /* ----------------------------------------------------------------------- */
    inline const char  * Filename::getName(void) const {
      if (fname == NULL)
        return NULL;
      else
        return apr_filepath_name_get(fname);
      // Probably returns a ptr to the end portion of fname so no memory allocated
    }

    /* ----------------------------------------------------------------------- */
    inline const char  * Filename::getExtension(void) const {
      if (fname == NULL)
        return NULL;
      else {
        const char* cpszExtn = strrchr(apr_filepath_name_get(fname), '.');
        return (cpszExtn!=NULL ? cpszExtn : "");
      }
    }

    /* ----------------------------------------------------------------------- */
    inline size_t Filename::getLength(void) const {
      if (fname == NULL)
        return 0;
      else
        return strlen(fname);
    }

    /* ----------------------------------------------------------------------- */
    inline void Filename::extractPath(char * pszPath) const {
      if (fname == NULL)
        *pszPath = '\0';
      else {
        // Get length of path up to the filename, copy with trailing separator and append a 0
        unsigned long len = apr_filepath_name_get(fname) - fname;
        strncpy(pszPath, fname, len);
        pszPath[len] = '\0';
      }
    }

    /* ----------------------------------------------------------------------- */
    inline void Filename::extractBaseName(char * pszBaseName) const {
      if (fname == NULL)
        *pszBaseName = '\0';
      else {
        // Get name after part and strip off extension (if any)
        const char* cpszName = apr_filepath_name_get(fname);
        const char* cpszExtn = strrchr(cpszName, '.');
        unsigned long len = cpszExtn==NULL ? strlen(cpszName) : cpszExtn - cpszName;
        strncpy(pszBaseName, cpszName, len);
        pszBaseName[len] = '\0';
      }
    }

    /* ----------------------------------------------------------------------- */
    inline bool Filename::isExistent(void) const {
      apr_finfo_t finfo;
      apr_status_t rv;

      // Check if can determine type (file, directory, pipe ...)
      rv = apr_stat(&finfo, fname, APR_FINFO_TYPE, fnPool);
      return !(APR_STATUS_IS_ENOENT(rv));
    }

    /* ----------------------------------------------------------------------- */
    inline bool Filename::isAbsolute(void) const {
      apr_status_t rv;
      const char * root;
      const char * fn = fname;

      // Modifies arg fn ... returns success if finds the root (?:\ on Windows)
      rv = apr_filepath_root(&root, &fn, 0, fnPool);
      return rv == APR_SUCCESS;
    }

    /* ----------------------------------------------------------------------- */
    inline bool Filename::matchesBase(const Filename & crclFilename) const {
      const char * cpszName1, * cpszName2, * cpszExtn1, * cpszExtn2;
      unsigned long len1, len2;

      if (fname == NULL)
        return FALSE;
      cpszName1 = apr_filepath_name_get(fname);
      cpszExtn1 = strrchr(cpszName1, '.');
      len1 = cpszExtn1==NULL ? strlen(cpszName1) : cpszExtn1 - cpszName1;

      cpszName2 = crclFilename.getName();
      if (cpszName2 == NULL)
        return FALSE;
      cpszExtn2 = strrchr(cpszName2, '.');
      len2 = cpszExtn2==NULL ? strlen(cpszName2) : cpszExtn2 - cpszName2;
      return (len1 == len2 && strncmp(cpszName1, cpszName2, len1) == 0);
    }

    /* ----------------------------------------------------------------------- */
    inline void Filename::setNew(const char * cpszPath, const char * cpszName,
                                 const char * cpszExtension) {
      // create new string from the 3 parts
      // The path must end in a separator & the extension must start with a '.'
      fname = apr_pstrcat( fnPool,cpszPath,cpszName,cpszExtension,NULL );
      return;
    }

    /* ----------------------------------------------------------------------- */
    inline void Filename::setNewName(const char * cpszName) {
      if (fname == NULL)
        return;
      // If new name is longer than old (or old had none) remove old
      // and create a new entry by concatenating new filename.
      char * pszName = (char*) apr_filepath_name_get(fname);
      if (strlen(pszName) >= strlen(cpszName))
        strcpy( pszName,cpszName );
      else {
        *pszName = '\0';
        fname = apr_pstrcat( fnPool,fname,cpszName,NULL );
      }
      return;
    }

    /* ----------------------------------------------------------------------- */
    inline void Filename::setNewExtension(const char * cpszExtension) {
      const char * cpszName;
      char * pszExtn;

      if (fname == NULL || cpszExtension == NULL)
        return;
      // If new extension not longer than old replace in place, otherwise
      // remove old and create a new entry by concatenating new extension.
      cpszName = apr_filepath_name_get(fname);
      pszExtn = CONST_CAST(char *, strrchr(cpszName, '.'));
      if ( pszExtn == NULL )
        fname = apr_pstrcat( fnPool,fname,cpszExtension,NULL );
      else {
        if (strlen(pszExtn) >= strlen(cpszExtension))
          strcpy(pszExtn, cpszExtension);
        else {
          *pszExtn = '\0';
          fname = apr_pstrcat( fnPool,fname,cpszExtension,NULL );
        }
      }
      return;
    }

    /* ----------------------------------------------------------------------- */
    inline unsigned long Filename::getFileSize(void) const {
      apr_finfo_t finfo;
      apr_status_t rv;

      rv = apr_stat(&finfo, fname, APR_FINFO_SIZE, fnPool);
      return rv == APR_SUCCESS ? finfo.size : 0;
    }

    /* ----------------------------------------------------------------------- */
    inline void Filename::normalizeAbsolute(void) {
      apr_status_t rv;
      char* newname;

      if (fname == NULL)
        return;
      rv = apr_filepath_merge(&newname, NULL, fname, APR_FILEPATH_NATIVE, fnPool);
      if (rv == APR_SUCCESS)
        fname = newname;
      return;
    }

    /* ----------------------------------------------------------------------- */
    inline void Filename::normalize(void) {
      apr_status_t rv;
      char* newname;

      if (fname == NULL)
        return;
      rv = apr_filepath_merge(&newname, ".", fname, APR_FILEPATH_NATIVE, fnPool);
      if (rv == APR_SUCCESS)
        fname = newname;
      return;
    }

    /* ----------------------------------------------------------------------- */
    inline bool Filename::determinePath(const char* searchPaths) {
      apr_array_header_t  * pathElts;
      char                * name;

      if (fname == NULL)
        return FALSE;
      // Drop any existing path
      name = (char*) apr_filepath_name_get(fname);
      apr_filepath_list_split(&pathElts, searchPaths, fnPool);              // Cannot fail!
      for ( int i = 0; i < pathElts->nelts; ++i ) {
        apr_filepath_merge(&fname, ((char**)pathElts->elts)[i], name, APR_FILEPATH_NATIVE, fnPool);
        if (isExistent())
          return TRUE;
      }
      return FALSE;
    }

  }  // namespace util
}  // namespace uima

#endif /* __UIMA_FILENAME_HPP */

/* <EOF> */
