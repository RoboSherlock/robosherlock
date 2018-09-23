/** \file dllfile.hpp .
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

   \brief the class DllProcLoaderFile is used to maintain dynamic link libraries

-------------------------------------------------------------------------- */

#ifndef __UIMA_DLLFILE_HPP
#define __UIMA_DLLFILE_HPP

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation dependencies                                       */
/* ----------------------------------------------------------------------- */

#include "apr_pools.h"
#include "apr_dso.h"
#include <uima/filename.hpp>

#include <uima/exceptions.hpp>
#include <uima/msg.h>
#include <uima/envvar.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define DSOBUFLEN 256

/* ----------------------------------------------------------------------- */
/*       Types                                                             */
/* ----------------------------------------------------------------------- */

namespace uima {

  typedef apr_dso_handle_sym_t        TyProcedure;

}

/* ----------------------------------------------------------------------- */
/*       Classes                                                           */
/* ----------------------------------------------------------------------- */
namespace uima {

  namespace util {

    /**
     * The class <tt> DllProcLoaderFile</tt> is used to load specific procedures
     * from a DLL (shared library) by their name.
     * \code
       typedef void (*MYPROC)(const char *);

       foo(const uima::util::Filename & crclFilename)
       {
          uima::util::DllProcLoaderFile clDll(crclFilename);

          if(clDll.isValid())
             {
             MYPROC utProc = (MYPROC) clDll.getProcedure("myFunction");

             if(utProc)
                (utProc)("My message");   // call my procedure in DLL
             else
                // error handling
             }
       }
       \endcode
     * The loaded functions may be called as long as the DLL is loaded. Calling
     * a loaded function after the destructor of <tt> DllProcLoaderFile</tt> has
     * unloaded the DLL will cause unexpected results. (An application crash
     * is very likely to happen.)
     *
     * <b> NOTE: dropped the Cos Special Requirements for AIX 4.2 and below </b>
     *
     */
    class DllProcLoaderFile {
    public:
      /** @name Constructors */
      /*@{*/
      /** instantiate a DLL object and load the specified DLL */
      DllProcLoaderFile(const Filename & crclFilename);
      /*@}*/
      ~DllProcLoaderFile(void);
      /** @name Properties */
      /*@{*/
      /** return TRUE if the DLL was loaded successfully */
      bool                       isValid(void) const                    {
        return isLoaded;
      }
      /** return a pointer to the specified DLL function or NULL if the specified
          function cannot be found in this DLL */
      TyProcedure                getProcedure(const char * cpszProcName);
      /** map APR error code (status) of last call to a UIMACPP error code. */
      TyErrorId               getErrorId(void) const; 
      /** map APR error code to a UIMACPP message id */
      TyMessageId  getErrorMsgId(void) const;
        
      /** return a pointer to the error msg (if any) */
      const char *               getErrorMsg(void) const;
      /** return the file's filename */
      const Filename &           getFilename(void) const                {
        return(iv_clFilename);
      }
      /** determine whether a file exists for filename on file system */
      bool                       isExistent(void) const                 {
        return iv_clFilename.isExistent();
      }

      /*@}*/
    protected:
      /* --- functions --- */
    private:
      apr_dso_handle_t         * dsoHandle;
      apr_pool_t               * dsoPool;
      bool         isLoaded;
      apr_status_t               aprError;
      Filename                   iv_clFilename;
      char *                     dsoBuffer;
#ifdef WIN32
      apr_status_t               aprError2;
#endif
      /* --- functions --- */
      /* BASE CONSTRUCTOR NOT SUPPORTED */
      DllProcLoaderFile(void); //lint !e1704
      /* COPY CONSTRUCTOR NOT SUPPORTED */
      DllProcLoaderFile(const DllProcLoaderFile & ); //lint !e1704
      /* ASSIGNMENT OPERATOR NOT SUPPORTED */
      DllProcLoaderFile & operator=(const DllProcLoaderFile & crclObject);
    }
    ;                                                 /* DllProcLoaderFile */

    /* ----------------------------------------------------------------------- */
    /*       Globals                                                           */
    /* ----------------------------------------------------------------------- */

    /* ----------------------------------------------------------------------- */
    /*       Function declarations                                             */
    /* ----------------------------------------------------------------------- */

    /* ----------------------------------------------------------------------- */
    /*       Macro definitions                                                 */
    /* ----------------------------------------------------------------------- */

#ifdef _MSC_VER
#  define DSO_EXTN ".dll"
#else
#  ifdef __APPLE__
#    define DSO_EXTN ".dylib"
#  else
#    define DSO_EXTN ".so"
#  endif
#endif

    /* ----------------------------------------------------------------------- */
    /*       Implementation                                                    */
    /* ----------------------------------------------------------------------- */

    inline DllProcLoaderFile::DllProcLoaderFile(const Filename & crclFilename)
        : iv_clFilename(crclFilename)
        /* ----------------------------------------------------------------------- */
    {
      isLoaded = FALSE;
      dsoHandle = NULL;
      dsoPool = NULL;

      aprError = apr_pool_create(&dsoPool, NULL);
      if ( aprError == APR_SUCCESS ) {
        // Always allocate error buffer as if done in getErrorMsg it loses it's const
        // and that is expected back in ResourceManager::requestAnnotatorFile
        dsoBuffer = (char*) apr_palloc ( dsoPool, DSOBUFLEN );

#ifdef _DEBUG
#ifdef _WINDOWS
        // Add "D" to library name
        char libName[128];
        strcpy(libName, iv_clFilename.getName());
        strcat(libName, "D");
        iv_clFilename.setNewName(libName);
#endif
#endif
        // Add the appropriate extension if missing
        if (! iv_clFilename.isAbsolute() && *iv_clFilename.getExtension() == '\0' )
          iv_clFilename.setNewExtension(DSO_EXTN);
        aprError = apr_dso_load(&dsoHandle, iv_clFilename.getAsCString(), dsoPool);
        isLoaded = ( aprError == APR_SUCCESS );

#ifdef WIN32
        // WIN32: LoadLibraryEx falsely reports ERROR_MOD_NOT_FOUND when the dll cannot be loaded.
        //   APR should use DONT_RESOLVE_DLL_REFERENCES to check if ERROR_INVALID_DLL is a better rc.
        //   Can check for missing case but not for damaged vs. missing library.   
		aprError2 = APR_SUCCESS;
        if ( !isLoaded ) {
          if ( APR_TO_OS_ERROR(aprError) == ERROR_MOD_NOT_FOUND ) {
            if ( ! iv_clFilename.isExistent() ) {                           // Not in cur dir
              util::EnvironmentVariableQueryOnly clEnvVarSearchPaths("PATH");
			  if ( !iv_clFilename.determinePath( clEnvVarSearchPaths.getValue() ) ) {
                return;                                                     // Definitely missing
			  }
			}
            // Exists, so must be damaged ... change error code
            aprError = APR_FROM_OS_ERROR(ERROR_INVALID_DLL);
            aprError2 = aprError;
          }
        }
#endif
      } else {
        UIMA_EXC_THROW_NEW(ExcOutOfMemory,
                           UIMA_ERR_ENGINE_OUT_OF_MEMORY,
                           UIMA_MSG_ID_EXC_OUT_OF_MEMORY,
                           ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_POOL_FOR_CLASS,
                                        "uima::util::DllProcLoaderFile"),
                           ErrorInfo::unrecoverable);
      }

    }

    inline DllProcLoaderFile::~DllProcLoaderFile(void)
    /* ----------------------------------------------------------------------- */
    {
      if ( dsoHandle != NULL ) {
        apr_dso_unload ( dsoHandle );
        dsoHandle = NULL;
      }
      if ( dsoPool != NULL )
        apr_pool_destroy ( dsoPool );
      isLoaded = FALSE;
    }

    inline TyProcedure DllProcLoaderFile::getProcedure(const char * cpszProcName)
    /* ----------------------------------------------------------------------- */
    {
      apr_dso_handle_sym_t procAddr = NULL;
      if (isLoaded) {
        aprError = apr_dso_sym(&procAddr, dsoHandle, cpszProcName);
        if (aprError == APR_SUCCESS)
          return (TyProcedure) procAddr;
      }
      return NULL;
    }

    inline const char * DllProcLoaderFile::getErrorMsg(void) const
    /* ----------------------------------------------------------------------- */
    {
      if ( dsoHandle == NULL )
        return NULL;
      else {
#ifdef WIN32
        if ( aprError == aprError2 )
          return apr_strerror ( aprError2, dsoBuffer, DSOBUFLEN );
#endif
        return apr_dso_error ( dsoHandle, dsoBuffer, DSOBUFLEN );
      }

    }

	inline TyErrorId  DllProcLoaderFile::getErrorId(void) const
    /* ----------------------------------------------------------------------- */
    {	
	  if (aprError == APR_SUCCESS) {
		  return UIMA_ERR_NONE;
	  }
#ifdef WIN32
	  if (APR_TO_OS_ERROR(aprError) == ERROR_MOD_NOT_FOUND) {
	    return UIMA_ERR_ANNOTATOR_COULD_NOT_FIND;
	  } 
#else
	  if (APR_TO_OS_ERROR(aprError) == ENOENT) {
		  return UIMA_ERR_ANNOTATOR_COULD_NOT_FIND;
	  }
#endif
	  return UIMA_ERR_ANNOTATOR_COULD_NOT_LOAD;   
    }

	inline TyMessageId  DllProcLoaderFile::getErrorMsgId(void) const
    /* ----------------------------------------------------------------------- */
    {	
	  if (aprError == APR_SUCCESS) {
		  return UIMA_ERR_NONE;
	  }
#ifdef WIN32
	  if (APR_TO_OS_ERROR(aprError) == ERROR_MOD_NOT_FOUND) {
	    return UIMA_MSG_ID_ANNOTATOR_COULD_NOT_FIND;
	  } 
#endif
	  return UIMA_MSG_ID_ANNOTATOR_COULD_NOT_LOAD;   
    }

  }   // namespace util
}   // namespace uima

#endif /* __UIMA_DLLFILE_HPP */

/* <EOF> */
