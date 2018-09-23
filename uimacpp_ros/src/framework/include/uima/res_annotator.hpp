/** \file res_annotator.hpp .
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

    \brief  Contains ResourceAnnotatorFile

   Description:

-----------------------------------------------------------------------------


   9/7/1999  Initial creation

-------------------------------------------------------------------------- */

#ifndef UIMA_RES_ANNOTATOR_HPP
#define UIMA_RES_ANNOTATOR_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/res_abase.hpp>
///#include "uima/annotator_file.hpp"

#include <uima/language.hpp>
#include <uima/assertmsg.h>
#include <uima/dllfile.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  namespace internal {
    /**
     * The class <TT>ResourceAnnotatorFile</TT> is used to maintain a Annotator file resource.
     */
    class UIMA_LINK_IMPORTSPEC ResourceAnnotatorFile : public FileResource {
    public:
      /** @name Constructors */
      /*@{*/
      ResourceAnnotatorFile(icu::UnicodeString const & crKey,
                            icu::UnicodeString const & crKind);
      /*@}*/
      /** @name Properties */
      /*@{*/
      /** Return the pointer to the annotator file class. */
      /** internal::AnnotatorFile **/
      util::DllProcLoaderFile * getAnnotatorFile(void) const        {
        return(iv_pAnnotatorFile);
      }
    protected:
      /** @name Miscellaneous */
      /*@{*/
      /** Method will be called by resource manager for initialization. */
      virtual void            init(ErrorInfo &);
      /** Method will be called by resource manager for de-initialization. */
      virtual void            deInit(void);
      /*@}*/

    private:
      /** AnnotatorFile **/
      util::DllProcLoaderFile *            iv_pAnnotatorFile;
      /* --- functions --- */
      /* BASE CONSTRUCTOR NOT SUPPORTED */
      ResourceAnnotatorFile(void); //lint !e1704
      /* COPY CONSTRUCTOR NOT SUPPORTED */
      ResourceAnnotatorFile(const ResourceAnnotatorFile & ); //lint !e1704
      /* ASSIGNMENT OPERATOR NOT SUPPORTED */
      ResourceAnnotatorFile & operator=(const ResourceAnnotatorFile & crclObject);
    }
    ; /* ResourceAnnotatorFile */

    /**
     * The class <TT>ResourceAnnotatorFactory</TT>, is used to generate a new object
     * of type ResourceAnnotatorFile.
     * @see
     */
    class UIMA_LINK_IMPORTSPEC ResourceAnnotatorFileFactory : public ResourceFactoryABase {
    public:
      /** @name Constructors */
      /*@{*/
      ResourceAnnotatorFileFactory(void);
      /*@}*/
      /** @name Miscellaneous */
      /*@{*/
      /** Create a new resource object. */
      virtual ResourceABase * createResource(icu::UnicodeString const & crclKey) const;
      /*@}*/
    protected:
      /* --- functions --- */
    private:
      /* --- functions --- */
      /* COPY CONSTRUCTOR NOT SUPPORTED */
      ResourceAnnotatorFileFactory(const ResourceAnnotatorFileFactory & ); //lint !e1704
      /* ASSIGNMENT OPERATOR NOT SUPPORTED */
      ResourceAnnotatorFileFactory & operator=(const ResourceAnnotatorFileFactory & crclObject);
    }
    ;                                                 /* ResourceAnnotatorFileFactory */

    /* ----------------------------------------------------------------------- */
    /*       Implementation                                                    */
    /* ----------------------------------------------------------------------- */

    inline ResourceAnnotatorFile::ResourceAnnotatorFile(icu::UnicodeString const & crKey,
        icu::UnicodeString const & crKind)
        : FileResource(crKey, crKind),
        iv_pAnnotatorFile(0)
        /* ----------------------------------------------------------------------- */
    {
      ;
    }

  }

}
/* ----------------------------------------------------------------------- */
#endif /* UIMA_RES_ANNOTATOR_HPP */

/* <EOF> */

