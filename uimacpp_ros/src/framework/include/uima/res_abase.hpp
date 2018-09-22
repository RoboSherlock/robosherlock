/** \file res_abase.hpp .
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

    \brief  Contains ResourceABase

   Description:

-----------------------------------------------------------------------------


   9/3/1999  Initial creation

-------------------------------------------------------------------------- */

#ifndef UIMA_RES_ABASE_HPP
#define UIMA_RES_ABASE_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> //must be included first to disable warnings

#include <string>

#include "unicode/unistr.h"
#include <uima/err_ids.h>
#include <uima/exceptions.hpp>
#include <uima/filename.hpp>
#include <uima/assertmsg.h>



/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class ResourceManager;
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  /**
   * This class is the abstract base class for all resources handled by the
   * resource manager (uima::ResourceManager). Override methods init() and
   * deInit(). A resource is uniquely identified by its "kind"
   * (e.g. if its a annotator, a stopword file, a FROST dictionary etc.)
   * and a "key" (e.g. the annotator name, the language of the stopword file etc.).
   * Objects of derived classes may only be created by objects of a class
   * derived from uima::ResourceFactoryABase.
   */
  class UIMA_LINK_IMPORTSPEC ResourceABase {
    friend class uima::ResourceManager;
  public:
    icu::UnicodeString const & getKey() const {
      return iv_key;
    }

    icu::UnicodeString const & getKind() const {
      return iv_kind;
    }
  protected:
    ResourceABase(icu::UnicodeString const & crKey, icu::UnicodeString const & crKind);
    virtual ~ResourceABase();

    // dont use between init() and deInit()
    void setNewKey(icu::UnicodeString const &);

    virtual void init(ErrorInfo &) = 0;
    virtual void deInit() = 0;
  private:
    icu::UnicodeString iv_key;
    icu::UnicodeString iv_kind;
  };


  /**
   * File resources are resources of the form "key.kind"
   * (e.g. en.twf, itusum.dll).
   * Call resolveFilename() in the init() method to set the protected
   * member iv_fileName to this name.

   */
  class UIMA_LINK_IMPORTSPEC FileResource : public ResourceABase {
  public:
    util::Filename const & getFilename() const {
      return iv_fileName;
    }
  protected:
    virtual ~FileResource();
    /**
     * construct the filename of the resource.
     * and set iv_fileName accordingly.
     * Can be used in init() of subclass.
     * default implementation assumes that the filename is "key.kind"
     * Note: No data path is prepended to the file name.
     */
    virtual void resolveFilename();

    FileResource(icu::UnicodeString const & crKey,
                 icu::UnicodeString const & crKind);

    util::Filename iv_fileName;
  };


  /**
   * LanguageKindFileResources are a special case of FileResources
   * in that the key always denotes a language (e.g. en.twf,
   * de-CH.tsw).
   * The overriden resolveFilename() has a special mimic for
   * handling territories (see uima::ResourceManager::createFilenameForLanguage()).
   */
  class UIMA_LINK_IMPORTSPEC LanguageKindFileResource : public FileResource {
  protected:
    virtual ~LanguageKindFileResource();

    /**
     * overrides method of superclass.
     * prepends the current data path to the filename.
     */
    virtual void resolveFilename();

    LanguageKindFileResource(icu::UnicodeString const & crKey,
                             icu::UnicodeString const & crKind,
                             bool bTryAlternativeTerritories,
                             char cLanguageTerritorySeparator = '-');

    bool iv_bTryAlternativeTerritories;
    char iv_cLanguageTerritorySeparator;
  };


  ////////////////////////////////////////////////////////////////////////////

  /**
   * The resource manager creates resources through an instance of this
   * class.
   */
  class UIMA_LINK_IMPORTSPEC ResourceFactoryABase {
  public:
    virtual ~ResourceFactoryABase();

    virtual ResourceABase * createResource(icu::UnicodeString const & crKey) const = 0;

    icu::UnicodeString const & getKind() const {
      return iv_kind;
    }
  protected:
    ResourceFactoryABase(icu::UnicodeString const & crKind);
  private:
    icu::UnicodeString iv_kind;
  };


  /**
   * A special factory for LanguageKindFileResources.
   * Simply contains the flag indicating if alternate territories
   * should be tried as a filename.
   */
  class UIMA_LINK_IMPORTSPEC LanguageKindFileResourceFactory : public ResourceFactoryABase {
  public:
    virtual ~LanguageKindFileResourceFactory();
    virtual ResourceABase * createResource(icu::UnicodeString const & crKey) const = 0;
  protected:
    LanguageKindFileResourceFactory(icu::UnicodeString const & crKind,
                                    bool bTryAlternativeTerritories);
    bool iv_bTryAlternativeTerritories;
  };

}


#endif /* UIMA_RES_ABASE_HPP */

/* <EOF> */

