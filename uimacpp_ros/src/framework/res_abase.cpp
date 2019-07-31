/** \file res_abase.cpp .
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

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
//#define DEBUG_VERBOSE
#include <uima/pragmas.hpp>
#include <uima/macros.h>
#include <uima/res_abase.hpp>
#include <uima/language.hpp>
#include <uima/resmgr.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {

  ResourceABase::ResourceABase(icu::UnicodeString const & crKey, icu::UnicodeString const & crKind)
      : iv_key(crKey),
      iv_kind(crKind) {}

  ResourceABase::~ResourceABase() {
    UIMA_TPRINT("Deleting Resource: " << this);
  }


  void ResourceABase::setNewKey(icu::UnicodeString const & crNewKey) {
    iv_key = crNewKey;
  }

  void FileResource::resolveFilename() {
    icu::UnicodeString file = getKey();
    file.append(".");
    file.append(getKind());

    char c[2056];
    int32_t len = file.extract(0, file.length(), c);
    assert( len == file.length());
    iv_fileName.setNew( c );   // set filename from extracted C-string
  }

  ////////////////////////////////////////////////////////////////

  FileResource::FileResource(icu::UnicodeString const & crKey,
                             icu::UnicodeString const & crKind)
      : ResourceABase(crKey, crKind) {}

  FileResource::~FileResource() {}

  ////////////////////////////////////////////////////////////////

  LanguageKindFileResource::LanguageKindFileResource (icu::UnicodeString const & crKey,
      icu::UnicodeString const & crKind,
      bool bTryAlternativeTerritories,
      char cLanguageTerritorySeparator)
      : FileResource(crKey, crKind),
      iv_bTryAlternativeTerritories(bTryAlternativeTerritories),
      iv_cLanguageTerritorySeparator(cLanguageTerritorySeparator) {
  }

  LanguageKindFileResource::~LanguageKindFileResource() {}


  void LanguageKindFileResource::resolveFilename() {
    const size_t bufLen = 2056;
    char buf[bufLen];
    icu::UnicodeString const & crKey = getKey();
    int32_t iConvLen = crKey.extract(0, crKey.length(), buf);
    assert( iConvLen < bufLen );
    uima::Language lang(buf);

    icu::UnicodeString crExt(".");
    crExt.append(getKind());
    iConvLen = crExt.extract(0, crExt.length(), buf);
    assert( iConvLen < bufLen );

    ResourceManager::createFilenameForLanguage(lang,
        buf,
        iv_bTryAlternativeTerritories,
        ResourceManager::getInstance().getLocationData(),
        iv_fileName);
  }


  /////////////////////////////////////////////////////////////////////

  ResourceFactoryABase::ResourceFactoryABase(icu::UnicodeString const & crKind)
      : iv_kind(crKind) {}

  ResourceFactoryABase::~ResourceFactoryABase() {}

  ///////////////////////////////////////////////////////////////////

  LanguageKindFileResourceFactory::LanguageKindFileResourceFactory(icu::UnicodeString const & crKind,
      bool bTryAlternativeTerritories)
      : ResourceFactoryABase(crKind),
      iv_bTryAlternativeTerritories( bTryAlternativeTerritories ) {}

  LanguageKindFileResourceFactory::~LanguageKindFileResourceFactory() {}


}

/* ----------------------------------------------------------------------- */



