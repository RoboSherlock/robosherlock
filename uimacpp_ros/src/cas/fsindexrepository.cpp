/** \file fsindexrepository.cpp .
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
#include <uima/pragmas.hpp>
#include <uima/fsindexrepository.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/lowlevel_indexrepository.hpp>
#include <uima/msg.h>

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
using namespace std;
namespace uima {

  UIMA_EXC_CLASSIMPLEMENT(InvalidIndexIDException, CASException);

  FSIndexRepository::FSIndexRepository() {}

  FSIndexRepository::~FSIndexRepository() {}

  FSIndex FSIndexRepository::getIndex(icu::UnicodeString const & crLabel) const {
    return getIndex(crLabel, getTypeForIndex(crLabel));
  }

  std::vector<icu::UnicodeString> FSIndexRepository::getAllIndexIDs() const {
    vector<icu::UnicodeString> ids;
    getLowlevelIndexRepository().getIndexDefinition().getAllIndexIDs(ids);
    return ids;
  }

  FSIndex FSIndexRepository::getIndex(icu::UnicodeString const & crLabel, Type const & crType) const {
    uima::lowlevel::TyFSType tyType = uima::internal::FSPromoter::demoteType(crType);
    if (! isValidIndexID( crLabel ) ) {
      ErrorMessage msg(UIMA_MSG_ID_EXCON_GETTING_INDEX);
      msg.addParam(crLabel);

      UIMA_EXC_THROW_NEW(InvalidIndexIDException,
                         UIMA_ERR_INVALID_INDEX_ID,
                         UIMA_MSG_ID_EXC_INVALID_INDEX_ID,
                         msg,
                         ErrorInfo::recoverable
                        );
    }
    if ( ! getTypeForIndex(crLabel).subsumes( crType) ) {
      ErrorMessage msg(UIMA_MSG_ID_EXCON_GETTING_INDEX);
      msg.addParam(crLabel);
      UIMA_EXC_THROW_NEW(WrongFSTypeForIndexException,
                         UIMA_ERR_WRONG_FSTYPE_FOR_INDEX,
                         UIMA_MSG_ID_EXC_WRONG_FSTYPE_FOR_INDEX,
                         msg,
                         ErrorInfo::recoverable
                        );
    }

    uima::lowlevel::IndexABase const * cpIx = & getLowlevelIndexRepository().getLowlevelIndex(crLabel, tyType);
    return FSIndex(cpIx, getLowlevelIndexRepository());
  }

  int FSIndexRepository::getIndexSize(Type const & crType) const {
    assertWithMsg(false, "Not implemented yet!");
    UIMA_EXC_THROW_NEW(NotYetImplementedException,
                       UIMA_ERR_NOT_YET_IMPLEMENTED,
                       UIMA_MSG_ID_EXC_NOT_YET_IMPLEMENTED,
                       ErrorMessage(UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT),
                       ErrorInfo::unrecoverable
                      );

    return -1;
  }

  Type FSIndexRepository::getTypeForIndex(icu::UnicodeString const & crLabel) const {
    uima::lowlevel::TyFSType tyType = getLowlevelIndexRepository().getIndexDefinition().getTypeForIndex(crLabel);
    return uima::internal::FSPromoter::promoteType(tyType, getLowlevelIndexRepository().getFSHeap().getTypeSystem() );
  }

  bool FSIndexRepository::isValidIndexID(icu::UnicodeString const & crLabel) const {
    return getLowlevelIndexRepository().getIndexDefinition().isValidIndexId(crLabel);
  }

  bool FSIndexRepository::isValidIndexID(icu::UnicodeString const & crLabel, Type const & crType) const {
    if (!isValidIndexID(crLabel) ) {
      return false;
    }
    return getTypeForIndex(crLabel).subsumes(crType);
  }


  void FSIndexRepository::addFS(FeatureStructure const & crFS) {
    if (!crFS.isValid()) {
      UIMA_EXC_THROW_NEW(InvalidFSObjectException,
                         UIMA_ERR_INVALID_FSTYPE_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_FSTYPE_OBJECT,
                         ErrorMessage(UIMA_MSG_ID_EXCON_ADDING_FS_TO_INDEX),
                         ErrorInfo::recoverable
                        );
    }
    getLowlevelIndexRepository().add( internal::FSPromoter::demoteFS( crFS ) );
  }

  void FSIndexRepository::removeFS(FeatureStructure const & crFS) {
    if (!crFS.isValid()) {
      UIMA_EXC_THROW_NEW(InvalidFSObjectException,
                         UIMA_ERR_INVALID_FSTYPE_OBJECT,
                         UIMA_MSG_ID_EXC_INVALID_FSTYPE_OBJECT,
                         ErrorMessage(UIMA_MSG_ID_EXCON_REMOVING_FS_FROM_INDEX),
                         ErrorInfo::recoverable
                        );
    }
    getLowlevelIndexRepository().remove( internal::FSPromoter::demoteFS( crFS ) );
  }


}




