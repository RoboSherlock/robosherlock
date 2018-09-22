#ifndef UIMA_FSINDEXREPOSITORY_HPP
#define UIMA_FSINDEXREPOSITORY_HPP
/** \file fsindexrepository.hpp .
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

    \brief Contains FSIndexRepository

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <vector>

#include "unicode/unistr.h"
#include <uima/fsindex.hpp>
#include <uima/types.h>
//#include "uima/typesystem.hpp"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class Type;
  namespace lowlevel {
    class IndexRepository;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  UIMA_EXC_CLASSDECLARE(InvalidIndexIDException, CASException);

  /** Repository of indexes over feature structures.  Use this interface to access
   *  previously defined indexes.
   *
   */
  class UIMA_LINK_IMPORTSPEC FSIndexRepository {
    friend class uima::CAS;
  protected:
    virtual uima::lowlevel::IndexRepository & getLowlevelIndexRepository() = 0;
    virtual uima::lowlevel::IndexRepository const & getLowlevelIndexRepository() const = 0;

    FSIndexRepository();
    virtual ~FSIndexRepository();
  public:
    /**
     * Retrieve an index according to a label.
     * @param crLabel The name of the index.
     * @return The index with the name <code>label</code>, or <code>null</code>
     * if no such index is defined.
     */
    FSIndex getIndex(icu::UnicodeString const & crLabel) const;

    /**
     * Retrieve an index according to a label and a type.  The type is used to
     * narrow down the index of a more general type to a more specific one.
     * @param crLabel The name of the index.
     * @param crType  A subtype of the type of the index.
     * @return The specified, or <code>null</code> if an index with that name
     * doesn't exist, or it exists but <code>type</code> is not a subtype of
     * the index's type.
     * @throws WrongFSTypeForIndexException
     */
    FSIndex getIndex(icu::UnicodeString const & crLabel, Type const & crType) const;

    /**
     * Return the number of feature structures of <code>type</code> in the
     * repository.
     * @param crType The type of feature structures we're interested in.
     * @return The number of such feature structures in the index repository.
     */
    int getIndexSize(Type const & crType) const;

    /**
     * Get all index names available in the system.
     */
    std::vector<icu::UnicodeString> getAllIndexIDs() const;

    /**
     * get the most general type where the index is defined.
     */
    Type getTypeForIndex(icu::UnicodeString const & ) const;

    /**
     * check if there exists an index with ID <code>crID</code>.
     */
    bool isValidIndexID(icu::UnicodeString const & crID) const;

    /**
     * check if there exists an index with ID <code>id</code> for type
     * <code>crType</code>.
     */
    bool isValidIndexID(icu::UnicodeString const & crID, Type const & crType) const;

    /**
     * add the feature structure <code>crFS</code> to all indexes appropriate for its type.
     * @throws InvalidFSObjectException
     */
    void addFS(FeatureStructure const & crFS);

    /**
     * remove the feature structure <code>crFS</code> from all indexes appropriate for its type.
     * @throws InvalidFSObjectException
     */
    void removeFS(uima::FeatureStructure const & crFS);

  };

}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */



/* ----------------------------------------------------------------------- */


#endif

