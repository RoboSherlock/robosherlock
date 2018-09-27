#ifndef UIMA_LOWLEVEL_INDEXDEFINITION_HPP
#define UIMA_LOWLEVEL_INDEXDEFINITION_HPP
/** \file lowlevel_indexdefinition.hpp .
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

#include <map>
#include <vector>
#include <uima/lowlevel_typedefs.hpp>
#include <uima/lowlevel_indexcomparator.hpp>
#include "unicode/unistr.h"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace lowlevel {
    class TypeSystem;
    class IndexComparator;
    namespace internal {
      class IndexFactory;
    }
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace lowlevel {

    /**
     * This class contains all index definitions together with the factories
     * needed for actually creating them.
     */
    class UIMA_LINK_IMPORTSPEC IndexDefinition {
      friend class uima::lowlevel::IndexRepository;
    public:
      typedef icu::UnicodeString TyIndexID;
      typedef enum {
        enOrdered = 0,
        enSet = 1,
        enFIFO = 2,
        enIndexKindNumber = 3
      } EnIndexKind;
    private:
      TypeSystem const & iv_crTypeSystem;
      // maps ids to index factories
      std::map<TyIndexID, internal::IndexFactory*> iv_mapFactories;

      // iv_mapIndexTypes[i] is the type of the index with ID i
      std::map<TyIndexID, TyFSType> iv_mapIndexTypes;

      // iv_mapIsPermanentFlags[i] is true if index with ID i contains permanent feature structures
      std::map<TyIndexID, bool> iv_mapIsPermanentFlags;
      std::vector<uima::lowlevel::IndexComparator*> iv_vecComparators;

      bool iv_bIsCommitted;

      /**
       * checks if the index defined in the first four arguments is compatible to
       * the one with ID crID.
       */
      bool isCompatibleIndexDefinition(EnIndexKind enIxKind,
                                       TyFSType tyType,
                                       std::vector<uima::lowlevel::TyFSFeature> const & crKeyFeatures,
                                       std::vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> const & crComparators,
                                       TyIndexID const & crID,
                                       bool ) const;
      /**
       * create a factory for the index defined by the arguments to this method.
       */
      uima::lowlevel::internal::IndexFactory * createFactory(EnIndexKind enIxKind,
          TyFSType,
          IndexComparator const * pComparator) const;

      void clear();

      IndexDefinition(IndexDefinition const &);
      IndexDefinition& operator=(IndexDefinition const &);

    public:
      IndexDefinition(uima::lowlevel::TypeSystem const &);
      ~IndexDefinition();

      TypeSystem const & getTypeSystem() const {
        return iv_crTypeSystem;
      }

      void commit();

      bool isCommitted() const {
        return iv_bIsCommitted;
      }

      /*@{*/
      /**
       * @name Index Creation
       */

      /**
       * add an index definition
       *
       * @param enIxKind the kind of the index
       * @param tyType the most general type where the index should be introduced
       * @param crKeyFeatures a vector of key features used for comparison
       * @param crComparators a vector defining how the key features should be compared
       * @param id the ID under which this index can be identified
       * @param bIsPermanent flag indicating if this index contains only permanent feature structures
       */
      void defineIndex(EnIndexKind enIxKind,
                       TyFSType tyType,
                       std::vector<uima::lowlevel::TyFSFeature> const & crKeyFeatures,
                       std::vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> const & crComparators,
                       TyIndexID const & id,
                       bool bIsPermanent = false);


      /*@}*/

      /**
       * check if an index with ID <code>id</code> exists.
       */
      bool isValidIndexId(TyIndexID const & crID) const {
        return(iv_mapFactories.find(crID) != iv_mapFactories.end());
      }

      /**
       * returns the comparator for the index with the specidied ID.
       * If the index has no comparator, NULL is returned.
       */
      uima::lowlevel::IndexComparator const * getComparator(TyIndexID const & crID) const;

      /**
       * get the kind of the index.
       */
      EnIndexKind getIndexKind(TyIndexID const & crID) const;

      /**
       * check if an index with ID <code>id</code> for type <code>tyType</code> exists.
       */
      bool isValidIndexId(TyIndexID const &, TyFSType tyType) const;

      /**
       * get the number of existing indexes.
       */
      size_t getNumberOfIndexes() const {
        return iv_mapFactories.size();
      }

      /**
       * get the most general type the index with ID <code>id</code> is defined for.
       */
      TyFSType getTypeForIndex(TyIndexID const & crID) const {
        //assert( isValidIndexId(crID) );
	std::map<TyIndexID, TyFSType>::const_iterator cit = iv_mapIndexTypes.find(crID);
        return (*cit).second;
      }

      uima::lowlevel::internal::IndexFactory const * getFactory(TyIndexID const & crID) const {
	std::map<TyIndexID, internal::IndexFactory*>::const_iterator cit = iv_mapFactories.find(crID);
        if (cit == iv_mapFactories.end()) {
          return NULL;
        }
        return (*cit).second;
      }

      /**
       * get the IDs of all indexes.
       * @param rResult output parameter
       */
      void getAllIndexIDs(std::vector<TyIndexID>& rResult) const;

      void reset();

    };
  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif


