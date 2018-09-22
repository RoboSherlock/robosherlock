/** \file lowlevel_indexdfinition.cpp .
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

#include <uima/msg.h>
#include <uima/engine.hpp>
#include <uima/lowlevel_indexdefinition.hpp>
#include <uima/lowlevel_internal_indexfactory.hpp>
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
  namespace lowlevel {

    IndexDefinition::IndexDefinition(uima::lowlevel::TypeSystem const & typeSystem)
        : iv_crTypeSystem(typeSystem),
        iv_bIsCommitted(false) {
    }

    IndexDefinition::~IndexDefinition() {
      clear();
    }

    void IndexDefinition::clear() {
      size_t i;
      // delete factories
      map<TyIndexID, internal::IndexFactory*>::iterator factoryIterator;
      for (factoryIterator = iv_mapFactories.begin(); factoryIterator != iv_mapFactories.end(); ++factoryIterator) {
        delete (*factoryIterator).second;
      }

      // delete comparators
      for (i=0; i<iv_vecComparators.size(); ++i) {
        delete iv_vecComparators[i];
        iv_vecComparators[i] = NULL;
      }
      iv_mapFactories.clear();
      iv_mapIndexTypes.clear();
      iv_mapIsPermanentFlags.clear();
      iv_vecComparators.clear();
      iv_bIsCommitted = false;
    }

    void IndexDefinition::reset() {
      clear();
    }


    bool IndexDefinition::isValidIndexId(IndexDefinition::TyIndexID const & crID, TyFSType tyType) const {
      return isValidIndexId(crID) && iv_crTypeSystem.subsumes( getTypeForIndex(crID), tyType);
    }

    /**
     * This method chooses the suitable factory depending on the index kind.
     */
    internal::IndexFactory * IndexDefinition::createFactory(EnIndexKind enIxKind, TyFSType tyType, IndexComparator const * pComparator) const {
      assert( !iv_bIsCommitted );
      internal::IndexFactory * pResult = NULL;
      switch (enIxKind) {
      case enOrdered: {
        assert( iv_crTypeSystem.subsumes( pComparator->getType(), tyType ) );
        pResult = new internal::OrderedIndexFactory(pComparator);
        break;
      }
      case enSet: {
        assert( iv_crTypeSystem.subsumes( pComparator->getType(), tyType ) );
        pResult = new internal::SetIndexFactory(pComparator);
        break;
      }
      case enFIFO: {
        pResult = new uima::lowlevel::internal::FIFOIndexFactory(tyType);
        break;
      }
      default:
        assert(false);

      }
      assert( pResult != NULL );
      return pResult;
    }



    void IndexDefinition::defineIndex(EnIndexKind enIxKind,
                                      TyFSType tyType,
                                      vector<uima::lowlevel::TyFSFeature> const & crKeyFeatures,
                                      vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> const & crComparators,
                                      IndexDefinition::TyIndexID const & crID,
                                      bool bIsPermanent) {
      assert(!iv_bIsCommitted);
      // if index does not yet exist
      if (! isValidIndexId(crID)) {
        // create comparator
        assert( crKeyFeatures.size() == crComparators.size() );
        uima::lowlevel::IndexComparator * pComparator = new uima::lowlevel::IndexComparator(*this, tyType);
        assert( EXISTS(pComparator) );

        size_t i;
        for (i=0; i<crKeyFeatures.size(); ++i) {
          pComparator->addKey(crKeyFeatures[i], crComparators[i]);
        }
        iv_vecComparators.push_back(pComparator);

        // create factory with the comparator
        internal::IndexFactory* pFactory = createFactory(enIxKind, tyType, pComparator);
        assert( pFactory != NULL );

        // register factory for index ID
        UIMA_TPRINT("creating index");
        assert( iv_mapFactories.find(crID) == iv_mapFactories.end() );
        iv_mapFactories[crID] = pFactory;
        assert( iv_crTypeSystem.subsumes( pFactory->getType(), tyType ) );
        assert( iv_mapIndexTypes.find(crID) == iv_mapIndexTypes.end() );

        // register type for index ID
        iv_mapIndexTypes[crID] = tyType;

        // register if index is contains permanent FSs
        assert( iv_mapIsPermanentFlags.find(crID) == iv_mapIsPermanentFlags.end() );
        iv_mapIsPermanentFlags[crID] = bIsPermanent;
      } else {
        // check if index is compatible with existing one
        UIMA_TPRINT(" An index with ID " << crID << " already exists, checking if it is compatible");

        if (! isCompatibleIndexDefinition( enIxKind, tyType, crKeyFeatures, crComparators, crID, bIsPermanent) ) {
          UIMA_EXC_THROW_NEW(IncompatibleIndexDefinitionsException,
                             UIMA_ERR_INCOMPATIBLE_INDEX_DEFINITIONS,
                             ErrorMessage(UIMA_MSG_ID_EXC_INCOMPATIBLE_INDEX_DEFINITIONS, crID),
                             UIMA_MSG_ID_EXCON_CREATING_INDEXES_FROM_CONFIG,
                             ErrorInfo::recoverable);
        }


      }
    }

    void IndexDefinition::commit() {
      iv_bIsCommitted = true;
    }

    uima::lowlevel::IndexComparator const * IndexDefinition::getComparator(IndexDefinition::TyIndexID const & crID) const {
      assert( isValidIndexId(crID) );
      map<IndexDefinition::TyIndexID, uima::lowlevel::internal::IndexFactory*>::const_iterator cit;
      cit = iv_mapFactories.find(crID);
      uima::lowlevel::internal::IndexFactory const * pFactory = (*cit).second;
      assert( EXISTS(pFactory) );
      uima::lowlevel::IndexComparator const * cpResult = pFactory->getComparator();
#ifndef NDEBUG
      if ( getIndexKind(crID) == enFIFO ) {
        assert( cpResult == NULL );
      } else {
        assert( EXISTS(cpResult) );
      }
#endif
      return cpResult;
    }

    IndexDefinition::EnIndexKind IndexDefinition::getIndexKind(IndexDefinition::TyIndexID const & crID) const {
      assert( isValidIndexId(crID) );
      map<IndexDefinition::TyIndexID, uima::lowlevel::internal::IndexFactory*>::const_iterator cit;
      cit = iv_mapFactories.find(crID);
      uima::lowlevel::internal::IndexFactory const * pFactory = (*cit).second;
      assert( EXISTS(pFactory) );
      return pFactory->getIndexKind();
    }

    void IndexDefinition::getAllIndexIDs(vector<IndexDefinition::TyIndexID>& rResult) const {
      rResult.clear();
      map<IndexDefinition::TyIndexID, TyFSType>::const_iterator cit;
      for (cit = iv_mapIndexTypes.begin(); cit != iv_mapIndexTypes.end(); ++cit) {
        rResult.push_back( (*cit).first );
      }
    }


    bool IndexDefinition::isCompatibleIndexDefinition(EnIndexKind enIxKind,
        TyFSType tyType,
        vector<uima::lowlevel::TyFSFeature> const & crKeyFeatures,
        vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> const & crComparators,
        IndexDefinition::TyIndexID const & crID,
        bool ) const {
      assert(isValidIndexId(crID));

      if (getIndexKind(crID) != enIxKind) {
        UIMA_TPRINT("Wrong index kind!");
        return false;
      }
      if (getTypeForIndex(crID) != tyType) {
        UIMA_TPRINT("Wrong index type!");
        return false;
      }
      uima::lowlevel::IndexComparator const * cpComp = getComparator(crID);
      if (cpComp != NULL) {
        assert( (enIxKind == enOrdered) || (enIxKind == enSet) );
        assert( cpComp->getType() == tyType );

        if (crKeyFeatures.size() != cpComp->getKeyFeatures().size()) {
          UIMA_TPRINT("key feature length different, number of key features: " << crKeyFeatures.size()
                      << ", existing key feature length: " << cpComp->getKeyFeatures().size());
          return false;
        }
        if (crKeyFeatures != cpComp->getKeyFeatures()) {
          UIMA_TPRINT("Wrong key features!");
          return false;
        }
        if (crComparators != cpComp->getComparisonOps() ) {
          UIMA_TPRINT("Wrong comparison ops!");
          return false;
        }
      } else {
        assert( enIxKind == enFIFO );
      }
      return true;
    }

  }
}




