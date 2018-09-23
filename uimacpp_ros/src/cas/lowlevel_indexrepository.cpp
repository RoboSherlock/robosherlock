/** \file lowlevel_indexrepository.cpp .
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

//#define DEBUG_VERBOSE
/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include <uima/macros.h>

#include <uima/lowlevel_indexrepository.hpp>
#include <uima/lowlevel_index.hpp>
#include <uima/lowlevel_indexiterator.hpp>
#include <uima/lowlevel_internal_indexfactory.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/engine.hpp>
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
  namespace lowlevel {

    IndexRepository::IndexRepository(IndexDefinition const & indexDef,
                                     uima::lowlevel::FSHeap & rFSHeap,
                                     CAS & cas)
        : iv_indexDefinition(indexDef),
        iv_cas(cas),
        iv_rFSHeap(rFSHeap),
        iv_bIsInitialized(false) {
      init();
    }

    void IndexRepository::init() {

      UIMA_TPRINT("constructing index store");
      iv_bIsInitialized = false;
      assert( !iv_bIsInitialized );

      iv_idMaximalTypeMapping.clear();
      iv_idNonMaximalTypeIndexes.clear();
      iv_indexes.clear();
      iv_cacheDirtyFlags.clear();
      iv_isUsed.clear();
      iv_usedIndexes.clear();

      size_t typeNum = iv_rFSHeap.getTypeSystem().getNumberOfTypes();
      iv_idMaximalTypeMapping.resize( typeNum + 1 );
      iv_indexes.resize( typeNum + 1);
      iv_idNonMaximalTypeIndexes.resize( typeNum + 1 );
      iv_cacheDirtyFlags.resize(typeNum+1);
      iv_isUsed.assign(typeNum+1, false);
      iv_bIsInitialized = true;
      UIMA_TPRINT("index store constructed");

      assert( iv_bIsInitialized );

      vector<IndexDefinition::TyIndexID> ids;
      iv_indexDefinition.getAllIndexIDs(ids);

      size_t i;

      for (i=0; i<ids.size(); ++i) {

        IndexDefinition::TyIndexID const & id = ids[i];
        uima::lowlevel::TyFSType indexType = iv_indexDefinition.getTypeForIndex(id);
        uima::lowlevel::internal::IndexFactory const * factory = iv_indexDefinition.getFactory(id);
        assert( EXISTS(factory) );

        vector<TyFSType> subsumedTypes;
        iv_indexDefinition.getTypeSystem().getSubsumedTypes(indexType, subsumedTypes);
        size_t i;
        // create the single indexes
        for (i=0; i<subsumedTypes.size(); ++i) {
          TyFSType type = subsumedTypes[i];
          internal::SingleIndex* ix = factory->createSingleIndex(*this, type);
          assert( type < iv_indexes.size() );
          iv_idMaximalTypeMapping[type][id] = iv_indexes[type].size();
          iv_indexes[type].push_back(ix);
        }

        // create all composite indexes
        for (i=0; i<subsumedTypes.size(); ++i) {
          TyFSType type = subsumedTypes[i];
          if (!iv_indexDefinition.getTypeSystem().isMaximalType(type)) {
            UIMA_TPRINT("creating index for non maximal type");
            internal::CompositeIndex* compIx = factory->createCompositeIndex(*this, type);
            iv_idNonMaximalTypeIndexes[type][id] = compIx;
            // add single indexes
            vector<TyFSType> subsumedTypes2;
            iv_indexDefinition.getTypeSystem().getSubsumedTypes(type, subsumedTypes2);
            size_t j;
            for (j=0; j<subsumedTypes2.size(); ++j) {
              TyFSType subsType = subsumedTypes2[j];
              assert( iv_idMaximalTypeMapping[subsType].find(id) != iv_idMaximalTypeMapping[subsType].end() );
              size_t pos = iv_idMaximalTypeMapping[subsType][id];
              compIx->addComponent( iv_indexes[subsType][pos] );
              UIMA_TPRINT("Adding single index of type " << iv_indexDefinition.getTypeSystem().getTypeName(subsType) );
            }
          }
        }
      }
    }


    IndexRepository::~IndexRepository() {
      clearAll();
    }

    void IndexRepository::clearAll() {
      size_t i,j;

      // delete indexes
      for (i=0; i<iv_indexes.size(); ++i) {
        for (j=0; j<iv_indexes[i].size(); ++j) {
          delete iv_indexes[i][j];
        }
      }
      // delete idNonMaximalTypeIndexes
      for (i=0; i<iv_idNonMaximalTypeIndexes.size(); ++i) {
        map<IndexDefinition::TyIndexID, internal::CompositeIndex*>::iterator it;
        for (it = iv_idNonMaximalTypeIndexes[i].begin(); it != iv_idNonMaximalTypeIndexes[i].end(); ++it) {
          delete (*it).second;
        }
      }

      iv_idMaximalTypeMapping.clear();
      iv_idNonMaximalTypeIndexes.clear();
      iv_indexes.clear();
      iv_cacheDirtyFlags.clear();
    }


    void IndexRepository::getUsedIndexes(vector<TyFSType>& fillit) {
      fillit.clear();
      fillit.assign(iv_usedIndexes.begin(), iv_usedIndexes.end());
    }

	void IndexRepository::getIndexedFSs(vector<TyFS>& fillit) {
      fillit.clear();
      for (size_t i=0;i < iv_undefinedindex.size(); i++ ) {
            TyFS tyFSHeapIndex = this->iv_undefinedindex[i];
            fillit.push_back(tyFSHeapIndex);
      }
	  for (size_t i=0; i<iv_usedIndexes.size(); ++i) {
          vector<uima::lowlevel::internal::SingleIndex*> const & crSingleIndexes =
            getAllSingleIndexesForType(iv_usedIndexes[i]);
          for (size_t j=0; j<crSingleIndexes.size(); ++j) {
            auto_ptr<uima::lowlevel::IndexIterator> apIt(crSingleIndexes[j]->createIterator());
            for (apIt->moveToFirst(); apIt->isValid(); apIt->moveToNext()) {
              uima::lowlevel::TyHeapCell pHeapCell = (uima::lowlevel::TyHeapCell) apIt->get();
              TyFS tyFSHeapIndex =  pHeapCell;
              fillit.push_back( tyFSHeapIndex );
            }
          }
      }
	  // eliminate duplicates
      sort(fillit.begin(), fillit.end());
      vector<TyFS>::iterator end = unique(fillit.begin(), fillit.end());
    }

    void IndexRepository::reset() {

      // clear undefined index
      iv_undefinedindex.clear();

      // check if anything to do
      if ( 0 == iv_usedIndexes.size())
        return;

      // Reset the indexes used
      for (size_t i=0; i<iv_usedIndexes.size(); ++i) {
        vector<uima::lowlevel::internal::SingleIndex*> const & crSingleIndexes = getAllSingleIndexesForType(iv_usedIndexes[i]);
        for (size_t j=0; j<crSingleIndexes.size(); ++j) {
          crSingleIndexes[j]->reset();
        }
        iv_cacheDirtyFlags[iv_usedIndexes[i]].clear();
        iv_isUsed[iv_usedIndexes[i]] = false;
      }
      iv_usedIndexes.clear();
    }


    void IndexRepository::resetDefinitions() {
      // reset all index data
      clearAll();
      init();
    }


    IndexABase const & IndexRepository::getLowlevelIndex(IndexDefinition::TyIndexID const & crID, TyFSType type) const {
      assert( iv_bIsInitialized );
      assert( iv_indexDefinition.getTypeSystem().isValidType(type) );
      assert( iv_indexDefinition.isValidIndexId(crID) );
      assert( iv_indexDefinition.getTypeSystem().subsumes( iv_indexDefinition.getTypeForIndex(crID), type) );

      IndexABase* result = NULL;
      if (iv_indexDefinition.getTypeSystem().isMaximalType(type)) {
        UIMA_TPRINT("requested index for maximal type");
        map<IndexDefinition::TyIndexID, size_t>::const_iterator cit = iv_idMaximalTypeMapping[type].find(crID);
        assert( cit != iv_idMaximalTypeMapping[type].end() );
        size_t i = (*cit).second;
        assert( (i >=0) && (i<iv_indexes[type].size()));
        result = iv_indexes[type][ i ];
      } else {
        UIMA_TPRINT("requested index for non maximal type");
        map<IndexDefinition::TyIndexID, internal::CompositeIndex*>::const_iterator cit = iv_idNonMaximalTypeIndexes[type].find(crID);
        assert( cit != iv_idNonMaximalTypeIndexes[type].end() );
        // composite index already exists
        result = (*cit).second;
      }
      assert( EXISTS(result) );
      return *result;
    }



    bool IndexRepository::contains(TyFS tyFS) const {
      assert( iv_bIsInitialized );
      TyFSType tyType = iv_rFSHeap.getType(tyFS);
      vector<internal::SingleIndex*> const & crSingleIndexes = getAllSingleIndexesForType(tyType);
      vector<internal::SingleIndex*>::const_iterator cit;
      for (cit = crSingleIndexes.begin(); cit != crSingleIndexes.end(); ++cit) {
        if ( (*cit)->contains(tyFS) ) {
          return true;
        }
      }
      return false;
    }


    void IndexRepository::add(TyFS fs) {
      assert( iv_bIsInitialized );
      TyFSType type = iv_rFSHeap.getType(fs);
      assert( iv_rFSHeap.getTypeSystem().isValidType(type) );
      UIMA_TPRINT("Adding fs of type " << iv_rFSHeap.getTypeSystem().getTypeName(type) << ": " << (int) fs);
      assert( type < iv_cacheDirtyFlags.size() );
      iv_cacheDirtyFlags[type].clear();

      assert( type < iv_indexes.size() );

      vector<internal::SingleIndex*>& typeIndexes = iv_indexes[type];

      if (!iv_isUsed[type]) {

        //if there is no index defined for this type
        if (typeIndexes.size() == 0 ) {
          iv_undefinedindex.push_back(fs);
          return;
        }

        iv_isUsed[type] = true;
        iv_usedIndexes.push_back(type);
      }


      // for all single indexes on the type of fs
      vector<internal::SingleIndex*>::iterator it;
      for (it = typeIndexes.begin(); it != typeIndexes.end(); ++it) {
        UIMA_TPRINT("index type: " << (*it)->getType());
        UIMA_TPRINT("     (" << iv_rFSHeap.getTypeSystem().getTypeName( (*it)->getType() ));
        assert( iv_rFSHeap.getTypeSystem().isValidType( (*it)->getType() ));
        assert( (*it)->getType() == type );
        (*it)->add(fs);
      }
    }


    void IndexRepository::clearDirtyFlagForIndex(IndexABase const * index) {
      assert(EXISTS(index) );
      TyFSType t = index->getType();
      assert( iv_indexDefinition.getTypeSystem().isValidType(t) );
      vector<TyFSType> types;
      iv_indexDefinition.getTypeSystem().getSubsumedTypes(t, types);
      vector<TyFSType>::const_iterator cit;
      for (cit = types.begin(); cit != types.end(); ++cit) {
        assert( (*cit) < iv_cacheDirtyFlags.size() );
        set<IndexABase const *> & rIndexSet = iv_cacheDirtyFlags[*cit];
        rIndexSet.insert(index);
      }
    }


    bool IndexRepository::isDirtyForIndex(IndexABase const * index) const {
      assert(EXISTS(index) );
      TyFSType t = index->getType();
      assert( iv_indexDefinition.getTypeSystem().isValidType(t) );
      vector<TyFSType> types;
      iv_indexDefinition.getTypeSystem().getSubsumedTypes(t, types);
      vector<TyFSType>::const_iterator cit;
      for (cit = types.begin(); cit != types.end(); ++cit) {
        assert( (*cit) < iv_cacheDirtyFlags.size() );
        set<IndexABase const *> const & crIndexSet = iv_cacheDirtyFlags[*cit];
        if ( crIndexSet.find(index) == crIndexSet.end() ) {
          return true;
        }
      }
      return false;
    }



    /*************************************************************/

#ifndef NDEBUG
    void IndexRepository::print(ostream& os) const {
      os << "===========================================" << endl;
      size_t i,j;
      for (i=1; i<iv_indexes.size(); ++i) {
        UIMA_TPRINT("index for type: " << i);
        assert( iv_indexDefinition.getTypeSystem().isValidType(i) );
        os << "Index for type " << iv_indexDefinition.getTypeSystem().getTypeName(i) << ":" << endl;
        for (j=0; j<iv_indexes[i].size(); ++j) {
          os << "  " << j << "th index: ";
          IndexABase* ix = iv_indexes[i][j];

          IndexIterator* it = ix->createIterator();
          for (it->moveToFirst(); it->isValid(); it->moveToNext() ) {
            os << (size_t) it->get()  << "  ";
          }
          os << endl;
          delete it;
        }
      }

      os << "===========================================" << endl;
    }
#endif


  }
}
/* ----------------------------------------------------------------------- */




