#ifndef UIMA_LOWLEVEL_INDEXREPOSITORY_HPP
#define UIMA_LOWLEVEL_INDEXREPOSITORY_HPP
/** \file indexrepository.hpp .
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
#include <list>
#include <uima/lowlevel_fsheap.hpp>
#include <uima/lowlevel_internal_indexes.hpp>
#include <uima/lowlevel_indexdefinition.hpp>
#include <uima/fsindexrepository.hpp>
#include <uima/types.h>
#include <uima/macros.h>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace lowlevel {
    class IndexDefinition;
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
     * The lowlevel index repository. Inherits from the OO API FSIndexRepository.
     */
    class UIMA_LINK_IMPORTSPEC IndexRepository : public FSIndexRepository {
      friend class uima::internal::CASSerializer;
      friend class uima::XCASWriter;
	  friend class uima::XmiWriter;
    private:
      IndexDefinition const & iv_indexDefinition;
      FSHeap & iv_rFSHeap;
      CAS & iv_cas;

      // iv_indexes[t][i] is the ith index registered for type t.
      std::vector<std::vector<internal::SingleIndex*> > iv_indexes;

      // idMaximalTypeMapping[t][id] is the position of the index with id
      //  at indexes[t]
      std::vector<std::map<IndexDefinition::TyIndexID, size_t> > iv_idMaximalTypeMapping;

      // idNonMaximalTypeIndexes[t][i] is the composite index for a nonmaximal type
      //  with id
      std::vector<std::map<IndexDefinition::TyIndexID, internal::CompositeIndex*> > iv_idNonMaximalTypeIndexes;

      // iv_cacheDirtyFlags[t] contains all composite indexes which do
      // not have to be updated. If a new FS of type t is added to the index
      // iv_cacheDirtyFlags[t] is cleared.
      std::vector<std::set<IndexABase const *> > iv_cacheDirtyFlags;

      // iv_isUsed[t] indicates if index for this type has something in it
      // iv_usedIndexes is list of indexes with something in it
      std::vector<bool> iv_isUsed;
      std::vector<int>  iv_usedIndexes;

      //constains FSs which have been added to the IndexRepository but
      //do not have an index definition in this CAS.
      std::vector<TyFS> iv_undefinedindex;

      bool iv_bIsInitialized;

      IndexRepository();
      IndexRepository(IndexRepository const &);
      IndexRepository & operator=(IndexRepository const &);


      void init();
      void clearAll();

      internal::SingleIndex* getSingleIndexForFS(TyFS fs, IndexDefinition::TyIndexID const & crID);
      internal::SingleIndex* getSingleIndexForType(TyFSType tyType, IndexDefinition::TyIndexID const & crID);

      std::vector<internal::SingleIndex*> const & getAllSingleIndexesForType(TyFSType tyType) const;

      void createIndex(TyFSType t, internal::IndexFactory* fac, IndexDefinition::TyIndexID const & id, bool bIsPermanent);

    protected:
      virtual uima::lowlevel::IndexRepository & getLowlevelIndexRepository() {
        return *this;
      }

      virtual uima::lowlevel::IndexRepository const & getLowlevelIndexRepository() const {
        return *this;
      }

    public:
      IndexRepository(IndexDefinition const &,
                      uima::lowlevel::FSHeap &,
                      CAS &);
      ~IndexRepository();

      IndexDefinition const & getIndexDefinition() const {
        return iv_indexDefinition;
      }

      CAS & getCas() const {
        return iv_cas;
      }

      FSHeap const & getFSHeap() const {
        return iv_rFSHeap;
      }

      FSHeap & getFSHeap()  {
        return iv_rFSHeap;
      }

      void getUsedIndexes(std::vector<TyFSType>& fillit);
      
	  //only used for serialization
      void getIndexedFSs(std::vector<TyFS>& fillit);

#ifndef NDEBUG
	  void print(std::ostream&) const;
#endif

      /**
       * reset all temporary indexes.
       */
      void reset();

      bool isInitialized() const {
        return iv_bIsInitialized;
      }

      /**
       * return true iff the index is up-to-date.
       */
      bool isDirtyForIndex(IndexABase const *) const;

      /**
       * marks the index as up-to-date.
       */
      void clearDirtyFlagForIndex(IndexABase const * );


      /**
       * reset all indexes including the index defintions.
       * Advanced use only.
       */
      void resetDefinitions();


      /**
       * get the index with ID <code>id</code> on type <code>tyType</code>.
       */
      IndexABase const & getLowlevelIndex(IndexDefinition::TyIndexID const &, TyFSType tyType) const;

      /**
       * add a feature structure to all possible indexes.
       */
      void add(TyFS tyFS);

      /**
       * add a feature structure to the index with ID <code>id</code>.
       */
      void add(TyFS tyFS, IndexDefinition::TyIndexID const &);

      /**
       * removes a feature structure to all possible indexes.
       */
      void remove(TyFS tyFS);

      /**
       * removes a feature structure to the index with ID <code>id</code>.
       */
      void remove(TyFS tyFS, IndexDefinition::TyIndexID const &);

      /**
       * check if the specified FS is in some index.
       */
      bool contains(TyFS) const;

    };

  }
}


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace lowlevel {

    inline internal::SingleIndex* IndexRepository::getSingleIndexForType(TyFSType tyType, IndexDefinition::TyIndexID const & crID) {
      assert( iv_bIsInitialized );
      assert( tyType < iv_idMaximalTypeMapping.size() );
      std::map<IndexDefinition::TyIndexID, size_t> const & aMap = iv_idMaximalTypeMapping[tyType];
      std::map<IndexDefinition::TyIndexID, size_t>::const_iterator it = aMap.find(crID);
      assert( it != iv_idMaximalTypeMapping[tyType].end() );
      size_t idIndex = (*it).second;
      assert( idIndex < iv_indexes[tyType].size() );
      return iv_indexes[tyType][idIndex];
    }


    inline internal::SingleIndex* IndexRepository::getSingleIndexForFS(TyFS fs, IndexDefinition::TyIndexID const & crID) {
      assert( iv_bIsInitialized );
      TyFSType type = iv_rFSHeap.getType(fs);
      return getSingleIndexForType(type, crID);
    }

    inline std::vector<internal::SingleIndex*> const & IndexRepository::getAllSingleIndexesForType(TyFSType tyType) const {
      assert( iv_bIsInitialized );
      assert( tyType < iv_idMaximalTypeMapping.size() );
      assert( iv_rFSHeap.getTypeSystem().isValidType(tyType) );
      return iv_indexes[tyType];
    }


    inline void IndexRepository::remove(TyFS fs) {
      assert( iv_bIsInitialized );
      TyFSType type = iv_rFSHeap.getType(fs);
      UIMA_TPRINT("Removing fs of type " << iv_rFSHeap.getTypeSystem().getTypeName(type) << ": " << (int) fs);
      assert( type < iv_indexes.size() );
      std::vector<internal::SingleIndex*>& typeIndexes = iv_indexes[type];

      std::vector<internal::SingleIndex*>::iterator it;
      for (it = typeIndexes.begin(); it != typeIndexes.end(); ++it) {
        assert( (*it)->getType() == type );
        (*it)->remove(fs);
      }
    }


    inline void IndexRepository::add(TyFS fs, IndexDefinition::TyIndexID const & crID) {
      assert( iv_bIsInitialized );
      internal::SingleIndex* ix = getSingleIndexForFS(fs, crID);
      ix->add(fs);
    }

    inline void IndexRepository::remove(TyFS fs, IndexDefinition::TyIndexID const & crID) {
      assert( iv_bIsInitialized );
      internal::SingleIndex* ix = getSingleIndexForFS(fs, crID);
      ix->remove(fs);
    }



  }
}
/* ----------------------------------------------------------------------- */


#endif


