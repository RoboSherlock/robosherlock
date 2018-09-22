#ifndef UIMA_LOWLEVEL_INDEXFACTORY_HPP
#define UIMA_LOWLEVEL_INDEXFACTORY_HPP
/** \file lowlevel_indexfactory.hpp .
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
#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/lowlevel_typesystem.hpp>
#include <uima/lowlevel_internal_indexes.hpp>
#include <uima/lowlevel_indexrepository.hpp>

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
  namespace lowlevel {
    namespace internal {

      /**
       * Base class for all index factories.
       * Every factory must be able to create a single and a composite index.
       */
      class UIMA_LINK_IMPORTSPEC IndexFactory {

      public:
        IndexFactory() {}

        virtual ~IndexFactory() {}

        virtual SingleIndex* createSingleIndex(IndexRepository const &, TyFSType) const = 0;
        virtual CompositeIndex* createCompositeIndex(IndexRepository const &, TyFSType) const = 0;

        virtual uima::lowlevel::TyFSType getType() const = 0;

        virtual IndexComparator const * getComparator() const = 0;

        virtual IndexDefinition::EnIndexKind getIndexKind() const = 0;

      };


      /**
       * A factory for creating indexes which use a comparator (ordered and set)
       */
      class UIMA_LINK_IMPORTSPEC ComparatorIndexFactory : public IndexFactory {
      protected:
        IndexComparator const * iv_cpclComparator;
      public:
        ComparatorIndexFactory(IndexComparator const * cpComparator)
            : IndexFactory(),
            iv_cpclComparator(cpComparator) {
          assert( EXISTS(iv_cpclComparator) );
        }

        virtual uima::lowlevel::TyFSType getType() const {
          assert( EXISTS(iv_cpclComparator) );
          return iv_cpclComparator->getType();
        }

        IndexComparator const * getComparator() const {
          return iv_cpclComparator;
        }

      };

      /**
       * The factory for ordered index.
       * Uses a comparator, thus inherits from ComparatorIndexFactory
       */
      class UIMA_LINK_IMPORTSPEC OrderedIndexFactory : public ComparatorIndexFactory {
      public:
        OrderedIndexFactory(IndexComparator const * aComparator)
            : ComparatorIndexFactory(aComparator) {}

        SingleIndex* createSingleIndex(IndexRepository const & indexRep,
                                       TyFSType aType) const {
          return new OrderedSingleIndex(indexRep, aType, iv_cpclComparator);
        }

        CompositeIndex* createCompositeIndex(IndexRepository const & indexRep, TyFSType tyFSType) const {
          return new OrderedCompositeIndex(indexRep, tyFSType, iv_cpclComparator);
        }

        virtual IndexDefinition::EnIndexKind getIndexKind() const {
          return IndexDefinition::enOrdered;
        }

      };

      /**
       * The factory for set index.
       * Uses a comparator, thus inherits from ComparatorIndexFactory
       */

      class UIMA_LINK_IMPORTSPEC SetIndexFactory : public ComparatorIndexFactory {
      public:
        SetIndexFactory(IndexComparator const * aComparator)
            : ComparatorIndexFactory(aComparator) {}

        SingleIndex* createSingleIndex(IndexRepository const & indexRep, TyFSType aType) const {
          return new SetSingleIndex(indexRep, aType, iv_cpclComparator);
        }

        CompositeIndex* createCompositeIndex(IndexRepository const & indexRep, TyFSType tyFSType) const {
          return new SetCompositeIndex(indexRep, tyFSType, iv_cpclComparator);
        }

        virtual IndexDefinition::EnIndexKind getIndexKind() const {
          return IndexDefinition::enSet;
        }

      };


      /**
       * Factory for the FIFO index.
       * Does not use a comparator, thus inherits directly from IndexFactory.
       */
      class UIMA_LINK_IMPORTSPEC FIFOIndexFactory : public IndexFactory {
      protected:
        uima::lowlevel::TyFSType iv_tyType;
      public:
        FIFOIndexFactory(uima::lowlevel::TyFSType tyType)
            : IndexFactory(),
            iv_tyType(tyType) {
        }

        SingleIndex* createSingleIndex(IndexRepository const & indexRep, TyFSType tyType) const {
          return new FIFOSingleIndex(indexRep, tyType);
        }

        CompositeIndex* createCompositeIndex(IndexRepository const & indexRep, TyFSType tyType) const {
          return new FIFOCompositeIndex(indexRep, tyType);
        }

        uima::lowlevel::TyFSType getType() const {
          return iv_tyType;
        }

        virtual IndexDefinition::EnIndexKind getIndexKind() const {
          return IndexDefinition::enFIFO;
        }

        virtual IndexComparator const * getComparator() const {
          return NULL;
        }

      };

    }
  }
}
/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

