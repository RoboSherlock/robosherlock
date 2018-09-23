#ifndef UIMA_LOWLEVEL_INDEXCOMPARATOR_HPP
#define UIMA_LOWLEVEL_INDEXCOMPARATOR_HPP
/** \file lowlevel_indexcomparator.hpp .
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
#include <uima/cas.hpp>
#include <uima/lowlevel_fsheap.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace lowlevel {
    class IndexDefinition;
  }
}
/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace lowlevel {


    /**
     * The abstract base class for all comparators used for creating indexes.
     * @see uima::lowlevel::IndexRepository
     */
    class UIMA_LINK_IMPORTSPEC IndexComparator {
    public:
      typedef enum {
        STANDARD_COMPARE,
        REVERSE_STANDARD_COMPARE
      } EnKeyFeatureComp;
    protected:
      typedef enum {
        BUILTIN_TYPE_INTEGER,
        BUILTIN_TYPE_FLOAT,
        BUILTIN_TYPE_STRING,
        BUILTIN_TYPE_BOOLEAN,
        BUILTIN_TYPE_BYTE,
        BUILTIN_TYPE_SHORT,
        BUILTIN_TYPE_LONG,
        BUILTIN_TYPE_DOUBLE,
        BUILTIN_TYPE_INVALID
      } EnBuiltinTypes;

      IndexDefinition const & iv_indexDefinition;
      TyFSType iv_tyType;

      std::vector<TyFSFeature> iv_features;
      std::vector<TyFeatureOffset> iv_offsets;
      std::vector<EnBuiltinTypes> iv_appropTypes;
      std::vector<EnKeyFeatureComp> iv_comparators;
    public:
      IndexComparator(IndexDefinition const & iv_indexDefinition,
                      TyFSType tyType);
      /**
       * Create a comparator where STANDARD_COMPARE is assumed for all features.
       * @param crKeyFeatures a list of key features (must have built-in range)
       */
      IndexComparator(IndexDefinition const & iv_indexDefinition,
                      TyFSType tyType,
                      std::vector<TyFSFeature> const & crKeyFeatures);

      void addKey(TyFSFeature tyFeat, EnKeyFeatureComp tyComp);


      /**
       * Compare two feature structures.
       * @return 1 if <code>tyFS1</code> is less than <code>tyFS2</code>,
       *         -1 if <code>tyFS1</code> is greater than <code>tyFS2</code>,
       *         0 if <code>tyFS1</code> equals <code>tyFS2</code>
       */
      int compare(uima::lowlevel::FSHeap const & heap, TyFS tyFS1, TyFS tyFS2) const;

      /**
       * return the type this comparator is capable to compare.
       */
      TyFSType getType() const {
        return iv_tyType;
      }

      std::vector<EnKeyFeatureComp> const & getComparisonOps() const {
        return iv_comparators;
      }

      std::vector<TyFSFeature> const & getKeyFeatures() const {
        return iv_features;
      }

    };

  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

