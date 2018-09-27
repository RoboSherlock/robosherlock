#ifndef UIMA_INTERNAL_FSPROMOTER_HPP
#define UIMA_INTERNAL_FSPROMOTER_HPP
/** \file internal_fspromoter.hpp .
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
#include <uima/featurestructure.hpp>
#include <uima/lowlevel_typedefs.hpp>
#include <uima/typesystem.hpp>
#include <uima/fsiterator.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class CAS;
  namespace lowlevel {
    class IndexIterator;
    class IndexRepository;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace internal {
    /**
     * tool class for promoting lowlevel FSs to OO API FSs.
     */
    class UIMA_LINK_IMPORTSPEC FSPromoter {
    public:
      static uima::FeatureStructure promoteFS(lowlevel::TyFS tyFS, uima::CAS & );
      static uima::FeatureStructure promoteFS(lowlevel::TyFS tyFS, uima::CAS const & );
      static uima::lowlevel::TyFS demoteFS(FeatureStructure const &);

      static uima::Type promoteType(lowlevel::TyFSType tyFSType, uima::lowlevel::TypeSystem & );
      static uima::Type promoteType(lowlevel::TyFSType tyFSType, uima::lowlevel::TypeSystem const & );
      static uima::lowlevel::TyFSType demoteType(Type const &);

      static uima::FSIterator promoteIterator(uima::lowlevel::IndexIterator *, uima::CAS &);

      static uima::lowlevel::TyFSFeature demoteFeature(Feature const &);
      static uima::Feature promoteFeature(lowlevel::TyFSFeature, uima::lowlevel::TypeSystem & );
      static uima::Feature promoteFeature(lowlevel::TyFSFeature, uima::lowlevel::TypeSystem const & );

      static uima::CAS* getFSCas(FeatureStructure const &);
      static uima::lowlevel::FSHeap* getFSHeap(FeatureStructure const &);
      static uima::lowlevel::FSHeap* getFSHeap(CAS &);
      static uima::lowlevel::TypeSystem* getTypeSystem(Feature const &);
      static uima::lowlevel::TypeSystem* getTypeSystem(Type const &);
    };
  }
}
/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

