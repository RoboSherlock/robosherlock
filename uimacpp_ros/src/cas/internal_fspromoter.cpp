/** \file internal_fspromoter.cpp .
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
#include <uima/internal_fspromoter.hpp>
#include <uima/cas.hpp>
#include <uima/internal_casimpl.hpp>
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
  namespace internal {

    FeatureStructure FSPromoter::promoteFS(lowlevel::TyFS tyFS, uima::CAS & cas) {
      //TODO? always use base CAS?
      return FeatureStructure(tyFS, cas);
    }

    FeatureStructure FSPromoter::promoteFS(lowlevel::TyFS tyFS, uima::CAS const & crFSSystem) {
      uima::CAS & rNonConstFSSys = CONST_CAST(uima::CAS&, crFSSystem);
      return FeatureStructure(tyFS, rNonConstFSSys);
    }

    lowlevel::TyFS FSPromoter::demoteFS(FeatureStructure const & crFS) {
      return crFS.iv_tyFS;
    }

    Type FSPromoter::promoteType(lowlevel::TyFSType tyFSType, lowlevel::TypeSystem & rFSSystem) {
      return Type(tyFSType, rFSSystem);
    }

    Type FSPromoter::promoteType(lowlevel::TyFSType tyFSType, lowlevel::TypeSystem const & crFSSystem) {
      lowlevel::TypeSystem & rNonConstFSSys = CONST_CAST(lowlevel::TypeSystem&, crFSSystem);
      return Type(tyFSType, rNonConstFSSys);
    }

    lowlevel::TyFSType FSPromoter::demoteType(Type const & crType) {
      return crType.iv_tyType;
    }

    uima::FSIterator FSPromoter::promoteIterator(uima::lowlevel::IndexIterator * it, CAS & fsSystem) {
      return uima::FSIterator(it, &fsSystem);
    }


    Feature FSPromoter::promoteFeature(lowlevel::TyFSFeature tyFSFeature, lowlevel::TypeSystem & rFSSystem) {
      return Feature(tyFSFeature, rFSSystem);
    }

    Feature FSPromoter::promoteFeature(lowlevel::TyFSFeature tyFSFeature, lowlevel::TypeSystem const & crFSSystem) {
      lowlevel::TypeSystem & rNonConstFSSys = CONST_CAST(lowlevel::TypeSystem&, crFSSystem);
      return Feature(tyFSFeature, rNonConstFSSys);
    }

    lowlevel::TyFSFeature FSPromoter::demoteFeature(Feature const & crFeature) {
      return crFeature.iv_tyFeature;
    }

    uima::CAS * FSPromoter::getFSCas(FeatureStructure const & crFS) {
      return crFS.iv_cas;
    }

    lowlevel::FSHeap * FSPromoter::getFSHeap(FeatureStructure const & crFS) {
      return crFS.iv_cas->getHeap();
    }
    lowlevel::TypeSystem * FSPromoter::getTypeSystem(Feature const & crF) {
      return crF.iv_typeSystem;
    }
    lowlevel::TypeSystem * FSPromoter::getTypeSystem(Type const & crT) {
      return crT.iv_typeSystem;
    }
    lowlevel::FSHeap* FSPromoter::getFSHeap(CAS & crCAS) {
      return crCAS.iv_heap;
    }

  }
}

/* ----------------------------------------------------------------------- */



