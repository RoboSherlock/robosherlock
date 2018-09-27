#ifndef UIMA_INTERNAL_CASSERIALIZER_HPP
#define UIMA_INTERNAL_CASSERIALIZER_HPP
/** \file filename .
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
#include "unicode/unistr.h"
#include <vector>
#include <map>
#include <iostream>
#include <uima/lowlevel_typedefs.hpp>
#include <uima/lowlevel_indexrepository.hpp>
#include <uima/internal_serializedcas.hpp>
#include <uima/timedatetools.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class ResultSpecification;
  namespace internal {
    class CASDefinition;
  }
}

/* taph 30.07.2003: use this define to turn on serialization timing here and
   in taf_jni/taf_jni.cpp
   Let's turn it off for now so we don't spoil our performance by measuring
   time and outputing timing information
 */
//#define UIMA_ENABLE_SERIALIZATION_TIMING

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace internal {

    /**
     * Class for serializing a CAS and CASDefinition.
     * The actual data is kept in a SerializedCAS object.
     * 
     * @see uima::internal::SerializedCAS
     * @see uima::internal::CASDeserializer
     */
    class UIMA_LINK_IMPORTSPEC CASSerializer {
    private:
      bool iv_bCopyStrings;
#ifdef UIMA_ENABLE_SERIALIZATION_TIMING
      Timer iv_timerFSHeap;
      Timer iv_timerIndexedFSs;
#endif
      // for blob serialization
      std::vector<SerializedCAS::TyNum> iv_vecIndexedFSs;

      CASSerializer(CASSerializer const &);
      CASSerializer & operator=(CASSerializer const &);

      typedef std::map<UChar*, uima::internal::SerializedCAS::TyNum> TyStringMap;
      SerializedCAS::TyNum adjustString(uima::lowlevel::TyHeapCell tyFeatureCell, TyStringMap &, uima::internal::SerializedCAS & rSerializedCAS);

      UnicodeStringRef createString(UChar const *, size_t, uima::internal::SerializedCAS & );

      void serializeTypeSystem(uima::internal::CASDefinition const &, uima::internal::SerializedCAS & rSerializedCAS);
      void serializeIndexDefinition(uima::internal::CASDefinition const &, uima::internal::SerializedCAS & rSerializedCAS);

      void serializeFSHeapAndStringHeap(uima::CAS const &, uima::internal::SerializedCAS & rSerializedCAS);
      void serializeHeaps(uima::CAS const &, uima::internal::SerializedCAS & rSerializedCAS);

      void serializeIndexedFSs(uima::CAS &, std::vector<uima::internal::SerializedCAS::TyNum> & iv_vecIndexedFSs);

    public:
      static void serializeResultSpec(ResultSpecification const & resultSpec,
                                      std::vector<internal::SerializedCAS::TyNum>& resultSpecTypes,
                                      std::vector<internal::SerializedCAS::TyNum>& resultSpecFeatures);


      CASSerializer(bool bCopyStrings);
      ~CASSerializer();

      // serialize definition (fixed after init)
      void serializeDefinitions(CASDefinition const & casDef,
                                uima::internal::SerializedCAS & result);

      // serialize document data
      void serializeData(uima::CAS & cas,
                         uima::internal::SerializedCAS & result);

      // estimate total size of serialized CAS data
      size_t getBlobSize(uima::CAS & cas);

      // serialize CAS data into single blob format
      size_t getBlob(uima::CAS & cas, void * buffer, size_t maxSize);

#ifdef UIMA_ENABLE_SERIALIZATION_TIMING
      Timer const & getHeapTimer() const {
        return iv_timerFSHeap;
      }

      Timer const & getIndexTimer() const {
        return iv_timerIndexedFSs;
      }
#endif
    };
  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif
