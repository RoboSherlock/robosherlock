#ifndef UIMA_INTERNAL_CAS_DESERIALIZER_HPP
#define UIMA_INTERNAL_CAS_DESERIALIZER_HPP
/** \file internal_casdeserializer.hpp .
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
#include <vector>
#include <uima/unistrref.hpp>
#include <uima/internal_serializedcas.hpp>
#include <uima/lowlevel_fsheap.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class CAS;
  class ResultSpecification;
  namespace internal {
    class SerializedCAS;
  }
  namespace lowlevel {
    class FSHeap;
  }
}
/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {
    UIMA_EXC_CLASSDECLARE(DeserializationException, Exception);

    /**
     * Class for deserializing a CAS from a SerializedCAS object.
     * 
     * @see uima::internal::SerializedCAS
     * @see uima::internal::CASSerializer
     */
    class UIMA_LINK_IMPORTSPEC CASDeserializer {
    protected:

      std::vector<std::pair<uima::lowlevel::TyHeapCell*, uima::lowlevel::TyHeapCell*> > vecHeapSegments;
      std::vector<size_t> segmentStarts, segmentTops;
      size_t lastSegmentUsed;

#ifdef BYEBYEPTRS
      static UChar** resolveStringRef( size_t indexInDeserializedStringTable,
                                       uima::lowlevel::FSHeap::TyStringRefHeap const & stringRefHeap);
#endif

      void createStringTables(uima::internal::SerializedCAS const & crSerializedCAS,
                              uima::CAS & rCAS);


      void createFSHeap(uima::internal::SerializedCAS const & crSerializedCAS,
                        uima::CAS & rCAS);

      void createByteHeap(uima::internal::SerializedCAS const & crSerializedCAS,
                          uima::CAS & rCAS);

      void createShortHeap(uima::internal::SerializedCAS const & crSerializedCAS,
                           uima::CAS & rCAS);

      void createLongHeap(uima::internal::SerializedCAS const & crSerializedCAS,
                          uima::CAS & rCAS);


      bool isTypeSystemMergable(uima::internal::SerializedCAS const & crSerializedCAS,
                                uima::internal::CASDefinition const &);


      void createNewTypesAndFeatures(uima::internal::SerializedCAS const & crSerializedCAS,
                                     uima::internal::CASDefinition &);

      void addTypePriorities(uima::internal::SerializedCAS const & crSerializedCAS,
                             uima::internal::CASDefinition & rCAS);

      void checkFeatureOffsets(uima::internal::SerializedCAS const & crSerializedCAS,
                               uima::internal::CASDefinition const & crCAS);

      void deserializeFSHeapAndStringTable(uima::internal::SerializedCAS const & crSerializedCAS,
                                           uima::CAS & rCAS);

      void deserializeHeapsAndStringTable(uima::internal::SerializedCAS const & crSerializedCAS,
                                          uima::CAS & rCAS);


      void deserializeIndexedFSs(std::vector<SerializedCAS::TyNum> & iv_vecIndexedFSs,
                                 uima::CAS & rCAS);

      void deserializeTypeSystem(SerializedCAS const &, CASDefinition &);
      void deserializeIndexDefinitions(SerializedCAS const &, CASDefinition &);
      void swapBlob(void* buffer);

    public:

      static void deserializeResultSpecification(std::vector<SerializedCAS::TyNum> const & types,
          std::vector<SerializedCAS::TyNum> const & features,
          CASDefinition const &,
          ResultSpecification & result);

      CASDeserializer();
      ~CASDeserializer();

      void deserializeDefinitions(uima::internal::SerializedCAS const & serCAS,
                                  uima::internal::CASDefinition & result);


      void deserializeData(uima::internal::SerializedCAS const & serCAS,
                           uima::CAS & result);

      void deserializeBlob(void* buffer, uima::CAS & result);

    };

  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

