#ifndef UIMA_INTERNAL_SERIALIZEDCAS_HPP
#define UIMA_INTERNAL_SERIALIZEDCAS_HPP
/** \file internal_serializedcas.hpp .
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
#include <uima/types.h>
#include <uima/unistrref.hpp>
#include <uima/lowlevel_internal_heap.hpp>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {
    class CASSerializer;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {

    /**
     * SerializedCAS is the container for all data structure which are the
     * result of a serialized CAS/CASDefinition.
     * 
     * The data format is now described in detail:
     * 1. Type system: In general, every type and feature is basically an integer > 0.
     *    - type inheritance table: an integer array which for each type contains its parent,
     *      e.g. iv_vecTypeInheritanceTable[ t ] is t's parent.
     *      (length: number of types)
     *    - feature definition table: an integer array containing for each feature its introduction 
     *      and range type, i.e.,
     *      iv_vecFeatureDefinitionTable[ 2*f   ] is the intro type of f and
     *      iv_vecFeatureDefinitionTable[ 2*f+1 ] is the range type of f and
     *      (length: 2 * (number of features+1) )
     *    - feature offset table: an integer array indicating for each feature its offset
     *      on the heap, i.e.,
     *      iv_vecFeatureOffsetTable[ f ]
     *      (length: number of features)
     *    - type priority table: integer array indicating the internal type priority number:
     *      Type t1 has priority over t2 iff 
     *        iv_vecTypePriorityTable[ t1 ] < iv_vecTypePriorityTable[ t2 ]
     *      (length: number of types)
     *    - type symbol table: string array indicating the name of each type:
     *      iv_vecTypeSymbolTable[ t ] is the fully qualified name of t
     *      (length: number of types)
     *    - feature symbol table: string array indicating the name of each feature:
     *      iv_vecFeatureSymbolTable[ f ] is the fully qualified name of f
     *      (length: number of features)
     *    - string sub types: integer array indicating which types are string sub types
     *      iv_stringSubTypes[ i ] is the ith string sub types
     *      (length: number of string sub types(
     *    - string subtype values: string array containing the string values for each string subtype:
     *      iv_stringSubTypeValues[ i ] is some string sub type value
     *      (length: nunber of string subtype values)
     *    - string subtype value positions: integer array indicating where the string values in
     *      the array above start:
     *      iv_stringSubTypeValuePos[ t ] is the index where the string sub types in 
     *        iv_stringSubTypeValues start.
     * 
     * 2. Index definitions:
     *    - index ID table: string array indication the names of the indexes:
     *      iv_vecIndexIDTable[ i ] is the name of the ith index
     *      (length: number of indexes)
     *    - comparator definition table: integer array of pairs of a feature and a comp ID:
     *      iv_vecComparatorDefinitionTable[ i ] is a feature and
     *      iv_vecComparatorDefinitionTable[ i+1 ] is an integer indicating if the comparison
     *           is standard or reverse
     *      for some adequate i (see next bullet)
     *      (length: number of overall features for all comparators)
     *    - comparator start table: integer array indicating for each index ID where its 
     *         comparator description starts in the comparator definition table:
     *      iv_vecComparatorStartTable[ i ] is an index into iv_vecComparatorDefinitionTable for 
     *         the index with the name iv_vecIndexIDTable[ i ]
     *      (length: number of indexes)
     * 
     * 3. The Feature Structure Heap:
     *    - heap array: the heap as one contiguous integer array:
     *      iv_vecFSHeapArray[ i ] is the ith heap cell
     *      (length: overall heap size)
     *    - string symbol table: the string symbol table for the heap as a string array
     *      iv_vecStringSYmbolTable[ i ] is the ith string referenced on the heap.
     *      (length: number of strings referenced on the heap)
     * 
     * 4. Indexes:
     *    - indexed FSs: integer array containing all indexed FSs
     *      iv_vecIndexedFSs[ i ] is the ith feature structure in any index
     *      (length: number of feature structures in any index)
     *    
     */
    class UIMA_LINK_IMPORTSPEC SerializedCAS {
      friend class uima::internal::CASSerializer;
    public:
      typedef WORD32 TyNum;
    private:
      SerializedCAS & operator=(SerializedCAS const &);
      SerializedCAS(SerializedCAS const &);

      typedef uima::lowlevel::internal::Heap<UChar> TyStringHeap;
      TyStringHeap iv_stringHeap;

      UnicodeStringRef addString(UnicodeStringRef const & crRef);

      icu::UnicodeString iv_emptyString;

    public:
      UnicodeStringRef iv_ulstrDocument;

      std::vector<TyNum> iv_vecTypeInheritanceTable;
      std::vector<TyNum> iv_vecFeatureDefinitionTable;
      std::vector<UnicodeStringRef> iv_vecTypeSymbolTable;
      std::vector<UnicodeStringRef> iv_vecFeatureSymbolTable;
      std::vector<TyNum> iv_vecFeatureOffsetTable;
      std::vector<TyNum> iv_vecTypePriorityTable;

      std::vector<TyNum> iv_stringSubTypes;
      std::vector<UnicodeStringRef> iv_stringSubTypeValues;
      std::vector<TyNum> iv_stringSubTypeValuePos;

      std::vector<UnicodeStringRef> iv_vecIndexIDTable;
      std::vector<TyNum> iv_vecComparatorDefinitionTable;
      std::vector<TyNum> iv_vecComparatorStartTable;
      std::vector<TyNum> iv_vecIndexKindTable;

      std::vector<TyNum> iv_vecFSHeapArray;
      std::vector<UnicodeStringRef> iv_vecStringSymbolTable;

      std::vector<TyNum> iv_vecIndexedFSs;

      std::vector<char> iv_vecByteHeapArray;
      std::vector<short> iv_vecShortHeapArray;
      std::vector<INT64> iv_vecLongHeapArray;


    public:
      SerializedCAS();

      ~SerializedCAS();

      // type system
      std::vector<TyNum> const & getTypeInheritanceTable() const {
        return iv_vecTypeInheritanceTable;
      }

      std::vector<TyNum> const & getFeatureDefinitionTable() const {
        return iv_vecFeatureDefinitionTable;
      }

      std::vector<UnicodeStringRef> const & getTypeSymbolTable() const {
        return iv_vecTypeSymbolTable;
      }

      std::vector<UnicodeStringRef> const & getFeatureSymbolTable() const {
        return iv_vecFeatureSymbolTable;
      }

      std::vector<TyNum> const & getFeatureOffsetTable() const {
        return iv_vecFeatureOffsetTable;
      }

      std::vector<TyNum> const & getTypePriorityTable() const {
        return iv_vecTypePriorityTable;
      }

      std::vector<TyNum> const & getStringSubTypes() const {
        return iv_stringSubTypes;
      }

      std::vector<UnicodeStringRef> const & getStringSubTypeValues() const {
        return iv_stringSubTypeValues;
      }

      std::vector<TyNum> const & getStringSubTypeValuePos() const {
        return iv_stringSubTypeValuePos;
      }

      // index definition
      std::vector<UnicodeStringRef> const & getIndexIDTable() const {
        return iv_vecIndexIDTable;
      }

      std::vector<TyNum> const & getComparatorDefinitionTable() const {
        return iv_vecComparatorDefinitionTable;
      }

      std::vector<TyNum> const & getComparatorStartTable() const {
        return iv_vecComparatorStartTable;
      }

      std::vector<TyNum> const & getIndexKindTable() const {
        return iv_vecIndexKindTable;
      }

      // document
      UnicodeStringRef getDocument() const {
        return iv_ulstrDocument;
      }

      // fsheap
      std::vector<TyNum> const & getFSHeapArray() const {
        return iv_vecFSHeapArray;
      }

      // 8bit heap
      std::vector<char> const & getByteHeapArray() const {
        return iv_vecByteHeapArray;
      }

      // 16bit heap
      std::vector<short> const & getShortHeapArray() const {
        return iv_vecShortHeapArray;
      }

      // 8bit heap
      std::vector<INT64> const & getLongHeapArray() const {
        return iv_vecLongHeapArray;
      }



      std::vector<UnicodeStringRef> const & getStringSymbolTable() const {
        return iv_vecStringSymbolTable;
      }

      // indexes
      std::vector<TyNum> const & getIndexedFSs() const {
        return iv_vecIndexedFSs;
      }

      void reset();

      // returns how many real Java objects (arrays and string) must be created
      //  in order to create a serialized representations of this CAS
      size_t getNumberOfJavaObjectsToBeCreatedDefinitions() const {
        return iv_vecTypeSymbolTable.size()
               + iv_vecFeatureSymbolTable.size()
               + iv_vecIndexIDTable.size()
               + iv_stringSubTypeValues.size()
               + 64;
      }

      size_t getNumberOfJavaObjectsToBeCreatedData() const {
        return iv_vecStringSymbolTable.size() + 64;
      }

      void print(std::ostream &) const;
    };

  }
}
/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {

    inline UnicodeStringRef SerializedCAS::addString(UnicodeStringRef const & crRef) {
      size_t l = crRef.length();
      int pOff = iv_stringHeap.increaseHeap(l + 1);
      UChar* p = iv_stringHeap.getHeapStart() + pOff;
      assert( (2*l) == crRef.getSizeInBytes() );
      memcpy(p, crRef.getBuffer(), crRef.getSizeInBytes() );
      return UnicodeStringRef(p,l);
    }
  }
}


/* ----------------------------------------------------------------------- */


#endif

