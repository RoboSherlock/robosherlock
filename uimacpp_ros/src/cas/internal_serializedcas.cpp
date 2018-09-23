/** \file internal_serializedcas.cpp .
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

#include <uima/internal_serializedcas.hpp>

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
  namespace internal {

//!TODO check this default size, particularly for blob-based data serialization
    SerializedCAS::SerializedCAS()
        : iv_stringHeap(10000, 0) {}

    SerializedCAS::~SerializedCAS() {}


    void SerializedCAS::reset() {
      iv_stringHeap.reset();

      iv_ulstrDocument = UnicodeStringRef();

      iv_vecTypeInheritanceTable.clear();
      iv_vecFeatureDefinitionTable.clear();
      iv_vecTypeSymbolTable.clear();
      iv_vecFeatureSymbolTable.clear();
      iv_vecFeatureOffsetTable.clear();
      iv_vecTypePriorityTable.clear();

      iv_vecIndexIDTable.clear();
      iv_vecComparatorDefinitionTable.clear();
      iv_vecComparatorStartTable.clear();
      iv_vecIndexKindTable.clear();

      iv_vecFSHeapArray.clear();
      iv_vecByteHeapArray.clear();
      iv_vecShortHeapArray.clear();
      iv_vecLongHeapArray.clear();


      iv_vecStringSymbolTable.clear();

      iv_vecIndexedFSs.clear();
    }


// cout << of a 64bit type causes a warning (conversion to int, loss of data)
// since we can't help the implementation of cout we surpress the warning
#if defined( _MSC_VER )
#pragma warning( push )
#pragma warning( disable : 4244 )
#endif
    template <class T>
    void printVector(ostream & os, vector<T> const & vec) {
      size_t i;
      for (i=0; i<vec.size(); ++i) {
        os << i << ": " << vec[i] << endl;
      }
    }
#if defined( _MSC_VER )
#pragma warning( pop )
#endif




// cout << of a 64bit type causes a warning (conversion to int, loss of data)
// since we can't help the implementation of cout we surpress the warning
#if defined( _MSC_VER )
#pragma warning( push )
#pragma warning( disable : 4244 )
#endif
    void SerializedCAS::print(ostream & os) const {

      os << "===============================" << endl;
      os << "Document:" << endl;
      os << getDocument() << endl;

      os << endl << "===============================" << endl;
      os << "Type Inheritance: " << endl;
      vector<TyNum> const & typeInh = getTypeInheritanceTable();
      printVector(os, typeInh);

      os << "Type Symbol Table: " << endl;
      printVector(os, getTypeSymbolTable());

      os << "Feature Definition: " << endl;
      printVector(os, getFeatureDefinitionTable());

      os << "Feature Symbol Table: " << endl;
      printVector(os, getFeatureSymbolTable());

      os << "Feature offsets:" << endl;
      printVector(os, getFeatureOffsetTable());

      os << "Index IDs:" << endl;
      printVector(os, getIndexIDTable() );

      os << "Comparator start:" << endl;
      printVector(os, getComparatorStartTable() );

      os << "Comparatordefinition:" << endl;
      printVector(os, getComparatorDefinitionTable());

      os << "IndexKind:" << endl;
      printVector(os, getIndexKindTable());


      os << endl << "===============================" << endl;
      os << "Serialized Heap:" << endl;
      vector<TyNum> const & heap = getFSHeapArray();
      size_t i;
      for (i=0; i<heap.size(); ++i) {
        os << i  << ": " << heap[i] << endl;
      }

      os << endl << "===============================" << endl;
      os << "Serialized ByteHeap:" << endl;
      vector<char> const & bheap = getByteHeapArray();

      for (i=0; i<bheap.size(); ++i) {
        os << i  << ": " << bheap[i] << endl;
      }

      os << endl << "===============================" << endl;
      os << "Serialized ShortHeap:" << endl;
      vector<short> const & sheap = getShortHeapArray();

      for (i=0; i<sheap.size(); ++i) {
        os << i  << ": " << sheap[i] << endl;
      }
      os << endl << "===============================" << endl;
      os << "Serialized LongHeap:" << endl;
      vector<INT64> const & lheap = getLongHeapArray();

      for (i=0; i<lheap.size(); ++i) {
        os << i  << ": " << lheap[i] << endl;
      }

      os << endl << "===============================" << endl;
      os << "Serialized String table:" << endl;
      vector<UnicodeStringRef> const & strings =  getStringSymbolTable();
      for (i=0; i<strings.size(); ++i) {
        os << i << ": " << strings[i] << endl;
      }

      os << endl << "===============================" << endl;
      os << "Indexed FSs:" << endl;
      vector<TyNum> const & indexedFSs = getIndexedFSs();
      for (i=0; i<indexedFSs.size(); ++i) {
        os << i << ": " << indexedFSs[i] << endl;
      }
    }
#if defined( _MSC_VER )
#pragma warning( pop )
#endif

  }
}

/* ----------------------------------------------------------------------- */



