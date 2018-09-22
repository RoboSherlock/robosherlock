/** \file internal_casserializer.cpp .
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

//#define DEBUG_VERBOSE
#include <uima/pragmas.hpp>

#include <uima/macros.h>
#include <uima/internal_casserializer.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/lowlevel_fsheap.hpp>
#include <uima/lowlevel_indexiterator.hpp>
#include <uima/result_specification.hpp>
#include <uima/casdefinition.hpp>

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


    void CASSerializer::serializeResultSpec(ResultSpecification const & resultSpec,
                                            vector<SerializedCAS::TyNum>& resultSpecTypes,
                                            vector<SerializedCAS::TyNum>& resultSpecFeatures) {
      ResultSpecification::TyTypeOrFeatureSTLSet const & tofSet = resultSpec.getTypeOrFeatureSTLSet();
      ResultSpecification::TyTypeOrFeatureSTLSet::const_iterator cit;
      for (cit = tofSet.begin(); cit != tofSet.end(); ++cit) {
        TypeOrFeature const & tof = *cit;
        if (tof.isType()) {
          Type t = tof.getType();
          assert( t.isValid() );
          resultSpecTypes.push_back( uima::internal::FSPromoter::demoteType(t) );
        } else {
          Feature f = tof.getFeature();
          assert( f.isValid() );
          resultSpecFeatures.push_back( uima::internal::FSPromoter::demoteFeature(f) );
        }
      }
    }


    bool isInterval(uima::lowlevel::TyFSType first, uima::lowlevel::TyFSType last, vector<uima::lowlevel::TyFSType> const & vec) {
      size_t i;
      for (i=0; i<vec.size(); ++i) {
        if (vec[i] != first + i) {
          return false;
        }
      }
      return true;
    }

    CASSerializer::CASSerializer(bool bCopyStrings)
        : iv_bCopyStrings(bCopyStrings) {}

    CASSerializer::~CASSerializer() {}

    UnicodeStringRef CASSerializer::createString(UChar const * cpBuf, size_t uiLen, uima::internal::SerializedCAS & rSerializedCAS) {
      UnicodeStringRef ref(cpBuf, uiLen);
      if (iv_bCopyStrings) {
        return rSerializedCAS.addString(ref);
      }
      return ref;
    }


    void CASSerializer::serializeTypeSystem(uima::internal::CASDefinition const & casDef, uima::internal::SerializedCAS & rSerializedCAS) {
      uima::lowlevel::TypeSystem const & crTypeSystem = casDef.getTypeSystem();
      UnicodeStringRef invalidUSP(rSerializedCAS.iv_emptyString.getBuffer(), rSerializedCAS.iv_emptyString.length());
      assert( invalidUSP.getBuffer() != NULL );
      assert( invalidUSP.length() == 0);

      // 1. inheritance vector
      rSerializedCAS.iv_vecTypeInheritanceTable.clear();
      size_t uiTypeNum = crTypeSystem.getNumberOfTypes() + 1;
      rSerializedCAS.iv_vecTypeInheritanceTable.resize(uiTypeNum, 0);

#ifndef NDEBUG
      vector<uima::lowlevel::TyFSType> vecTypes;
      crTypeSystem.getAllTypes(vecTypes);
      assert( isInterval(1, uiTypeNum, vecTypes) );
      assert( uiTypeNum == vecTypes.size() + 1);
#endif

      size_t i;
      assert( 0 == uima::lowlevel::TypeSystem::INVALID_TYPE );
      assert( 1 == crTypeSystem.getTopType() );
      for (i=2; i<uiTypeNum; ++i) {
        uima::lowlevel::TyFSType tyChild = (uima::lowlevel::TyFSType) i;
        uima::lowlevel::TyFSType tyParent = crTypeSystem.getParentType( tyChild );
        assert( tyParent <rSerializedCAS.iv_vecTypeInheritanceTable.size() );
        rSerializedCAS.iv_vecTypeInheritanceTable[tyChild] =  tyParent;
      }

      // 2. feature intro vector
      rSerializedCAS.iv_vecFeatureDefinitionTable.clear();
      size_t uiFeatureNum = crTypeSystem.getNumberOfFeatures() + 1;

      // leave the first three cells empty
      rSerializedCAS.iv_vecFeatureDefinitionTable.resize(3,0);

#ifndef NDEBUG
      vector<uima::lowlevel::TyFSFeature> vecFeatures;
      crTypeSystem.getAllFeatures(vecFeatures);
      assert( isInterval(1, uiFeatureNum, vecFeatures) );
      assert( uiFeatureNum == vecFeatures.size() + 1);
#endif

      assert( 0 == uima::lowlevel::TypeSystem::INVALID_FEATURE );
      for (i=1; i<uiFeatureNum; ++i) {
        uima::lowlevel::TyFSFeature tyFeat = (uima::lowlevel::TyFSFeature) i;
        UIMA_TPRINT("Adding feature with ID: " << tyFeat );
        UIMA_TPRINT("Adding feature: " << crTypeSystem.getFeatureName(tyFeat) );
        uima::lowlevel::TyFSType tyIntroType = crTypeSystem.getIntroType(tyFeat);
        uima::lowlevel::TyFSType tyRangeType = crTypeSystem.getRangeType(tyFeat);
		int tyMultiRefs = crTypeSystem.isMultipleReferencesAllowed(tyFeat) ? 1 : 0;
        rSerializedCAS.iv_vecFeatureDefinitionTable.push_back( tyIntroType );
        rSerializedCAS.iv_vecFeatureDefinitionTable.push_back( tyRangeType );
        rSerializedCAS.iv_vecFeatureDefinitionTable.push_back( tyMultiRefs );
      }

#ifndef NDEBUG
      for (i=1; i<vecFeatures.size(); ++i) {
        uima::lowlevel::TyFSFeature tyFeat = vecFeatures[i];
        uima::lowlevel::TyFSType tyIntroType = crTypeSystem.getIntroType(tyFeat);
        uima::lowlevel::TyFSType tyRangeType = crTypeSystem.getRangeType(tyFeat);
		int tyMultiRefs = crTypeSystem.isMultipleReferencesAllowed(tyFeat) ? 1 : 0;
        assert( (tyFeat*2) <rSerializedCAS.iv_vecFeatureDefinitionTable.size() );
        assert( (tyFeat*2+1) <rSerializedCAS.iv_vecFeatureDefinitionTable.size() );
        assert( rSerializedCAS.iv_vecFeatureDefinitionTable[tyFeat*3] == tyIntroType );
        assert( rSerializedCAS.iv_vecFeatureDefinitionTable[tyFeat*3+1] == tyRangeType );
        assert( rSerializedCAS.iv_vecFeatureDefinitionTable[tyFeat*3+2] == tyMultiRefs );
      }
#endif


      // 3. type string table
      rSerializedCAS.iv_vecTypeSymbolTable.resize(uiTypeNum);
      assert( rSerializedCAS.iv_vecTypeSymbolTable.size() == uiTypeNum );
      rSerializedCAS.iv_vecTypeSymbolTable[0] = invalidUSP;
      for (i=1; i<uiTypeNum; ++i) {
        icu::UnicodeString const & crTypeName = crTypeSystem.getTypeName(i);
        UnicodeStringRef pus = createString( crTypeName.getBuffer(), crTypeName.length(), rSerializedCAS);
        rSerializedCAS.iv_vecTypeSymbolTable[i] = pus;
      }

      // 4. feature string and feature offset table
      rSerializedCAS.iv_vecFeatureSymbolTable.resize(uiFeatureNum);
      assert( rSerializedCAS.iv_vecFeatureSymbolTable.size() == uiFeatureNum );
      rSerializedCAS.iv_vecFeatureOffsetTable.resize(uiFeatureNum);
      assert( rSerializedCAS.iv_vecFeatureOffsetTable.size() == uiFeatureNum );
      rSerializedCAS.iv_vecFeatureSymbolTable[0] = invalidUSP;
      rSerializedCAS.iv_vecFeatureOffsetTable[0] = 0;
      for (i=1; i<uiFeatureNum; ++i) {
        uima::lowlevel::TyFSFeature tyFeat = i;
        // string
        icu::UnicodeString const & crFeatureName = crTypeSystem.getFeatureBaseName(tyFeat);
        UnicodeStringRef pus = createString( crFeatureName.getBuffer(), crFeatureName.length(), rSerializedCAS);
        rSerializedCAS.iv_vecFeatureSymbolTable[i] = pus;

        // offset
        rSerializedCAS.iv_vecFeatureOffsetTable[i] = crTypeSystem.getFeatureOffset(tyFeat);
      }

      // 5. type priorities
      rSerializedCAS.iv_vecTypePriorityTable.resize(uiTypeNum-1);
      for (i=1; i<uiTypeNum; ++i) {
        size_t num = crTypeSystem.getTypePriorityNumber((uima::lowlevel::TyFSType) i);
        rSerializedCAS.iv_vecTypePriorityTable[num] = i;
      }

      // 6. string sub types
      vector<uima::lowlevel::TyFSType> stringSubTypes;
      crTypeSystem.getDirectSubTypes( uima::internal::gs_tyStringType,
                                      stringSubTypes );

      rSerializedCAS.iv_stringSubTypes.clear();
      for (i=0; i<stringSubTypes.size(); ++i) {
        rSerializedCAS.iv_stringSubTypes.push_back(stringSubTypes[i]);
      }
      rSerializedCAS.iv_stringSubTypeValues.clear();
      rSerializedCAS.iv_stringSubTypeValuePos.clear();
      for (i=0; i<rSerializedCAS.iv_stringSubTypes.size(); ++i) {
        size_t n = rSerializedCAS.iv_stringSubTypeValues.size();
        rSerializedCAS.iv_stringSubTypeValuePos.push_back(n);

        vector<icu::UnicodeString> const & stringValues = crTypeSystem.getStringsForStringSubtype(rSerializedCAS.iv_stringSubTypes[i]);
        size_t j;
        for (j=0; j<stringValues.size(); ++j) {
          UnicodeStringRef ref(stringValues[j]);
          rSerializedCAS.iv_stringSubTypeValues.push_back( ref );
        }
      }
      assert( rSerializedCAS.iv_stringSubTypes.size() == rSerializedCAS.iv_stringSubTypeValuePos.size() );
    }


#if defined( _MSC_VER )
// locally disable warning about conversion from 'uima::internal::SerializedCAS::TyNum' to 'const int', possible loss of data
#  pragma warning( disable: 4244 )
#endif
    void CASSerializer::serializeIndexDefinition(uima::internal::CASDefinition const & casdef, uima::internal::SerializedCAS & rSerializedCAS) {
      uima::lowlevel::IndexDefinition const & indexDef = casdef.getIndexDefinition();

      vector<uima::lowlevel::IndexDefinition::TyIndexID> vecIndexIDs;

      indexDef.getAllIndexIDs(vecIndexIDs);
      size_t uiIndexNum = vecIndexIDs.size();
      rSerializedCAS.iv_vecIndexIDTable.resize(uiIndexNum);
      rSerializedCAS.iv_vecComparatorStartTable.resize(uiIndexNum);
      rSerializedCAS.iv_vecIndexKindTable.resize(uiIndexNum);
      rSerializedCAS.iv_vecComparatorDefinitionTable.clear();

      size_t i;
      for (i=0; i<uiIndexNum; ++i) {
        uima::lowlevel::IndexDefinition::TyIndexID const & crIndexID = vecIndexIDs[i];
        rSerializedCAS.iv_vecIndexIDTable[i] = createString( crIndexID.getBuffer(),
                                               crIndexID.length(), rSerializedCAS);
        rSerializedCAS.iv_vecIndexKindTable[i] = indexDef.getIndexKind( crIndexID );
        UIMA_TPRINT("Index ID: " << crIndexID );
        // start of the next comparator definition
        // is at the end of rSerializedCAS.iv_vecComparatorDefinitionTable
        rSerializedCAS.iv_vecComparatorStartTable[i] = rSerializedCAS.iv_vecComparatorDefinitionTable.size();

        // add type of the index
        uima::lowlevel::TyFSType indexType = indexDef.getTypeForIndex(crIndexID);
        // add type of the comparator (even if the index has none)
        rSerializedCAS.iv_vecComparatorDefinitionTable.push_back( indexType );

        uima::lowlevel::IndexComparator const * pComparator = indexDef.getComparator( crIndexID );
        if ( pComparator != NULL ) {
          UIMA_TPRINT(" Index has comparator!");
          assert( pComparator->getType() == indexType );
          // serialize comparator
          vector<uima::lowlevel::TyFSFeature> const & crKeyFeatures = pComparator->getKeyFeatures();
          vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> const & crCompOps = pComparator->getComparisonOps();
          assert( crKeyFeatures.size() == crCompOps.size() );
          // add all key features
          size_t j;
          for (j=0; j<crKeyFeatures.size(); ++j) {
            rSerializedCAS.iv_vecComparatorDefinitionTable.push_back( (SerializedCAS::TyNum) crKeyFeatures[j] );
            rSerializedCAS.iv_vecComparatorDefinitionTable.push_back( (SerializedCAS::TyNum) crCompOps[j] );
          }
        }
      }
    }

#ifdef BYEBYEPTRS
    SerializedCAS::TyNum CASSerializer::adjustString(uima::lowlevel::TyHeapCell tyFeatureCell,
        TyStringMap & stringMap,
        uima::internal::SerializedCAS & rSerializedCAS) {
      UIMA_TPRINT("adjustString() entered");
      UChar* * pPointerStringRefHeap = (UChar * *) tyFeatureCell;
      UChar * puc = *pPointerStringRefHeap;
      if (puc == NULL) {
        return 0;
      }
      assert( puc != NULL);
      assert( EXISTS(puc) );
      assert( EXISTS(pPointerStringRefHeap+1) );

      SerializedCAS::TyNum iStrLen = (SerializedCAS::TyNum) * (pPointerStringRefHeap+1);

      ptrdiff_t iStringIndex = 0;

      // try to find the string
      TyStringMap::iterator it = stringMap.lower_bound( puc );
      // if not found
      if (  (it == stringMap.end()) || ( (*it).first != puc ) ) {
        iStringIndex = stringMap.size() + 1;
        // insert new one
        TyStringMap::value_type vt(puc, iStringIndex);
        stringMap.insert(it, vt);
        UIMA_TPRINT("  iStringIndex: " << iStringIndex << ", StringSymblTableSize: " << rSerializedCAS.iv_vecStringSymbolTable.size());
        assert( iStringIndex == rSerializedCAS.iv_vecStringSymbolTable.size() );
        UnicodeStringRef ustrp = createString(puc, iStrLen, rSerializedCAS);
        rSerializedCAS.iv_vecStringSymbolTable.push_back(ustrp);
      } else {
        iStringIndex = (*it).second;
      }

      return iStringIndex;
    }
#endif

    void CASSerializer::serializeFSHeapAndStringHeap(uima::CAS const & crCAS, uima::internal::SerializedCAS & rSerializedCAS) {
      uima::internal::CASImpl const & crCASImpl = uima::internal::CASImpl::promoteCAS(crCAS);
      uima::lowlevel::FSHeap const & crHeap = crCASImpl.getHeap();
      uima::lowlevel::FSHeap::TyFSHeap const & tyTempHeap = crHeap.iv_clTemporaryHeap;

      // copy the FSHeap as is (all offsets and values)
      size_t uiSegmentLength = tyTempHeap.getTopOfHeap();
      uima::lowlevel::TyHeapCell* daHeap = tyTempHeap.getHeapStart();
      rSerializedCAS.iv_vecFSHeapArray.resize(uiSegmentLength);
      // copy the heap (better way to do this?)
      for (size_t i=0; i<uiSegmentLength; i++) {
        rSerializedCAS.iv_vecFSHeapArray[i] = daHeap[i];
      }

      // fill the vector of strings from the StringRefHeap
      uima::lowlevel::FSHeap::TyStringHeap const & tyStringHeap = crHeap.iv_clTemporaryStringHeap;
      uima::lowlevel::FSHeap::TyStringRefHeap const & tyStringRefHeap = crHeap.iv_clTemporaryStringRefHeap;
      int uiStringRefLength = tyStringRefHeap.getTopOfHeap();
      int j = 1; // point at the first entry
      rSerializedCAS.iv_vecStringSymbolTable.resize(1);
      while (j < uiStringRefLength) {
        UnicodeStringRef ustrp = UnicodeStringRef( tyStringHeap.getHeapStart()+
                                 tyStringRefHeap.getHeapValue(j),
                                 (size_t) tyStringRefHeap.getHeapValue(j+1));
        rSerializedCAS.iv_vecStringSymbolTable.push_back(ustrp);
        j += 2;
      }
    }



    void CASSerializer::serializeHeaps(uima::CAS const & crCAS, uima::internal::SerializedCAS & rSerializedCAS) {

      //serialize the fs heap and string heap
      serializeFSHeapAndStringHeap(crCAS, rSerializedCAS);

      uima::internal::CASImpl const & crCASImpl = uima::internal::CASImpl::promoteCAS(crCAS);
      uima::lowlevel::FSHeap const & crHeap = crCASImpl.getHeap();

      //8 bit heap
      uima::lowlevel::FSHeap::Ty8BitHeap const & ty8BitHeap = crHeap.iv_clTemporary8BitHeap;
      size_t uiSegmentLength = ty8BitHeap.getTopOfHeap();
      char* byteHeap = ty8BitHeap.getHeapStart();
      rSerializedCAS.iv_vecByteHeapArray.resize(uiSegmentLength);
      for (size_t i=0; i<uiSegmentLength; i++) {
        rSerializedCAS.iv_vecByteHeapArray[i] = byteHeap[i];
      }

      //16 bit heap
      uima::lowlevel::FSHeap::Ty16BitHeap const & ty16BitHeap = crHeap.iv_clTemporary16BitHeap;
      uiSegmentLength = ty16BitHeap.getTopOfHeap();
      short* shortHeap = ty16BitHeap.getHeapStart();
      rSerializedCAS.iv_vecShortHeapArray.resize(uiSegmentLength);
      for (size_t i=0; i<uiSegmentLength; i++) {
        rSerializedCAS.iv_vecShortHeapArray[i] = shortHeap[i];
      }

      //64 bit heap
      uima::lowlevel::FSHeap::Ty64BitHeap const & ty64BitHeap = crHeap.iv_clTemporary64BitHeap;
      uiSegmentLength = ty64BitHeap.getTopOfHeap();
      INT64* longHeap = ty64BitHeap.getHeapStart();
      rSerializedCAS.iv_vecLongHeapArray.resize(uiSegmentLength);
      for (size_t i=0; i<uiSegmentLength; i++) {
        rSerializedCAS.iv_vecLongHeapArray[i] = longHeap[i];
      }



    }






    //---------------------------------------------------------------------
    // Indexed FS Format
    //
    // Element Size     Number of     Description
    //   (bytes)        Elements
    // ------------     ---------     --------------------------------
    //      4               1         Number of Views in this CAS
    //      4               1         Number of Sofas in base Index Repository = nBase
    //      4             nBase       TyFS array
    //
    //   For each View:
    //      4               1         Number of FS in sofa Index Repository = nFS
    //      4             nFS         TyFS array
    //
    //---------------------------------------------------------------------


    void CASSerializer::serializeIndexedFSs(uima::CAS & crCAS,
                                            vector<uima::internal::SerializedCAS::TyNum> & iv_vecIndexedFSs) {

      uima::internal::CASImpl & crCASImpl = uima::internal::CASImpl::promoteCAS(crCAS);

      int numViews = crCAS.getBaseCas()->iv_sofaCount;
      iv_vecIndexedFSs.clear();
      iv_vecIndexedFSs.push_back(numViews);

      uima::lowlevel::IndexRepository * crIndexRep =
        (uima::lowlevel::IndexRepository*)&crCASImpl.getBaseIndexRepository();

      for (int view=0; view<=numViews; view++) {
        vector<SerializedCAS::TyNum> perLoopIndexedFSs;
        vector<uima::lowlevel::TyFSType> vecAllTypes;
        perLoopIndexedFSs.clear();
        if (view==0) {
          // First time through is for base CAS index
          // FS returned should only be for SofaFS!
          crIndexRep->getUsedIndexes(vecAllTypes);
        } else {
          // for all views found in the CAS, get new IndexRepository
          crIndexRep = crCASImpl.iv_baseCas->iv_sofa2indexMap[view];
          if (crIndexRep == 0) {
            // no indexed FS for this View, move on
            iv_vecIndexedFSs.push_back(0);
            continue;
          }
          crIndexRep->getUsedIndexes(vecAllTypes);

          //serialize the undefined index FSs
          for (size_t i=0;i < crIndexRep->iv_undefinedindex.size(); i++ ) {
            SerializedCAS::TyNum tyFSHeapIndex = (SerializedCAS::TyNum) crIndexRep->iv_undefinedindex[i];
            perLoopIndexedFSs.push_back(tyFSHeapIndex);
          }
        }

        // serialize index per type
        if ( 0 == vecAllTypes.size() && 0 == perLoopIndexedFSs.size() ) {
          // no indexed FS for this View, move on
          iv_vecIndexedFSs.push_back(0);
          continue;
        }
        for (size_t i=0; i<vecAllTypes.size(); ++i) {
          vector<uima::lowlevel::internal::SingleIndex*> const & crSingleIndexes =
            crIndexRep->getAllSingleIndexesForType(vecAllTypes[i]);
          for (size_t j=0; j<crSingleIndexes.size(); ++j) {
            auto_ptr<uima::lowlevel::IndexIterator> apIt(crSingleIndexes[j]->createIterator());
            for (apIt->moveToFirst(); apIt->isValid(); apIt->moveToNext()) {
              uima::lowlevel::TyHeapCell pHeapCell = (uima::lowlevel::TyHeapCell) apIt->get();
              SerializedCAS::TyNum tyFSHeapIndex =  (SerializedCAS::TyNum) pHeapCell;
              perLoopIndexedFSs.push_back( tyFSHeapIndex );
            }
          }
        }

        // eliminate duplicates
        sort(perLoopIndexedFSs.begin(), perLoopIndexedFSs.end());
        vector<SerializedCAS::TyNum>::iterator end = unique(perLoopIndexedFSs.begin(), perLoopIndexedFSs.end());
        // append indexedFSs from this loop
        iv_vecIndexedFSs.push_back(end - perLoopIndexedFSs.begin());
        iv_vecIndexedFSs.insert(iv_vecIndexedFSs.end(),
                                perLoopIndexedFSs.begin(),
                                end);
      }
    }


    /* no more document in de CAS
          void CASSerializer::serializeDocument(uima::TCAS const & crCAS, uima::internal::SerializedCAS & rSerializedCAS) {
             uima::internal::TCASImpl const & crTCASImpl = uima::internal::TCASImpl::promoteCAS(crCAS);
             rSerializedCAS.iv_ulstrDocument = crTCASImpl.getDocumentText();
          }
    */

    void CASSerializer::serializeDefinitions(uima::internal::CASDefinition const & casDef, uima::internal::SerializedCAS & rSerializedCAS) {
      serializeTypeSystem(casDef, rSerializedCAS);
      serializeIndexDefinition(casDef, rSerializedCAS);
    }


#ifdef UIMA_ENABLE_SERIALIZATION_TIMING
#define UIMA_SERIALIZATION_TIMING(x) x
#else
#define UIMA_SERIALIZATION_TIMING(x)
#endif

    void CASSerializer::serializeData(uima::CAS & crCAS, uima::internal::SerializedCAS & rSerializedCAS) {
//         serializeDocument(crCAS, rSerializedCAS);
      // serialize indexed FSs first so that the docAnnot can be created if necessary
      UIMA_TPRINT("Serializing indexed FSs");
      UIMA_SERIALIZATION_TIMING( iv_timerIndexedFSs.reset() );
      UIMA_SERIALIZATION_TIMING( iv_timerIndexedFSs.start() );
      serializeIndexedFSs(*crCAS.getBaseCas(), rSerializedCAS.iv_vecIndexedFSs);
      UIMA_SERIALIZATION_TIMING( iv_timerIndexedFSs.stop() );
      UIMA_TPRINT("indexed FSs serialized");

      UIMA_TPRINT("serializing all heaps");
      UIMA_SERIALIZATION_TIMING( iv_timerFSHeap.reset() );
      UIMA_SERIALIZATION_TIMING( iv_timerFSHeap.start() );
      //serializeFSHeapAndStringHeap(*crCAS.getBaseCas(), rSerializedCAS);
      serializeHeaps(*crCAS.getBaseCas(), rSerializedCAS);
      UIMA_SERIALIZATION_TIMING( iv_timerFSHeap.stop() );
      UIMA_TPRINT("FS heap serialized");
    }

    //---------------------------------------------------------------------
    // Blob Format
    //
    // Element Size     Number of     Description
    //   (bytes)        Elements
    // ------------     ---------     --------------------------------
    //      4               1         Blob key = "UIMA" in utf-8
    //      4               1         Version (currently = 1)
    //      4               1         size of 32-bit FS Heap array = s32H
    //      4             s32H        32-bit FS heap array
    //      4               1         size of 16-bit string Heap array = sSH
    //      2              sSH        16-bit string heap array
    //      4               1         size of string Ref Heap array = sSRH
    //      4             2*sSRH      string ref offsets and lengths
    //      4               1         size of FS index array = sFSI
    //      4             sFSI        FS index array
    //      4               1         size of 8-bit Heap array = s8H
    //      1              s8H        8-bit Heap array
    //      4               1         size of 16-bit Heap array = s16H
    //      2             s16H        16-bit Heap array
    //      4               1         size of 64-bit Heap array = s64H
    //      8             s64H        64-bit Heap array
    //---------------------------------------------------------------------


    // estimate total size of serialized CAS data
    size_t CASSerializer::getBlobSize(uima::CAS & crCAS) {

      // create STL vector of indexed FS so that we can size the output
      UIMA_SERIALIZATION_TIMING( iv_timerIndexedFSs.reset() );
      UIMA_SERIALIZATION_TIMING( iv_timerIndexedFSs.start() );
      serializeIndexedFSs(*crCAS.getBaseCas(), iv_vecIndexedFSs);
      UIMA_SERIALIZATION_TIMING( iv_timerIndexedFSs.stop() );

      // get a heap of references
      uima::internal::CASImpl const & crCASImpl = uima::internal::CASImpl::promoteCAS(crCAS);
      uima::lowlevel::FSHeap const & crHeap = crCASImpl.getHeap();
      uima::lowlevel::FSHeap::TyFSHeap const & tyTempHeap = crHeap.iv_clTemporaryHeap;
      uima::lowlevel::FSHeap::TyStringHeap const & tyStringHeap = crHeap.iv_clTemporaryStringHeap;
      uima::lowlevel::FSHeap::TyStringRefHeap const & tyStringRefHeap = crHeap.iv_clTemporaryStringRefHeap;
      uima::lowlevel::FSHeap::Ty8BitHeap const & ty8BitHeap = crHeap.iv_clTemporary8BitHeap;
      uima::lowlevel::FSHeap::Ty16BitHeap const & ty16BitHeap = crHeap.iv_clTemporary16BitHeap;
      uima::lowlevel::FSHeap::Ty64BitHeap const & ty64BitHeap = crHeap.iv_clTemporary64BitHeap;

      size_t uiFSHeapLength = tyTempHeap.getTopOfHeap();
      size_t uiStringHeapLength = tyStringHeap.getTopOfHeap();
      size_t uialignedStrLen = 2 * ((uiStringHeapLength + 1)/2);
      size_t uiRefHeapLength = tyStringRefHeap.getTopOfHeap();
      size_t uiIndexedFSLength = iv_vecIndexedFSs.size();
      size_t ui8BitHeapLength = ty8BitHeap.getTopOfHeap();
      size_t uialigned8BitHeapLength = 4 * ((ui8BitHeapLength+3)/4);
      size_t ui16BitHeapLength = ty16BitHeap.getTopOfHeap();
      size_t uialigned16BitHeapLength = 2 * ((ui16BitHeapLength+1)/2);
      size_t ui64BitHeapLength = ty64BitHeap.getTopOfHeap();



      size_t blobSize = 2*4               // key and version
                        + (1 + uiFSHeapLength) * 4        // FSHeap length and data
                        + 1*4 + (uialignedStrLen * 2)  // StringHeap length and data
                        + (1 + uiRefHeapLength) * 4     // StringRefheap length and data
                        + (1 + uiIndexedFSLength) * 4    // Indexed FS length and data
                        + (1*4 + uialigned8BitHeapLength)  // 8 Bit Heap length and data
                        + (1*4 + uialigned16BitHeapLength*2 ) //16 Bit Heap length and data
                        + (1*4 + ui64BitHeapLength*8 );        //64 Bit Heap length and data
      return blobSize;
    }

    // serialize CAS data into single blob format
    size_t CASSerializer::getBlob(uima::CAS & crCAS, void * buffer, size_t maxSize) {

      UIMA_SERIALIZATION_TIMING( iv_timerFSHeap.reset() );
      UIMA_SERIALIZATION_TIMING( iv_timerFSHeap.start() );

      // get a heap of references
      uima::internal::CASImpl const & crCASImpl = uima::internal::CASImpl::promoteCAS(crCAS);
      uima::lowlevel::FSHeap const & crHeap = crCASImpl.getHeap();
      uima::lowlevel::FSHeap::TyFSHeap const & tyTempHeap = crHeap.iv_clTemporaryHeap;
      uima::lowlevel::FSHeap::TyStringHeap const & tyStringHeap = crHeap.iv_clTemporaryStringHeap;
      uima::lowlevel::FSHeap::TyStringRefHeap const & tyStringRefHeap = crHeap.iv_clTemporaryStringRefHeap;
      uima::lowlevel::FSHeap::Ty8BitHeap const & ty8BitHeap = crHeap.iv_clTemporary8BitHeap;
      uima::lowlevel::FSHeap::Ty16BitHeap const & ty16BitHeap = crHeap.iv_clTemporary16BitHeap;
      uima::lowlevel::FSHeap::Ty64BitHeap const & ty64BitHeap = crHeap.iv_clTemporary64BitHeap;

      size_t uiFSHeapLength = tyTempHeap.getTopOfHeap();
      size_t uiStringHeapLength = tyStringHeap.getTopOfHeap();
      size_t uialignedStrLen = 2 * ((uiStringHeapLength + 1)/2);
      size_t uiRefHeapLength = tyStringRefHeap.getTopOfHeap();
      size_t uiIndexedFSLength = iv_vecIndexedFSs.size();

      size_t ui8BitHeapLength = ty8BitHeap.getTopOfHeap();
      size_t uialigned8BitHeapLength = 4 * ((ui8BitHeapLength+3)/4);
      size_t ui16BitHeapLength = ty16BitHeap.getTopOfHeap();
      size_t uialigned16BitHeapLength = 2 * ((ui16BitHeapLength+1)/2);
      size_t ui64BitHeapLength = ty64BitHeap.getTopOfHeap();

      size_t blobSize = 2*4               // key and version
                        + (1 + uiFSHeapLength) * 4        // FSHeap length and data
                        + 1*4 + (uialignedStrLen * 2)  // StringHeap length and data
                        + (1 + uiRefHeapLength) * 4     // StringRefheap length and data
                        + (1 + uiIndexedFSLength) * 4    // Indexed FS length and data
                        + (1*4 + uialigned8BitHeapLength)  // 8 Bit Heap length and data
                        + (1*4 + uialigned16BitHeapLength*2 ) //16 Bit Heap length and data
                        + (1*4 + ui64BitHeapLength*8 );        //64 Bit Heap length and data


      if (blobSize > maxSize) {
        return 0;             // can't serialize into given buffer
      }

      // copy all data into the blob buffer
      int* intPtr = (int*) buffer;

#if defined(WORDS_BIGENDIAN)
      char key[] = "UIMA";
#else
      char key[] = "AMIU";
#endif
      int version = 1;
      intPtr[0] = ((int*)key)[0];
      intPtr[1] = version;
      intPtr[2] = uiFSHeapLength;
      assert (blobSize > (size_t)((intPtr + 3 + uiFSHeapLength) - (int*)buffer));
      memcpy(intPtr+3, tyTempHeap.getHeapStart(), 4*uiFSHeapLength);
      intPtr += 3 + uiFSHeapLength;

      intPtr[0] = uialignedStrLen;
      assert (blobSize > (size_t)((intPtr + 1 + uiStringHeapLength/2) - (int*)buffer));
      memcpy(intPtr+1, tyStringHeap.getHeapStart(), 2*uiStringHeapLength);
      intPtr += 1 + uialignedStrLen/2;

      intPtr[0] = uiRefHeapLength;
      assert (blobSize > (size_t)((intPtr + 1 + uiRefHeapLength) - (int*)buffer));
      memcpy(intPtr+1, tyStringRefHeap.getHeapStart(), 4*uiRefHeapLength);
      intPtr += 1 + uiRefHeapLength;

      intPtr[0] = uiIndexedFSLength;
      assert (blobSize >= (size_t)((intPtr + 1 + uiIndexedFSLength) - (int*)buffer));
      memcpy(intPtr+1, &iv_vecIndexedFSs[0], 4*uiIndexedFSLength);
      intPtr += 1 + uiIndexedFSLength;

      intPtr[0] = uialigned8BitHeapLength;
      assert (blobSize > (size_t)((intPtr + 1 + uialigned8BitHeapLength/4) - (int*)buffer));
      memcpy(intPtr+1, ty8BitHeap.getHeapStart(), ui8BitHeapLength);
      intPtr += 1 + uialigned8BitHeapLength/4;

      intPtr[0] = uialigned16BitHeapLength;
      assert (blobSize > (size_t)((intPtr + 1 + ui16BitHeapLength/2) - (int*)buffer));
      memcpy(intPtr+1, ty16BitHeap.getHeapStart(), 2*ui16BitHeapLength);
      intPtr += 1 + uialigned16BitHeapLength/2;

      intPtr[0] = ui64BitHeapLength;
      assert (blobSize > (size_t)((intPtr + 1 + ui64BitHeapLength*2) - (int*)buffer));
      memcpy(intPtr+1, ty64BitHeap.getHeapStart(), 8*ui64BitHeapLength);

      UIMA_SERIALIZATION_TIMING( iv_timerFSHeap.stop() );
      return blobSize;
    }




  }

}


/* ----------------------------------------------------------------------- */



