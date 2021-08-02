/** \file internal_casdeserializer.cpp .
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

#include <uima/internal_casdeserializer.hpp>
#include <uima/macros.h>
#include <uima/lowlevel_fsheap.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/internal_serializedcas.hpp>
#include <uima/result_specification.hpp>
#include <uima/stltools.hpp>
#include <uima/msg.h>
#include <uima/casdefinition.hpp>
#include <uima/lowlevel_indexcomparator.hpp>
#include <uima/lowlevel_indexdefinition.hpp>
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

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

#ifdef ENABLE_RUNTIME_CHECKS
#define CHECK(x) if ( !(x) ) { \
   uima::ErrorMessage msg(UIMA_MSG_ID_EXC_WRONG_DESERIALIZED_DATA, __LINE__); \
   UIMA_EXC_THROW_NEW(DeserializationException, UIMA_ERR_WRONG_DESERIALIZED_DATA, msg, UIMA_MSG_ID_EXCON_DESERIALIZING_CAS, uima::ErrorInfo::unrecoverable); \
}
#else
#define CHECK(x) assert(x)
#endif
using namespace std;
namespace uima {
  namespace internal {

    UIMA_EXC_CLASSIMPLEMENT(DeserializationException, Exception);

    void CASDeserializer::deserializeResultSpecification(vector<SerializedCAS::TyNum> const & rsTypes,
        vector<SerializedCAS::TyNum> const & rsFeatures,
        uima::internal::CASDefinition const & casDef,
        ResultSpecification & result) {
      size_t i;
      for (i=0; i<rsTypes.size(); ++i) {
        assert( rsTypes[i] != uima::lowlevel::TypeSystem::INVALID_TYPE );
        uima::Type t = uima::internal::FSPromoter::promoteType(rsTypes[i], casDef.getTypeSystem());
        assert( t.isValid() );
        result.add( t );
      }
      for (i=0; i<rsFeatures.size(); ++i) {
        result.add(uima::internal::FSPromoter::promoteFeature(rsFeatures[i], casDef.getTypeSystem() ) );
      }

    }


    CASDeserializer::CASDeserializer() {}

    CASDeserializer::~CASDeserializer() {}


    void CASDeserializer::checkFeatureOffsets(uima::internal::SerializedCAS const & crSerializedCAS,
        uima::internal::CASDefinition const & casDef) {
      uima::lowlevel::TypeSystem const & crTypeSystem = casDef.getTypeSystem();
      vector<SerializedCAS::TyNum> const & crFeatureOffsetTable = crSerializedCAS.getFeatureOffsetTable();

      vector<uima::lowlevel::TyFSFeature> allFeatures;
      crTypeSystem.getAllFeatures(allFeatures);

      size_t i;
      for (i=0; i<allFeatures.size(); ++i) {
        uima::lowlevel::TyFSFeature tyFeat = allFeatures[i];
        uima::lowlevel::TyFeatureOffset tyOffset = crTypeSystem.getFeatureOffset( tyFeat );
        assert( tyFeat < crFeatureOffsetTable.size() );
        if ( tyOffset != crFeatureOffsetTable[tyFeat] ) {
#ifndef NDEBUG
          cerr << __FILE__ << "WARNING: Feature offsets are not correct!" << endl;
#endif
          assert(false);
        }
      }

    }


    void printStringVector(vector<UnicodeStringRef>const & v) {
      cout << __FILE__ <<__LINE__ << ": vector length: " << v.size() << endl;
      for (size_t i=0; i<v.size(); ++i) {
        cout << __FILE__ <<__LINE__ << ": " << i << "th element: " << v[i] << endl;
      }
    }
    void printTypeVector(vector<uima::lowlevel::TyFSType>const & v , uima::lowlevel::TypeSystem const & ts) {
      cout << __FILE__ <<__LINE__ << ": vector length: " << v.size() << endl;
      for (size_t i=0; i<v.size(); ++i) {
        cout << __FILE__ <<__LINE__ << ": " << i << "th element: ";
        if (ts.isValidType(v[i]) ) {
          cout << ts.getTypeName( v[i] ) << endl;
        } else {
          cout << "INVALIDTYPE" << endl;
        }
      }
    }

    bool CASDeserializer::isTypeSystemMergable(uima::internal::SerializedCAS const & crSerializedCAS,
        uima::internal::CASDefinition const & casDef) {
      uima::lowlevel::TypeSystem const & crTypeSystem = casDef.getTypeSystem();

      vector<UnicodeStringRef> const & crTypeSymbolTable = crSerializedCAS.getTypeSymbolTable();
      vector<UnicodeStringRef> const & crFeatureSymbolTable = crSerializedCAS.getFeatureSymbolTable();
      vector<SerializedCAS::TyNum> const & crTypeInhTable =  crSerializedCAS.getTypeInheritanceTable();
      vector<SerializedCAS::TyNum> const & crFeatureDefTable = crSerializedCAS.getFeatureDefinitionTable();

      // this method only succeeds if merging is possible, i.e., if rCAS contains
      // a "subset" of the type system in crSerializedCAS
      vector<uima::lowlevel::TyFSType> allTypes;
      crTypeSystem.getAllTypes(allTypes);

      vector<uima::lowlevel::TyFSFeature> allFeatures;
      crTypeSystem.getAllFeatures(allFeatures);

      if (!( allTypes.size() <= ( crTypeSymbolTable.size()-1) ) ) {
        return false;
      }


      // check that all types in the serialized CAS exist in the CAS by name
      size_t i;
      for (i=0; i<allTypes.size(); ++i) {
        // check type name
        lowlevel::TyFSType tyType = allTypes[i];
        uima::UnicodeStringRef typeName(crTypeSystem.getTypeName(tyType));
        int iIndex = findIndex(crTypeSymbolTable, typeName);
        if (!( iIndex == tyType ) ) {
          return false;
        }
      }

      // check that inheritance is compatible
      for (i=0; i<allTypes.size(); ++i) {
        uima::lowlevel::TyFSType tyChild = allTypes[i];
        uima::lowlevel::TyFSType tyParent = crTypeSystem.getParentType(tyChild);
        if (tyParent != uima::lowlevel::TypeSystem::INVALID_TYPE) {
          if (!( tyChild < crTypeInhTable.size() )) {
            return false;
          }
          if (!( tyParent == crTypeInhTable[tyChild] )) {
            return false;
          }
        } else {
          assert( tyChild == crTypeSystem.getTopType() );
        }
      }

      // check features names
#ifdef DEBUG_VERBOSE
      for (i=0; i<crFeatureSymbolTable.size(); ++i) {
        UIMA_TPRINT("Feature name " << i << ": " << crFeatureSymbolTable[i]);
      }
#endif
      for (i=0; i<allFeatures.size(); ++i) {
        uima::lowlevel::TyFSFeature tyFeat = allFeatures[i];
        uima::UnicodeStringRef featureName( crTypeSystem.getFeatureBaseName(tyFeat) );
        UIMA_TPRINT("Looking for feature: " << featureName);
        int iIndex = findIndex(crFeatureSymbolTable, featureName);
        if (!( iIndex != -1 )) {
          return false;
        }
        if (!( iIndex != 0 )) {
          return false;
        }
      }


      assert( crFeatureDefTable.size() %2 == 0 );
      if ( crFeatureDefTable.size() > 0 ) {
        assert( crFeatureDefTable[0] == uima::lowlevel::TypeSystem::INVALID_TYPE );
        assert( crFeatureDefTable[1] == uima::lowlevel::TypeSystem::INVALID_TYPE );
      }
      // check feature definition
      UIMA_TPRINT("Checking feature definitions");
      UIMA_TPRINT("Feature def table:");
#ifdef DEBUG_VERBOSE
      for (i=2; i<crFeatureDefTable.size(); i+=2) {
        if (crTypeSystem.isValidFeature(i/2) ) {
          UIMA_TPRINT(" feature: " << (i/2) << " (" << crTypeSystem.getFeatureName(i/2) << ") "
                      << ", intro type: " << crFeatureDefTable[i] << "(" << crTypeSystem.getTypeName(crFeatureDefTable[i]) << ") "
                      << ", range type: " << crFeatureDefTable[i+1] << "(" << crTypeSystem.getTypeName(crFeatureDefTable[i+1]) << ") ");
        }
      }
#endif

      for (i=0; i<allFeatures.size(); ++i) {
        uima::lowlevel::TyFSFeature tyFeat = allFeatures[i];
        uima::lowlevel::TyFSType tyIntroType = crTypeSystem.getIntroType(tyFeat);
        uima::lowlevel::TyFSType tyRangeType = crTypeSystem.getRangeType(tyFeat);

        UIMA_TPRINT("Checking feature: " << crTypeSystem.getFeatureName(tyFeat) );
        UIMA_TPRINT("    intro type: " << crTypeSystem.getTypeName(tyIntroType) );
        UIMA_TPRINT("    range type: " << crTypeSystem.getTypeName(tyRangeType) );

        assert( (tyFeat*2) < crFeatureDefTable.size());
        assert( (tyFeat*2+1) < crFeatureDefTable.size());

        UIMA_TPRINT("      intro type in serialized CAS: " << crTypeSystem.getTypeName(crFeatureDefTable[tyFeat*2]) );
        UIMA_TPRINT("      range type in serialized CAS: " << crTypeSystem.getTypeName(crFeatureDefTable[tyFeat*2+1]) );

        if (!( crFeatureDefTable[tyFeat*2] == tyIntroType )) {
          return false;
        }
        if (!( crFeatureDefTable[tyFeat*2+1] == tyRangeType )) {
          return false;
        }
      }

      return true;
    }

    void CASDeserializer::addTypePriorities(uima::internal::SerializedCAS const & crSerializedCAS,
                                            uima::internal::CASDefinition & casDef) {
      uima::lowlevel::TypeSystem & rTypeSystem = casDef.getTypeSystem();
      vector<SerializedCAS::TyNum> const & crTypePrioTable =  crSerializedCAS.getTypePriorityTable();
      size_t i;
      for (i=0; i<crTypePrioTable.size()-1; ++i) {
        uima::lowlevel::TyFSType t1 = (uima::lowlevel::TyFSType) crTypePrioTable[i];
        uima::lowlevel::TyFSType t2 = (uima::lowlevel::TyFSType) crTypePrioTable[i+1];
        rTypeSystem.addTypePriority(t1, t2);
      }
    }


    // this method assumes that checkTypeSystemMergable() ran successfully
    void CASDeserializer::createNewTypesAndFeatures(uima::internal::SerializedCAS const & crSerializedCAS,
        uima::internal::CASDefinition & casDef) {
      static char const * cpszCREATOR_ID_CAS_DESERIALIZER = "CASDeserializer";
      static icu::UnicodeString ustrCREATOR_ID_CAS_DESERIALIZER(cpszCREATOR_ID_CAS_DESERIALIZER);

      uima::lowlevel::TypeSystem & rTypeSystem = casDef.getTypeSystem();

      vector<UnicodeStringRef> const & crTypeSymbolTable = crSerializedCAS.getTypeSymbolTable();
      vector<UnicodeStringRef> const & crFeatureSymbolTable = crSerializedCAS.getFeatureSymbolTable();
      vector<SerializedCAS::TyNum> const & crTypeInhTable =  crSerializedCAS.getTypeInheritanceTable();
      vector<SerializedCAS::TyNum> const & crFeatureDefTable = crSerializedCAS.getFeatureDefinitionTable();
      vector<SerializedCAS::TyNum> const & crFeatureOffsetTable = crSerializedCAS.getFeatureOffsetTable();

      vector<SerializedCAS::TyNum> const & stringSubTypes = crSerializedCAS.getStringSubTypes();
      vector<SerializedCAS::TyNum> const & stringSubTypeValuePos = crSerializedCAS.getStringSubTypeValuePos();
      vector<UnicodeStringRef> const & stringSubTypeValues = crSerializedCAS.getStringSubTypeValues();

      // create all new types
      vector<uima::lowlevel::TyFSType> allTypes;
      rTypeSystem.getAllTypes(allTypes);
      size_t uiNewTypeStart = allTypes.size() + 1;
      assert( allTypes[allTypes.size() - 1] == uiNewTypeStart-1 );

      size_t i;
      // for each new type
      for (i=uiNewTypeStart; i<crTypeSymbolTable.size(); ++i) {
        uima::lowlevel::TyFSType tyParent = uima::lowlevel::TypeSystem::INVALID_TYPE;
        assert( i< crTypeInhTable.size() );
        tyParent = crTypeInhTable[i];
        // and create type
        UnicodeStringRef uref = crTypeSymbolTable[i];
        icu::UnicodeString ustrTypeName( uref.getBuffer(), uref.length() );
        UIMA_TPRINT("Creating type: " << ustrTypeName);
        assert( tyParent != uima::lowlevel::TypeSystem::INVALID_TYPE );
        // if string sub type
        if (tyParent == uima::internal::gs_tyStringType) {

          int n = findIndex( stringSubTypes, (uima::internal::SerializedCAS::TyNum) i );
          CHECK( n >= 0 && n < (int)stringSubTypes.size() );

          size_t m;
          if (n == (stringSubTypes.size() -1) ) {
            m = stringSubTypeValues.size();
          } else {
            m = stringSubTypeValuePos[n+1];
          }

          vector<icu::UnicodeString> stringValues;
          size_t j;
          for (j=stringSubTypeValuePos[n]; j<m; ++j) {
            icu::UnicodeString us(stringSubTypeValues[j].getBuffer(),
                                  stringSubTypeValues[j].length() );
            stringValues.push_back( us );
          }
          rTypeSystem.createStringSubtype(ustrTypeName,
                                          stringValues,
                                          ustrCREATOR_ID_CAS_DESERIALIZER);
        } else {
          rTypeSystem.createTypeNoChecks(tyParent, ustrTypeName, ustrCREATOR_ID_CAS_DESERIALIZER);
        }
      }

      // create all new features
      vector<uima::lowlevel::TyFSFeature> allFeatures;
      rTypeSystem.getAllFeatures(allFeatures);
      size_t uiNewFeatureStart = allFeatures.size() + 1;
      assert( allFeatures[allFeatures.size() - 1] == uiNewFeatureStart-1 );

      // for each new feature
      for (i=uiNewFeatureStart; i<crFeatureSymbolTable.size(); ++i) {
        assert( (3*i+2) < crFeatureDefTable.size() );
        uima::lowlevel::TyFSType tyIntro = crFeatureDefTable[3*i];
        uima::lowlevel::TyFSType tyRange = crFeatureDefTable[3*i+1];
        bool multiRefs = crFeatureDefTable[3*i+2] == 1;
        UnicodeStringRef uref = crFeatureSymbolTable[i];
        icu::UnicodeString ustrFeatureName( uref.getBuffer(), uref.length() );
        UIMA_TPRINT("Creating feature: " << ustrFeatureName);
        rTypeSystem.createFeature(tyIntro, tyRange, multiRefs, ustrFeatureName, ustrCREATOR_ID_CAS_DESERIALIZER);
      }


    }

    void CASDeserializer::deserializeTypeSystem(uima::internal::SerializedCAS const & crSerializedCAS,
        uima::internal::CASDefinition & casDef) {
      casDef.getTypeSystem().reset();
      createNewTypesAndFeatures(crSerializedCAS, casDef);
      addTypePriorities(crSerializedCAS, casDef);
      casDef.getTypeSystem().commit();
    }


    void CASDeserializer::deserializeIndexDefinitions(uima::internal::SerializedCAS const & crSerializedCAS,
        uima::internal::CASDefinition & casDef) {

      uima::lowlevel::IndexDefinition & rIndexRep = casDef.getIndexDefinition();
      rIndexRep.reset();

      vector<UnicodeStringRef> const & crIndexIDTable = crSerializedCAS.getIndexIDTable();
      vector<uima::internal::SerializedCAS::TyNum> const & crIndexKindTable = crSerializedCAS.getIndexKindTable();

      vector<uima::internal::SerializedCAS::TyNum> const & crComparatorStartTable = crSerializedCAS.getComparatorStartTable();
      vector<uima::internal::SerializedCAS::TyNum> const & crComparatorDefTable = crSerializedCAS.getComparatorDefinitionTable();

      assert( crIndexIDTable.size() == crIndexKindTable.size() );
      assert( crIndexIDTable.size() == crComparatorStartTable.size() );

      size_t i;
      for (i=0; i<crIndexIDTable.size(); ++i) {
        assert( i < crComparatorStartTable.size() );
        size_t uiCompStart = crComparatorStartTable[i];
        size_t uiCompEnd = crComparatorDefTable.size();
        if (i < crComparatorStartTable.size() - 1) {
          uiCompEnd = crComparatorStartTable[i+1];
        }

        vector<lowlevel::TyFSFeature> vecKeyFeatures;
        vector<uima::lowlevel::IndexComparator::EnKeyFeatureComp> vecComparators;

        assert( uiCompStart < crComparatorDefTable.size() );
        uima::lowlevel::TyFSType tyComparatorType = crComparatorDefTable[uiCompStart];
        size_t j=uiCompStart+1;
        while (j<uiCompEnd) {
          assert( j < crComparatorDefTable.size() - 1);
          uima::lowlevel::TyFSFeature keyFeat = crComparatorDefTable[j];
          uima::lowlevel::IndexComparator::EnKeyFeatureComp keyComp = (uima::lowlevel::IndexComparator::EnKeyFeatureComp) crComparatorDefTable[j+1];
          j += 2;
          vecKeyFeatures.push_back(keyFeat);
          vecComparators.push_back(keyComp);
        }

        icu::UnicodeString indexID(crIndexIDTable[i].getBuffer(), crIndexIDTable[i].length());
        uima::lowlevel::IndexDefinition::EnIndexKind kind = (uima::lowlevel::IndexDefinition::EnIndexKind) crIndexKindTable[i];

        // now create index
        rIndexRep.defineIndex(kind,
                              tyComparatorType,
                              vecKeyFeatures,
                              vecComparators,
                              indexID);
      }
      rIndexRep.commit();
    }


#ifdef BYEBYEPTRS
    UChar** CASDeserializer::resolveStringRef( size_t indexInDeserializedStringTable,
        uima::lowlevel::FSHeap::TyStringRefHeap const & stringRefHeap) {
      assert( indexInDeserializedStringTable != 0 );
      UChar** result = stringRefHeap.iv_pTHeap + 1 + 2*indexInDeserializedStringTable;
      return result;
    }
#endif

    void CASDeserializer::createStringTables(uima::internal::SerializedCAS const & crSerializedCAS,
        uima::CAS & rCAS) {
      UIMA_TPRINT("createStringTables started");
      uima::internal::CASImpl & rCASImpl = uima::internal::CASImpl::promoteCAS(rCAS);
      uima::lowlevel::FSHeap & rHeap = rCASImpl.getHeap();
      vector<UnicodeStringRef> const & crStringTable = crSerializedCAS.getStringSymbolTable();

      uima::lowlevel::FSHeap::TyStringRefHeap & rTempStringRefHeap = rHeap.iv_clTemporaryStringRefHeap;
      uima::lowlevel::FSHeap::TyStringHeap & tyStringHeap = rHeap.iv_clTemporaryStringHeap;

      UIMA_TPRINT("original string table size: " << crStringTable.size());
#ifdef DEBUG_VERBOSE
      size_t iVerbose;
      for (iVerbose=0; iVerbose<crStringTable.size(); ++iVerbose) {
        UIMA_TPRINT("  " << iVerbose << ": " << crStringTable[iVerbose]);
      }
#endif
      assert( crStringTable.size() >= 1 );
      assert( crStringTable[0].length() == 0 );
//         assert( crStringTable[0].getBuffer() == NULL );
      size_t uiStringTableSize = crStringTable.size();

      // reset stringRef heap to new required size
      rTempStringRefHeap.reset();
      rTempStringRefHeap.increaseHeap(2*(uiStringTableSize-1));

      // 2 cells per string, plus 1 for the 0th cell // , the 0th string of the string table is invalid

      uima::lowlevel::TyHeapCell * pStringRef = rTempStringRefHeap.getHeapStart() + 1;
      uima::lowlevel::TyHeapCell * lastCellRef = rTempStringRefHeap.getHeapStart()
          + rTempStringRefHeap.getLastHeapCell();

      size_t i;
      for (i=1; i<uiStringTableSize; ++i) {
        // copy string to StringHeap
        int uref = rHeap.addString( crStringTable[i] );
        // and add entry on the StringRefHeap
        assert( pStringRef < lastCellRef );
        (*pStringRef) = uref;
        ++pStringRef;
        (*pStringRef) = crStringTable[i].length();
        ++pStringRef;
      }

      // now the following holds:
      // crStringTable[i] is associated with the two cells at
      //   (rTempStringRefHeap.iv_pTHeap) + 1 + 2*i and
      //   (rTempStringRefHeap.iv_pTHeap) + 1 + 2*i + 1

      // check this
#ifndef NDEBUG
      for (i=1; i<uiStringTableSize; i++) {
        UnicodeStringRef uref1 = crStringTable[i];
//            UChar** pStr = rTempStringRefHeap.iv_pTHeap+1+2*i;
        UnicodeStringRef uref2 = UnicodeStringRef( tyStringHeap.getHeapStart()+
                                 rTempStringRefHeap.getHeapValue(2*i - 1),
                                 (size_t) rTempStringRefHeap.getHeapValue(2*i));
        UIMA_TPRINT("Comparing '" << uref1 << "' with '"<< uref2 << "'");
        assert( uref1 == uref2 );
      }
#endif
      UIMA_TPRINT("createStringTables finished");
    }

    void CASDeserializer::createFSHeap(uima::internal::SerializedCAS const & crSerializedCAS,
                                       uima::CAS & rCAS) {
      uima::internal::CASImpl & rCASImpl = uima::internal::CASImpl::promoteCAS(rCAS);
      /* ee: do we still need this???
             rCASImpl.refreshCachedTypes();
      */
      uima::lowlevel::FSHeap & rHeap = rCASImpl.getHeap();
      vector<SerializedCAS::TyNum> const & crFSHeapArray = crSerializedCAS.getFSHeapArray();

      assert( crFSHeapArray.size() > 0 );
      size_t uiFSHeapSize = crFSHeapArray.size();
      uima::lowlevel::FSHeap::TyFSHeap & rTempFSHeap = rHeap.iv_clTemporaryHeap;

      // reset and enlarge heap to hold new FS if necessary
      rTempFSHeap.reset();
      rTempFSHeap.increaseHeap(uiFSHeapSize-1);

      uima::lowlevel::TyHeapCell * pHeap = rTempFSHeap.iv_pTHeap;
      assert( EXISTS( pHeap ) );

      CHECK( crFSHeapArray[0] == 0 );
      assert( (*pHeap) == 0 );

      // Just copy the offsets and values into the new FSHeap
      UIMA_TPRINT("FSHeap array size: " << crFSHeapArray.size());
      for (size_t i=0; i<uiFSHeapSize; i++) {
        pHeap[i] = (lowlevel::TyHeapCell) crFSHeapArray[i];
      }
    }


    void CASDeserializer::createByteHeap(uima::internal::SerializedCAS const & crSerializedCAS,
                                         uima::CAS & rCAS) {
      uima::internal::CASImpl & rCASImpl = uima::internal::CASImpl::promoteCAS(rCAS);

      uima::lowlevel::FSHeap & rFSHeap = rCASImpl.getHeap();
      uima::lowlevel::FSHeap::Ty8BitHeap & rHeap = rFSHeap.iv_clTemporary8BitHeap;

      vector<char> const & crHeapArray = crSerializedCAS.getByteHeapArray();

      size_t uiHeapSize = crHeapArray.size();

      // reset
      rHeap.reset();

      if (uiHeapSize > 0) {
        assert( crHeapArray.size() > 0 );
        //enlarge heap to hold new FS if necessary
        rHeap.increaseHeap(uiHeapSize-1);

        char * pHeap = rHeap.iv_pTHeap;
        assert( EXISTS( pHeap ) );

        CHECK( crHeapArray[0] == 0 );
        assert( (*pHeap) == 0 );

        //copy
        UIMA_TPRINT("Heap array size: " << crHeapArray.size());
        for (size_t i=0; i<uiHeapSize; i++) {
          pHeap[i] = (char) crHeapArray[i];
        }
      }
    }



    void CASDeserializer::createShortHeap(uima::internal::SerializedCAS const & crSerializedCAS,
                                          uima::CAS & rCAS) {
      uima::internal::CASImpl & rCASImpl = uima::internal::CASImpl::promoteCAS(rCAS);

      uima::lowlevel::FSHeap & rFSHeap = rCASImpl.getHeap();
      uima::lowlevel::FSHeap::Ty16BitHeap & rHeap = rFSHeap.iv_clTemporary16BitHeap;

      vector<short> const & crHeapArray = crSerializedCAS.getShortHeapArray();

      size_t uiHeapSize = crHeapArray.size();

      // reset
      rHeap.reset();
      if (uiHeapSize > 0) {
        assert( crHeapArray.size() > 0 );
        //enlarge heap to hold new FS if necessary
        rHeap.increaseHeap(uiHeapSize-1);
        short * pHeap = rHeap.iv_pTHeap;
        assert( EXISTS( pHeap ) );

        CHECK( crHeapArray[0] == 0 );
        assert( (*pHeap) == 0 );

        //copy
        UIMA_TPRINT("Heap array size: " << crHeapArray.size());
        for (size_t i=0; i<uiHeapSize; i++) {
          pHeap[i] = (short) crHeapArray[i];
        }
      }
    }

    void CASDeserializer::createLongHeap(uima::internal::SerializedCAS const & crSerializedCAS,
                                         uima::CAS & rCAS) {
      uima::internal::CASImpl & rCASImpl = uima::internal::CASImpl::promoteCAS(rCAS);

      uima::lowlevel::FSHeap & rFSHeap = rCASImpl.getHeap();
      uima::lowlevel::FSHeap::Ty64BitHeap & rHeap = rFSHeap.iv_clTemporary64BitHeap;

      vector<INT64> const & crHeapArray = crSerializedCAS.getLongHeapArray();

      size_t uiHeapSize = crHeapArray.size();

      // reset
      rHeap.reset();
      if (uiHeapSize > 0) {
        assert( crHeapArray.size() > 0 );
        //enlarge heap to hold new FS if necessary
        rHeap.increaseHeap(uiHeapSize-1);
        INT64 * pHeap = rHeap.iv_pTHeap;
        assert( EXISTS( pHeap ) );

        CHECK( crHeapArray[0] == 0 );
        assert( (*pHeap) == 0 );

        //copy
        UIMA_TPRINT("Heap array size: " << crHeapArray.size());
        for (size_t i=0; i<uiHeapSize; i++) {
          pHeap[i] = (INT64) crHeapArray[i];
        }
      }
    }

    void CASDeserializer::deserializeIndexedFSs(vector<SerializedCAS::TyNum> & crIndexFSs,
        uima::CAS & rCAS) {

      uima::internal::CASImpl & rCASImpl = uima::internal::CASImpl::promoteCAS(rCAS);
      uima::lowlevel::FSHeap & crHeap = rCASImpl.getHeap();
      uima::lowlevel::IndexRepository * crIndexRep = &rCASImpl.getIndexRepository();
      uima::lowlevel::FSHeap::TyFSHeap const & rTempFSHeap = crHeap.iv_clTemporaryHeap;
      SerializedCAS::TyNum iMaxOffset = rTempFSHeap.getTopOfHeap();

      vector<SerializedCAS::TyNum>::const_iterator cit, loopit;
      vector<SerializedCAS::TyNum> perLoopIndexedFSs;
      cit = crIndexFSs.begin();
      int numViews = *cit++;
      int loopSize = *cit;

      crIndexRep->reset();

      // deserialize base CAS
      if (loopSize > 0) {
        lastSegmentUsed = 0;
        perLoopIndexedFSs.insert(perLoopIndexedFSs.end(), cit+1, cit+1+loopSize);
        cit += loopSize + 1;

        for (loopit = perLoopIndexedFSs.begin(); loopit != perLoopIndexedFSs.end(); ++loopit) {
          assert( *loopit < iMaxOffset );
          crIndexRep->add( *loopit );
        }
      }

      // book keeping for all Sofas
      rCAS.getBaseCas()->iv_sofaCount = 1; // reserve for initial view
      FSIndex fsIdx = crIndexRep->getIndex(CAS::INDEXID_SOFA);
      FSIterator fsIt = fsIdx.iterator();
      while (fsIt.isValid()) {
        SofaFS aSofa = (SofaFS) fsIt.get();
        if ( 0 == aSofa.getSofaID().compare(icu::UnicodeString(CAS::NAME_DEFAULT_SOFA)) ) {
          rCAS.registerInitialSofa();
        } else {
          // only bump sofa count if not initial View
          rCAS.bumpSofaCount();
        }
        rCAS.getView(aSofa)->registerView(aSofa);
        fsIt.moveToNext();
      }

      for (int view = 1; view <= numViews; view++) {

        // Check if sofa's index has anything in it
        loopSize = *cit;
        if (0 == loopSize) {
          cit++;
          continue;
        }

        CAS* tcas = rCAS.getViewBySofaNum(view);
        uima::internal::CASImpl & crTCASImpl = uima::internal::CASImpl::promoteCAS(*tcas);
        crIndexRep = &crTCASImpl.getIndexRepository();
        crIndexRep->reset();

        perLoopIndexedFSs.clear();
        perLoopIndexedFSs.insert(perLoopIndexedFSs.end(), cit+1, cit+1+loopSize);
        cit += loopSize + 1;

        for (loopit = perLoopIndexedFSs.begin(); loopit != perLoopIndexedFSs.end(); ++loopit) {
          assert( *loopit < iMaxOffset );
          crIndexRep->add( *loopit );
        }
        tcas->pickupDocumentAnnotation();
      }

    }

    void CASDeserializer::deserializeFSHeapAndStringTable(uima::internal::SerializedCAS const & crSerializedCAS,
        uima::CAS & rCAS) {
      /*
      // feature offsets can only be checked here because only here is the
      // type system committed !!!
      checkFeatureOffsets(crSerializedCAS, rCAS);
      */
      createStringTables(crSerializedCAS, rCAS);
      createFSHeap(crSerializedCAS, rCAS);
    }


    void CASDeserializer::deserializeHeapsAndStringTable(uima::internal::SerializedCAS const & crSerializedCAS,
        uima::CAS & rCAS) {
      createStringTables(crSerializedCAS, rCAS);
      createFSHeap(crSerializedCAS, rCAS);
      createByteHeap(crSerializedCAS, rCAS);
      createShortHeap(crSerializedCAS, rCAS);
      createLongHeap(crSerializedCAS, rCAS);
    }



    /*
          void CASDeserializer::deserializeDocument(uima::internal::SerializedCAS const & crSerializedCAS,
                                                    uima::TCAS & rCAS) {
             UnicodeStringRef uref = crSerializedCAS.getDocument();
             rCAS.setDocumentText( uref.getBuffer(), uref.length(), true );
          }
    */

// utilities for swapBlob

    INT64 swap8(INT64 in) {
      char output[8];
      output[0] = ((char*)&in)[7];
      output[1] = ((char*)&in)[6];
      output[2] = ((char*)&in)[5];
      output[3] = ((char*)&in)[4];
      output[4] = ((char*)&in)[3];
      output[5] = ((char*)&in)[2];
      output[6] = ((char*)&in)[1];
      output[7] = ((char*)&in)[0];
      in = *(int*)output;
      return in;
    }

    int swap4(int in) {
      char output[4];
      output[0] = ((char*)&in)[3];
      output[1] = ((char*)&in)[2];
      output[2] = ((char*)&in)[1];
      output[3] = ((char*)&in)[0];
      in = *(int*)output;
      return in;
    }

    short swap2(short in) {
      char output[2];
      output[0] = ((char*)&in)[1];
      output[1] = ((char*)&in)[0];
      in = *(short*)output;
      return in;
    }

    void intSwap(int* buff, int count) {
      for (int i=0; i<count; i++) {
        buff[i] = swap4(buff[i]);
      }
    }

    void shortSwap(short* buff, int count) {
      for (int i=0; i<count; i++) {
        buff[i] = swap2(buff[i]);
      }
    }

    void longSwap(INT64 * buff, int count) {
      for (int i=0; i<count; i++) {
        buff[i] = swap8(buff[i]);
      }
    }


// swaps blob from opposite endian order
    void CASDeserializer::swapBlob(void * buffer) {
      int *intBuff, loopcnt;

      // key, version, FSHeap-size
      intBuff = (int*)buffer;
      intSwap(intBuff, 3);
      loopcnt = intBuff[2];
      intBuff += 3;

      // FSheap
      intSwap(intBuff, loopcnt);
      intBuff += loopcnt;

      // string heap
      intSwap(intBuff, 1);
      loopcnt = intBuff[0];
      intBuff++;
      shortSwap((short*)intBuff, loopcnt);
      // assure word alignment
      intBuff += (1 + loopcnt)/2;

      // string ref heap
      intSwap(intBuff, 1);
      loopcnt = intBuff[0];
      intBuff++;
      intSwap(intBuff, loopcnt);
      intBuff += loopcnt;

      // FS index array
      intSwap(intBuff, 1);
      loopcnt = intBuff[0];
      intBuff++;
      intSwap(intBuff, loopcnt);
      intBuff += loopcnt;

      // 8 bit heap
      intSwap(intBuff, 1);
      loopcnt = intBuff[0];
      intBuff++;
      intBuff += (3 + loopcnt) / 4;

      // 16 bit heap
      intSwap(intBuff, 1);
      loopcnt = intBuff[0];
      intBuff++;
      shortSwap((short*)intBuff, loopcnt);
      // assure word alignment
      intBuff += (1 + loopcnt)/2;


      //64 bit heap
      intSwap(intBuff, 1);
      loopcnt = intBuff[0];
      intBuff++;
      longSwap((INT64*) intBuff, loopcnt);
      intBuff += loopcnt*2;

    }

// for blob deserialization
    void CASDeserializer::deserializeBlob(void * buffer, uima::CAS & rCAS) {
      vector<SerializedCAS::TyNum> iv_vecIndexedFSs;

      // check blob "key" and version
      int* intPtr = (int*) buffer;

      // Check if blob needs byteswap
#if defined(WORDS_BIGENDIAN)
      char key[] = "UIMA";
      char yek[] = "AMIU";
#else
      char key[] = "AMIU";
      char yek[] = "UIMA";
#endif
      if (intPtr[0] == ((int*)yek)[0]) {
        swapBlob(buffer);
      }

      CHECK(intPtr[0] == ((int*)key)[0]);
      CHECK(intPtr[1] == 1);

      // get a heap of references
      uima::internal::CASImpl & crCASImpl = uima::internal::CASImpl::promoteCAS(rCAS);
      uima::lowlevel::FSHeap & crHeap = crCASImpl.getHeap();
      uima::lowlevel::FSHeap::TyFSHeap & tyTempHeap = crHeap.iv_clTemporaryHeap;
      uima::lowlevel::FSHeap::TyStringHeap & tyStringHeap = crHeap.iv_clTemporaryStringHeap;
      uima::lowlevel::FSHeap::TyStringRefHeap & tyStringRefHeap = crHeap.iv_clTemporaryStringRefHeap;
      uima::lowlevel::FSHeap::Ty8BitHeap & ty8BitHeap = crHeap.iv_clTemporary8BitHeap;
      uima::lowlevel::FSHeap::Ty16BitHeap & ty16BitHeap = crHeap.iv_clTemporary16BitHeap;
      uima::lowlevel::FSHeap::Ty64BitHeap & ty64BitHeap = crHeap.iv_clTemporary64BitHeap;



      // deserialize FSHeap
      tyTempHeap.reset();
      size_t uiFSHeapLength = intPtr[2];
      if (uiFSHeapLength > 1) {
        CHECK(intPtr[3] == 0);
        tyTempHeap.increaseHeap(uiFSHeapLength-1);
        memcpy(tyTempHeap.getHeapStart(), intPtr+3, 4*uiFSHeapLength);
      }
      intPtr += 3 + uiFSHeapLength;

      // deserialize StringTable
      tyStringHeap.reset();
      size_t uiStringHeapLength = intPtr[0];
      size_t uialignedStrLen = 2 * ((uiStringHeapLength + 1)/2);
      if (uiStringHeapLength > 1) {
        CHECK(((short*)intPtr)[2] == 0); // check the first short after the length
        tyStringHeap.increaseHeap(uiStringHeapLength-1);
        memcpy(tyStringHeap.getHeapStart(), intPtr+1, 2*uiStringHeapLength);
      }
      intPtr += 1 + uialignedStrLen/2;

      // deserialize StringRef
      tyStringRefHeap.reset();
      size_t uiRefHeapLength = intPtr[0];
      if (uiRefHeapLength > 1) {
        CHECK(intPtr[1] == 0);
        tyStringRefHeap.increaseHeap(uiRefHeapLength-1);
        memcpy(tyStringRefHeap.getHeapStart(), intPtr+1, 4*uiRefHeapLength);
      }
      intPtr += 1 + uiRefHeapLength;

      // create FS indexes
      size_t uiIndexedFSLength = intPtr[0];
      if (uiIndexedFSLength > 0) {
        uima::CAS * baseCAS = rCAS.getBaseCas();
        iv_vecIndexedFSs.resize(uiIndexedFSLength);
        memcpy(&iv_vecIndexedFSs[0], intPtr+1, 4*uiIndexedFSLength);
        deserializeIndexedFSs(iv_vecIndexedFSs, *baseCAS);
      }
      intPtr += 1 + uiIndexedFSLength;

      //8bit heap
      ty8BitHeap.reset();
      size_t ui8BitHeapLength = intPtr[0];
      size_t uialigned8BitHeapLen = 4 * ((ui8BitHeapLength + 3)/4);

      if (ui8BitHeapLength > 1) {
        //CHECK(((char*)intPtr)[1] == 0);
        ty8BitHeap.increaseHeap(ui8BitHeapLength-1);
        memcpy(ty8BitHeap.getHeapStart(), intPtr+1, ui8BitHeapLength);
      }
      intPtr += 1 + uialigned8BitHeapLen/4;

      //16 bit heap
      ty16BitHeap.reset();
      size_t ui16BitHeapLength = intPtr[0];
      size_t uialigned16BitHeapLen = 2 * ((ui16BitHeapLength + 1)/2);
      if (ui16BitHeapLength > 1) {
        //CHECK(((short*)intPtr)[2] == 0);
        ty16BitHeap.increaseHeap(ui16BitHeapLength-1);
        memcpy(ty16BitHeap.getHeapStart(), intPtr+1, 2*ui16BitHeapLength);
      }
      intPtr += 1 + uialigned16BitHeapLen/2;
      //64 bit heap
      ty64BitHeap.reset();
      size_t ui64BitHeapLength = intPtr[0];
      if (ui64BitHeapLength > 0) {
        //CHECK(intPtr[1] == 0);
        ty64BitHeap.increaseHeap(ui64BitHeapLength-1);
        memcpy(ty64BitHeap.getHeapStart(), intPtr+1, 8*ui64BitHeapLength);
      }



    }

    void CASDeserializer::deserializeData(uima::internal::SerializedCAS const & crSerializedCAS,
                                          uima::CAS & rCAS) {
      uima::CAS * baseCAS = rCAS.getBaseCas();
      baseCAS->reset();
      deserializeHeapsAndStringTable( crSerializedCAS, *baseCAS );
      deserializeIndexedFSs(CONST_CAST(vector<SerializedCAS::TyNum>&,crSerializedCAS.iv_vecIndexedFSs), *baseCAS);
//         deserializeDocument(crSerializedCAS, rCAS); // do this last because only here is the document annotation created
    }


    void CASDeserializer::deserializeDefinitions(uima::internal::SerializedCAS const & crSerializedCAS,
        uima::internal::CASDefinition & casDef) {
      deserializeTypeSystem(crSerializedCAS, casDef);

      deserializeIndexDefinitions(crSerializedCAS, casDef);
      //         checkFeatureOffsets(crSerializedCAS, rCAS);
    }


  }
}


/* ----------------------------------------------------------------------- */




