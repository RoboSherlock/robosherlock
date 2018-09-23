/** \file lowlevel_indexcomparator.cpp .
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
#include <uima/lowlevel_indexcomparator.hpp>
#include <uima/lowlevel_indexdefinition.hpp>
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
  namespace lowlevel {

    IndexComparator::IndexComparator(IndexDefinition const & indexDefinition,
                                     TyFSType tyType,
                                     vector<TyFSFeature> const & crIndexs)
        : iv_indexDefinition( indexDefinition),
        iv_tyType(tyType) {
      size_t i;
      for (i=0; i<crIndexs.size(); ++i) {
        addKey(crIndexs[i], STANDARD_COMPARE);
      }
    }

    IndexComparator::IndexComparator(IndexDefinition const & indexDefinition,
                                     TyFSType tyType)
        : iv_indexDefinition( indexDefinition),
        iv_tyType(tyType) {
    }


    void IndexComparator::addKey(TyFSFeature tyFeature,
                                 EnKeyFeatureComp enComp) {
      TypeSystem const & crTypeSystem = iv_indexDefinition.getTypeSystem();
      assert( crTypeSystem.isValidType(iv_tyType) );

      // if this key is type priority
      if (tyFeature == uima::lowlevel::TypeSystem::INVALID_FEATURE) {
        iv_features.push_back(uima::lowlevel::TypeSystem::INVALID_FEATURE);
        iv_offsets.push_back(0);
        iv_appropTypes.push_back(BUILTIN_TYPE_INVALID);
      } else {
        assert( crTypeSystem.isAppropriateFeature(iv_tyType, tyFeature) );
        iv_features.push_back(tyFeature);
        iv_offsets.push_back( crTypeSystem.getFeatureOffset(tyFeature) );
        TyFSType tyAppropType = crTypeSystem.getRangeType(tyFeature);

        if (tyAppropType == crTypeSystem.getTypeByName( CAS::TYPE_NAME_INTEGER)) {
          iv_appropTypes.push_back( BUILTIN_TYPE_INTEGER );
        } else if (tyAppropType == crTypeSystem.getTypeByName( CAS::TYPE_NAME_FLOAT)) {
          iv_appropTypes.push_back( BUILTIN_TYPE_FLOAT );
        } else if (tyAppropType == crTypeSystem.getTypeByName( CAS::TYPE_NAME_STRING)) {
          iv_appropTypes.push_back( BUILTIN_TYPE_STRING );
        } else if (tyAppropType == crTypeSystem.getTypeByName( CAS::TYPE_NAME_BOOLEAN)) {
          iv_appropTypes.push_back( BUILTIN_TYPE_BOOLEAN );
        } else if (tyAppropType == crTypeSystem.getTypeByName( CAS::TYPE_NAME_BYTE)) {
          iv_appropTypes.push_back( BUILTIN_TYPE_BYTE );
        } else if (tyAppropType == crTypeSystem.getTypeByName( CAS::TYPE_NAME_SHORT)) {
          iv_appropTypes.push_back( BUILTIN_TYPE_SHORT );
        } else if (tyAppropType == crTypeSystem.getTypeByName( CAS::TYPE_NAME_LONG)) {
          iv_appropTypes.push_back( BUILTIN_TYPE_LONG );
        } else if (tyAppropType == crTypeSystem.getTypeByName( CAS::TYPE_NAME_DOUBLE)) {
          iv_appropTypes.push_back( BUILTIN_TYPE_DOUBLE);
        }
        else {
          assertWithMsg(false, "Type not supported for key features!");
        }
      }
      iv_comparators.push_back(enComp);

      assert( iv_offsets.size() == iv_appropTypes.size() );
      assert( iv_offsets.size() == iv_features.size() );
      assert( iv_offsets.size() == iv_comparators.size() );
    }

    inline int
    inline_strCompare(const UChar *s1, size_t length1,
                      const UChar *s2, size_t length2) {
      const UChar *limit1;
      UChar c1, c2;

      /* memcmp/UnicodeString style, both length-specified */
      int lengthResult;

      /* limit1=s1+min(lenght1, length2) */
      if (length1<length2) {
        lengthResult=-1;
        limit1=s1+length1;
      } else if (length1==length2) {
        lengthResult=0;
        limit1=s1+length1;
      } else { /* length1>length2 */
        lengthResult=1;
        limit1=s1+length2;
      }

      if (s1==s2) {
        return lengthResult;
      }

      for (;;) {
        /* check pseudo-limit */
        if (s1==limit1) {
          return lengthResult;
        }

        c1=*s1;
        c2=*s2;
        if (c1!=c2) {
          break;
        }
        ++s1;
        ++s2;
      }

      return(int)c1-(int)c2;
    }

    int IndexComparator::compare(uima::lowlevel::FSHeap const & heap, TyFS fs1, TyFS fs2) const {
      if (fs1 == fs2) {
        return 0;
      }
      assert( heap.getTypeSystem().subsumes( iv_tyType, heap.getType(fs1)) );
      assert( heap.getTypeSystem().subsumes( iv_tyType, heap.getType(fs2)) );
      UIMA_TPRINT("Comparing fs of type " << heap.getTypeSystem().getTypeName( heap.getType(fs1)));
      UIMA_TPRINT("   with fs of type " << heap.getTypeSystem().getTypeName( heap.getType(fs2)));
      bool isGreater = false;
      bool isLess = false;
      size_t i;
      const size_t uiOffsetsSize = iv_offsets.size();
      TyFeatureOffset utCurrentOffset;
      // for each feature
      for (i=0; i <uiOffsetsSize; ++i) {
        UIMA_TPRINT("Comparing feature " << i);
        utCurrentOffset = iv_offsets[i];
        // if type priority
        if (utCurrentOffset == 0) {
          assert( iv_features[i] == uima::lowlevel::TypeSystem::INVALID_FEATURE );

          uima::lowlevel::TyFSType t1 = heap.getType(fs1);
          uima::lowlevel::TyFSType t2 = heap.getType(fs2);

          uima::lowlevel::TypeSystem const & crTypeSystem = heap.getTypeSystem();
          if (t1 == t2) {
            isLess = false;
            isGreater = false;
          } else if (crTypeSystem.hasPriorityOver(t1, t2)) {
            isLess = true;
            isGreater = false;
          } else {
            isLess = false;
            isGreater = true;
          }
        } else {
          TyFS val1 = heap.getFeatureWithOffset(fs1, utCurrentOffset);
          TyFS val2 = heap.getFeatureWithOffset(fs2, utCurrentOffset);
          switch (iv_appropTypes[i]) {
          case BUILTIN_TYPE_INTEGER: {
            int i1 = FSHeap::getFSAsInt(val1);
            int i2 = FSHeap::getFSAsInt(val2);
            UIMA_TPRINT("  val1: " << i1 << ", val2: " << i2);
            isLess = (i1 < i2);
            isGreater = (i1 > i2);
            break;
          }
          case BUILTIN_TYPE_FLOAT: {
            float f1 = FSHeap::getFSAsFloat(val1);
            float f2 = FSHeap::getFSAsFloat(val2);
            isLess = (f1 < f2);
            isGreater = (f1 > f2);
            break;
          }
          case BUILTIN_TYPE_STRING: {
            UnicodeStringRef us1 = heap.getFSAsString(val1);
            UnicodeStringRef us2 = heap.getFSAsString(val2);
            int iCmp = inline_strCompare(us1.getBuffer(), us1.length(),
                                         us2.getBuffer(), us2.length());
            isLess    = (iCmp < 0);
            isGreater = (iCmp > 0);
            break;
          }
          case BUILTIN_TYPE_BOOLEAN: {
            bool f1 = FSHeap::getFSAsBoolean(val1);
            bool f2 =  FSHeap::getFSAsBoolean(val2);
            isLess = (f1 < f2);
            isGreater = (f1 > f2);
            break;
          }
          case BUILTIN_TYPE_BYTE: {
            char f1 = FSHeap::getFSAsByte(val1);
            char f2 =  FSHeap::getFSAsByte(val2);
            isLess = (f1 < f2);
            isGreater = (f1 > f2);
            break;
          }
          case BUILTIN_TYPE_SHORT: {
            short f1 = FSHeap::getFSAsShort(val1);
            short f2 = FSHeap::getFSAsShort(val2);
            isLess = (f1 < f2);
            isGreater = (f1 > f2);
            break;
          }
          case BUILTIN_TYPE_LONG: {
            INT64 f1 = heap.getFSAsLong(val1);
            INT64 f2 = heap.getFSAsLong(val2);
            isLess = (f1 < f2);
            isGreater = (f1 > f2);
            break;
          }
          case BUILTIN_TYPE_DOUBLE: {
            double f1 = heap.getFSAsDouble(val1);
            double f2 = heap.getFSAsDouble(val2);
            isLess = (f1 < f2);
            isGreater = (f1 > f2);
            break;
          }

          default:
//                  UIMA_TPRINT("Unsupported type: " << heap.getTypeSystem().getTypeName(appropTypes[i]));
            assertWithMsg(false, "Unsupported built-in type");
          }
        }
        UIMA_TPRINT("   isLess: " << isLess);
        UIMA_TPRINT("   isGreater: " << isGreater);
        assert( ! (isLess && isGreater) );

        switch (iv_comparators[i]) {
        case STANDARD_COMPARE: {
          if (isLess) {
            return 1;
          } else {
            if (isGreater) {
              return -1;
            }
          }
          break;
        }
        case REVERSE_STANDARD_COMPARE: {
          if (isLess) {
            return -1;
          } else {
            if (isGreater) {
              return 1;
            }
          }
          break;
        }
        default:
          assert(false);
        };
      }
      return 0;
    }


  }
}


/* ----------------------------------------------------------------------- */





