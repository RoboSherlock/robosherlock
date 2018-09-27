#ifndef UIMA_LOWLEVEL_TYPESYSTEM_HPP
#define UIMA_LOWLEVEL_TYPESYSTEM_HPP
/** \file lowlevel_typesystem.hpp .
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
#include <uima/pragmas.hpp>

#include <vector>
#include <map>

#include <uima/lowlevel_typedefs.hpp>

#include <uima/internal_typeshortcuts.hpp>
#include <uima/casexception.hpp>
#include <uima/typesystem.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace internal {
    class CASDefinition;
    class CASDeserializer;
  }
}
/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace lowlevel {

    UIMA_EXC_CLASSDECLARE(FeatureIntroductionFailedException, CASException);
    UIMA_EXC_CLASSDECLARE(TypeCreationFailedException, CASException);

    /**
     * The lowlevel type system.
     * Inherits from the type system of the OO API.
     */
    class UIMA_LINK_IMPORTSPEC TypeSystem  : public uima::TypeSystem {
      friend class uima::internal::CASDefinition;
      friend class uima::internal::CASDeserializer;
    protected:
      bool iv_bIsCommitted;
      bool iv_bBuiltinTypesCreated;

      TyFSType iv_tyTop;

      // for building the hierarchy

      // tree[t] are the daughters of type t
       
      std::vector<std::vector<TyFSType> > iv_vecTree;
      // introducedFeatures[t] are the features introduced at type t
      std::vector<std::vector<TyFSFeature> > iv_vecIntroducedFeatures;
      // useful map to parents of type t
      std::vector<TyFSType> iv_vecParents;

      // iv_vecRangeTypes[f] is the range type for feature f
      std::vector<TyFSType> iv_vecRangeTypes;
      // iv_vecMultiRefs[f] is the multipleReferencesAllowed property for feature f
      std::vector<bool> iv_vecMultiRefs;

      // names of types and features
      std::vector<icu::UnicodeString> iv_vecTypeNames;
      std::vector<icu::UnicodeString> iv_vecTypeCreatorIDs;
      std::vector<icu::UnicodeString> iv_vecFeatureBaseNames;
      std::vector<icu::UnicodeString> iv_vecFeatureCreatorIDs;

      // string sub types
      std::vector<std::vector<icu::UnicodeString> > iv_enumStrings;
      std::map<TyFSType, TyFeatureOffset> iv_stringSubtypeToStringSet;

      //  list of priority pairs
      std::vector< std::pair<TyFSType, TyFSType> > iv_typePriorityList;

      // the next fields are computed during commit()

      // featureNumber[t] is the number of features of type t
      std::vector<TyFeatureOffset> iv_vecFeatureNumber;

      // featureOffset[f] is the offset of feature f
      std::vector< TyFeatureOffset > iv_vecFeatureOffset;

      // iv_vecTypeFeatureOffsetMapping[t][o] is the feature appropriate
      //   for type t with offset o
      std::vector< std::vector<TyFSFeature> > iv_vecTypeFeatureOffsetMapping;

      // approp[t][f] is true if feature f is appropriate for type t
      std::vector< std::vector< bool > > iv_vecApprop;

      // the total order embedding the user defined type priorities
      std::map<TyFSType, size_t> iv_mapTypePriority;
      // use to avoid calling expensive routine if no custom priorities defined
      bool iv_customTypePrioritySet;

      // find typeToBeFound in the subtree of the hierarchy rooted at root
      bool findInTree(TyFSType root, TyFSType typeToBeFound) const;

      /**
       * compute appropriateness conditions on t, in particular,
       * compute which feature is defined on each type, i.e., introduced
       * of an ancestor type
       */
      void computeApprop(TyFSType t);

      /**
       * compute the mapping from types and offsets to features.
       */
      void computeTypeFeatureOffsetMapping();

      /**
       * assign a type priority number to all types.
       */
      void computeTypePriorityClosure();

      /**
       * combine the type priorities explictly defined by the configuration
       * with the the ones of the sibling order (special Talent requirement).
       */
      void combineUserDefinedPrioritiesWithSiblingPriority();

      TypeSystem(TypeSystem const &);
      TypeSystem & operator=(TypeSystem const &);

      TypeSystem();
      ~TypeSystem();


      /**
       * check if <code>tyType</code> is a builtin type
       * (int, string, float).
       */
      bool isBuiltinType(TyFSType tyType) const {
        assert( isValidType(tyType) );
        return ( tyType == uima::internal::gs_tyIntegerType )
               || ( tyType == uima::internal::gs_tyFloatType )
               || ( tyType == uima::internal::gs_tyStringType );
      }

      void createPredefinedCASTypes();

      TyFSType createTypeNoChecks(TyFSType tyParent, icu::UnicodeString const & crName, icu::UnicodeString const & crusCreatorID);

    protected:
      virtual uima::lowlevel::TypeSystem const & getLowlevelTypeSystem() const {
        return *this;
      }
    public:
      static uima::lowlevel::TypeSystem const & promoteTypeSystem(uima::TypeSystem const & typeSystem) {
        return typeSystem.getLowlevelTypeSystem();
      }

      static uima::lowlevel::TypeSystem & promoteTypeSystem(uima::TypeSystem & typeSystem) {
        return CONST_CAST(uima::lowlevel::TypeSystem&, typeSystem.getLowlevelTypeSystem() );
      }

      typedef struct {
        char const * iv_cpszName;
        char const * iv_cpszParentName;
        char const * iv_cpszDescription;
      }
      StTypeInfo;

      typedef struct {
        char const * iv_cpszName;
        char const * iv_cpszIntroTypeName;
        char const * iv_cpszRangeTypeName;
        bool const iv_multipleRefsAllowed;
        char const * iv_cpszDescription;
      }
      StFeatureInfo;


      static const TyFSType INVALID_TYPE;
      static const TyFSFeature INVALID_FEATURE;

      /*
      // names of types and features
      static UIMA_LINK_IMPORTSPEC char const * TYPENAME_INVALID;
      static UIMA_LINK_IMPORTSPEC char const * TYPENAME_TOP;

      static UIMA_LINK_IMPORTSPEC char const * FEATURENAME_INVALID;
      */

#ifndef NDEBUG
      bool debugIsTreeConsistent() const;
      void printTree(int, TyFSType, std::ostream&) const;
#endif
      // These two are used in test_cas when built debug
      void print(std::ostream&) const;
      bool debugIsConsistent() const;

      /**
       * call commit() after you added the complete type hierarchy. It compiles
       * out several tables for faster lookup.
       */
      void commit();

      /**
       * @return true if the type system was already committed.
       */
      bool isCommitted() const {
        return iv_bIsCommitted;
      }

      /**
       * resets the type system, all types and features will be deleted.
       * Advanced use only.
       */
      void reset();

      /*@{*/
      /**
       * @name Building the Type Hierarchy
       */

      /**
       * create a type as a subtype of <code>tyParent</code> with
       * name <code>crName</code>.
       */
      TyFSType createType(TyFSType tyParent, icu::UnicodeString const & crName, icu::UnicodeString const & crusCreatorID);

      TyFSType createStringSubtype(icu::UnicodeString const & crName, std::vector<icu::UnicodeString> const & crStrings, icu::UnicodeString const & crusCreatorID);

      /**
       * create a feature on <code>tyIntro</code> with value type <code>tyValueType</code>
       * and name <code>crName</code>.
       */
      TyFSFeature createFeature(TyFSType tyIntro, TyFSType tyValueType, bool multiRef, icu::UnicodeString const & crName, icu::UnicodeString const & crusCreatorID);

      /**
       * create a type from a type info struct.
       */
      TyFSType createType( StTypeInfo const & , icu::UnicodeString const & crusCreatorID);

      /**
       * create a feature from a feature info struct.
       */
      TyFSType createFeature( StFeatureInfo const & , icu::UnicodeString const & crusCreatorID);

      /**
       * Add a type priority between t1 and t2. Returns true if the pair
       * does not lead to a contradiction, false otherwise.
       */
      bool addTypePriority(TyFSType t1, TyFSType t2);

      /**
       * get the number of the type in an absolute ordering embedding the specified
       * type priorities, i.e.,
       * hasPriorityOver(t1, t2) iff getTypePriorityNumber[t1] < getTypePriorityNumber[t2]
       */
      size_t getTypePriorityNumber(TyFSType t) const;

      /*@}*/


      /**
       * returns true iff it is allowed to add sub types to tyType.
       */
      bool isAllowedToAddSubTypes(TyFSType tyType) const {
        assert( isValidType(tyType) );
        // Integer, Float, String, ArrayBase, FSArray, IntArray, FloatArray, StringArray
        return (!isBuiltinType(tyType))
               && (tyType != uima::internal::gs_tyArrayBaseType)
               && (tyType != uima::internal::gs_tyFSArrayType)
               && (tyType != uima::internal::gs_tyIntArrayType)
               && (tyType != uima::internal::gs_tyFloatArrayType)
               && (tyType != uima::internal::gs_tyStringArrayType);
      }

      /**
       * returns true iff it is allowed to create new feature on tyType.
       */
      bool isAllowedToCreateFeatures(TyFSType tyType) const {
        assert( isValidType(tyType) );
        return (isAllowedToAddSubTypes(tyType))
               && (tyType != iv_tyTop);
      }

      /**
       * returns true iff it is allowd to create an object of type tyType.
       * (Not possible for e.g. integers)
       */
      bool isAllowedToBeInstantiated(TyFSType tyType) const {
        assert( isValidType(tyType) );
        return (! isBuiltinType(tyType) );
      }

      /**
       * check if <code>tyFeature</code> is a valid feature ID.
       */
      bool isValidFeature(TyFSFeature tyFeature) const {
        return( tyFeature>0) && (tyFeature < iv_vecFeatureBaseNames.size());
      }

      /**
       * check if <code>tyType</code> is a valid type ID.
       */
      bool isValidType(TyFSType tyType) const {
        return( tyType>0) && (tyType< iv_vecTypeNames.size());
      }

      /**
       * check if <code>tyType</code> is maximal, i.e.,
       * has no subtypes.
       */
      bool isMaximalType(TyFSType tyType) const {
        assert( isValidType(tyType) );
        return iv_vecTree[tyType].size() == 0;
      }

      /**
       * get the number of all types in the type system.
       */
      size_t getNumberOfTypes() const {
        return iv_vecTypeNames.size() - 1;
      }

      /**
       * get the number of all features in the type system.
       */
      size_t getNumberOfFeatures() const {
        return iv_vecFeatureBaseNames.size() - 1;
      }

      std::vector<icu::UnicodeString> const & getStringsForStringSubtype(TyFSType type) const;

      /**
       * get a list of all the types in the type system.
       * @param rvecResult output parameter
       */
      void getAllTypes(std::vector<TyFSType>& rvecResult) const {
        rvecResult.clear();
        TyFSType i;
        for (i=1; i<iv_vecTypeNames.size(); ++i) {
          rvecResult.push_back(i);
        }
      }

      /**
       * get a list of all the features in the type system.
       * @param rvecResult output parameter
       */
      void getAllFeatures(std::vector<TyFSFeature>& rvecResult) const {
        rvecResult.clear();
        TyFSFeature i;
        for (i=1; i<iv_vecFeatureBaseNames.size(); ++i) {
          rvecResult.push_back(i);
        }
      }

      /**
       * find the type by its name.
       * Returns <code>INVALID_TYPE</code> if no type with the name exists.
       */
      TyFSType getTypeByName(icu::UnicodeString const & crusTypeName) const;

      /**
       * get the name of the type.
       */
      icu::UnicodeString const & getTypeName(TyFSType tyType) const {
        assert( isValidType(tyType) );
        return iv_vecTypeNames[ (size_t) tyType];
      }

      /**
       * get the creator id of the type.
       */
      icu::UnicodeString const & getTypeCreatorID(TyFSType tyType) const {
        assert( isValidType(tyType) );
        return iv_vecTypeCreatorIDs[ (size_t) tyType];
      }

      /**
       * get the feature by its fully qualified name.
       * Returns <code>INVALID_FEATURE</code> if no feature with the name exists.
       */
      TyFSFeature getFeatureByFullName(icu::UnicodeString const & crusFeatureName) const;

      /**
       * get the feature by its base name w.r.t. to a type it is defined for.
       */
      TyFSFeature getFeatureByBaseName(TyFSType tyType, icu::UnicodeString const & crusFeatureName) const;

      TyFSFeature getFeatureByBaseName(icu::UnicodeString const & crusTypeName, icu::UnicodeString const & crusFeatureName) const;

      /**
       * get the name of the feature.
       */
      icu::UnicodeString const & getFeatureBaseName(TyFSFeature tyFeature) const {
        assert( isValidFeature(tyFeature) );
        return iv_vecFeatureBaseNames[ tyFeature ];
      }

      /**
       * returns the fully qualified name of the feature.
       * The type name is its introduction type.
       */
      icu::UnicodeString getFeatureName(TyFSFeature tyFeature) const;

      /**
       * get the creator id of the feature.
       */
      icu::UnicodeString const & getFeatureCreatorID(TyFSFeature tyFeature) const {
        assert( isValidFeature(tyFeature) );
        return iv_vecFeatureCreatorIDs[ tyFeature ];
      }


      /**
       * get a list of all features appropriate for <code>tyType</code>, i.e.,
       * all features introduced by <code>tyType</code> or one of its ancestors.
       * @param rResult output parameter
       */
      void getAppropriateFeatures(TyFSType tyType, std::vector<TyFSFeature>& rResult) const;

      void getDirectSubTypes(TyFSType tyType, std::vector<TyFSType> & rResult) const;

      /**
       * get the range of the feature, i.e., the value type <code>tyFeature</code>
       * must have.
       */
      TyFSType getRangeType(TyFSFeature tyFeature) const {
        assert( isValidFeature(tyFeature) );
        return iv_vecRangeTypes[tyFeature];
      }

      /**
       * get the feature property multipleReferencesAllowed.
       *
       */
      bool isMultipleReferencesAllowed(TyFSFeature tyFeature) const {
        assert( isValidFeature(tyFeature) );
        return iv_vecMultiRefs[tyFeature];
      }


      /**
       * check if <code>tyFeature</code> is appropriate for <code>tyType</code>.
       */
      bool isAppropriateFeature(TyFSType tyType, TyFSFeature tyFeature) const {
        assert( iv_bIsCommitted );
        assert( isValidType(tyType) );
        assert( isValidFeature(tyFeature) );
        return iv_vecApprop[tyType][tyFeature];
      }


      /**
       * get the top type.
       */
      TyFSType getTopType() const {
        return iv_tyTop;
      }

      TyFSType getMostSpecificCommonSupertype(TyFSType t1, TyFSType t2) const;

      /**
       * get all subsumed types of <code>tyType</code>.
       * @param rResult output parameter
       */
      void getSubsumedTypes(TyFSType tyType, std::vector<TyFSType>& rResult) const;

      /**
       * get the parent type of <code>tyType</code>.
       */
      TyFSType getParentType(TyFSType tyType) const;

      /**
       * check if <code>tyType1</code> subsumes <code>tyType2</code>.
       */
      bool subsumes(TyFSType tyType1, TyFSType tyType2) const;

      bool hasPriorityOver(TyFSType tyType1, TyFSType tyType2) const {
        assert( isValidType(tyType1) );
        assert( isValidType(tyType2) );
        assert( iv_mapTypePriority.find(tyType1) != iv_mapTypePriority.end() );
        assert( iv_mapTypePriority.find(tyType2) != iv_mapTypePriority.end() );
        size_t p1 = (*iv_mapTypePriority.find(tyType1)).second;
        size_t p2 = (*iv_mapTypePriority.find(tyType2)).second;
        return p1 < p2;
      }

      /**
       * get the type where <code>tyFeature</code> was introduced.
       */
      TyFSType getIntroType(TyFSFeature tyFeature) const;

      /**
       * method for determining positional encodings of features
       * for heap representation. Since we impose feature introduction,
       * we don't need type information.
       * Use this in combination with uima::lowlevel::FSHeap::getFeatureWithOffset for
       * fast access of features.
       */
      TyFeatureOffset getFeatureOffset(TyFSFeature tyFeature) const {
        assert( iv_bIsCommitted );
        assert( isValidFeature(tyFeature) );
        return iv_vecFeatureOffset[tyFeature];
      }

      TyFSFeature getFeatureFromOffset(TyFSType tyType, TyFeatureOffset tyOffset) const {
        assert( isValidType(tyType) );
        assert( tyType < iv_vecTypeFeatureOffsetMapping.size() );
        assert( tyOffset < iv_vecTypeFeatureOffsetMapping[tyType].size() );
        return iv_vecTypeFeatureOffsetMapping[tyType][tyOffset];
      }

      /**
       * get the number of features of <code>tyType</code>.
       */
      TyFeatureOffset getFeatureNumber(TyFSType tyType) const {
        assert( iv_bIsCommitted );
        assert( isValidType(tyType) );
        return iv_vecFeatureNumber[tyType];
      }

      static icu::UnicodeString const ustrCREATOR_ID_SYSTEM;
    };

  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

