/** \file lowlevel_typesystem.cpp .
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

//#define DEBUG_VERBOSE


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>
#include <uima/macros.h>

#include <uima/macros.h>

#include <algorithm>
#include <uima/lowlevel_typesystem.hpp>
// for definition of string constants for public built-in types
#include <uima/cas.hpp>
#include <uima/internal_fspromoter.hpp>
#include <uima/msg.h>
#include <uima/stltools.hpp>
#include <uima/typesystem.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
using namespace std;
namespace uima {
  namespace lowlevel {
    icu::UnicodeString const TypeSystem::ustrCREATOR_ID_SYSTEM("System");

    TypeSystem::StTypeInfo gs_arBuiltinTypes[] = {
          {
            CAS::TYPE_NAME_INTEGER,      CAS::TYPE_NAME_TOP,      "integer type"
          },
          { CAS::TYPE_NAME_FLOAT,        CAS::TYPE_NAME_TOP,      "float type"},
          { CAS::TYPE_NAME_STRING,       CAS::TYPE_NAME_TOP,      "string type"},
          { CAS::TYPE_NAME_ARRAY_BASE,   CAS::TYPE_NAME_TOP,      "array base type"},
          { CAS::TYPE_NAME_FS_ARRAY,      CAS::TYPE_NAME_ARRAY_BASE,"fs array"},
          { CAS::TYPE_NAME_FLOAT_ARRAY,   CAS::TYPE_NAME_ARRAY_BASE,"float array"},
          { CAS::TYPE_NAME_INTEGER_ARRAY,     CAS::TYPE_NAME_ARRAY_BASE,"int array"},
          { CAS::TYPE_NAME_STRING_ARRAY,  CAS::TYPE_NAME_ARRAY_BASE,"string array"},
          { CAS::TYPE_NAME_LIST_BASE,    CAS::TYPE_NAME_TOP,      "list base type"},
          { CAS::TYPE_NAME_FS_LIST,       CAS::TYPE_NAME_LIST_BASE,"fs list"},
          { CAS::TYPE_NAME_EMPTY_FS_LIST,       CAS::TYPE_NAME_FS_LIST,   "e_fs list"},
          { CAS::TYPE_NAME_NON_EMPTY_FS_LIST,      CAS::TYPE_NAME_FS_LIST,   "ne fs list"},
          { CAS::TYPE_NAME_FLOAT_LIST,    CAS::TYPE_NAME_LIST_BASE,"list of floats"},
          { CAS::TYPE_NAME_EMPTY_FLOAT_LIST,  CAS::TYPE_NAME_FLOAT_LIST,"empty list of floats"},
          { CAS::TYPE_NAME_NON_EMPTY_FLOAT_LIST, CAS::TYPE_NAME_FLOAT_LIST,"non empty list of floats"},
          { CAS::TYPE_NAME_INTEGER_LIST,      CAS::TYPE_NAME_LIST_BASE,"list of integers"},
          { CAS::TYPE_NAME_EMPTY_INTEGER_LIST,    CAS::TYPE_NAME_INTEGER_LIST,  "empty list of integers"},
          { CAS::TYPE_NAME_NON_EMPTY_INTEGER_LIST,   CAS::TYPE_NAME_INTEGER_LIST,  "non empty list of integers"},
          { CAS::TYPE_NAME_STRING_LIST,   CAS::TYPE_NAME_LIST_BASE,"list of strings"},
          { CAS::TYPE_NAME_EMPTY_STRING_LIST, CAS::TYPE_NAME_STRING_LIST,"empty list of strings"},
          { CAS::TYPE_NAME_NON_EMPTY_STRING_LIST,CAS::TYPE_NAME_STRING_LIST,"non empty list of strings"},
          { CAS::TYPE_NAME_BOOLEAN,        CAS::TYPE_NAME_TOP,   "boolean  type"},
          { CAS::TYPE_NAME_BYTE,           CAS::TYPE_NAME_TOP,      "8Bit  type"},
          { CAS::TYPE_NAME_SHORT,          CAS::TYPE_NAME_TOP,      "16Bit type"},
          { CAS::TYPE_NAME_LONG,           CAS::TYPE_NAME_TOP,      "64Bit type"},
          { CAS::TYPE_NAME_DOUBLE,         CAS::TYPE_NAME_TOP,      "long double  type"},
          { CAS::TYPE_NAME_BOOLEAN_ARRAY,   CAS::TYPE_NAME_ARRAY_BASE,      "boolean array  type"},
          { CAS::TYPE_NAME_BYTE_ARRAY,      CAS::TYPE_NAME_ARRAY_BASE,      "8Bit array  type"},
          { CAS::TYPE_NAME_SHORT_ARRAY,     CAS::TYPE_NAME_ARRAY_BASE,      "16Bit array type"},
          { CAS::TYPE_NAME_LONG_ARRAY,      CAS::TYPE_NAME_ARRAY_BASE,      "64Bit array type"},
          { CAS::TYPE_NAME_DOUBLE_ARRAY,    CAS::TYPE_NAME_ARRAY_BASE,      "long double array  type"},

        };

    TypeSystem::StFeatureInfo gs_arBuiltinFeatures[] = {
          {
            CAS::FEATURE_BASE_NAME_HEAD, CAS::TYPE_NAME_NON_EMPTY_FS_LIST, CAS::TYPE_NAME_TOP, false, "head"},
          { CAS::FEATURE_BASE_NAME_TAIL, CAS::TYPE_NAME_NON_EMPTY_FS_LIST, CAS::TYPE_NAME_FS_LIST, false, "tail"},
          { CAS::FEATURE_BASE_NAME_HEAD, CAS::TYPE_NAME_NON_EMPTY_FLOAT_LIST, CAS::TYPE_NAME_FLOAT, false, "head of float list"},
          { CAS::FEATURE_BASE_NAME_TAIL, CAS::TYPE_NAME_NON_EMPTY_FLOAT_LIST, CAS::TYPE_NAME_FLOAT_LIST, false, "tail of float list"},
          { CAS::FEATURE_BASE_NAME_HEAD, CAS::TYPE_NAME_NON_EMPTY_INTEGER_LIST, CAS::TYPE_NAME_INTEGER, false, "head of integer list"},
          { CAS::FEATURE_BASE_NAME_TAIL, CAS::TYPE_NAME_NON_EMPTY_INTEGER_LIST, CAS::TYPE_NAME_INTEGER_LIST, false, "tail of integer list"},
          { CAS::FEATURE_BASE_NAME_HEAD, CAS::TYPE_NAME_NON_EMPTY_STRING_LIST, CAS::TYPE_NAME_STRING, false, "head of string list"},
          { CAS::FEATURE_BASE_NAME_TAIL, CAS::TYPE_NAME_NON_EMPTY_STRING_LIST, CAS::TYPE_NAME_STRING_LIST, false, "tail of string list"}
        };

    char const * TYPE_NAME_INVALID               = "INVALID_TYPE";
    char const * FEATURE_NAME_INVALID            = "INVALIDFEATURE";
  }
}




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
  namespace lowlevel {

    UIMA_EXC_CLASSIMPLEMENT(FeatureIntroductionFailedException, CASException);
    UIMA_EXC_CLASSIMPLEMENT(TypeCreationFailedException, CASException);

    ////////////////////////////////////////////////////////////////////

    TypeSystem::TypeSystem()
        : iv_bIsCommitted(false),
        iv_bBuiltinTypesCreated(false),
        iv_tyTop(1) {
      reset();
    }

    const TyFSType TypeSystem::INVALID_TYPE = 0;
    const TyFSFeature TypeSystem::INVALID_FEATURE = 0;


    void TypeSystem::createPredefinedCASTypes() {
      assert( !iv_bIsCommitted );
      assert( !iv_bBuiltinTypesCreated );

      iv_tyTop = 1;
      iv_vecTree.clear();
      iv_vecTree.resize(2); // reserve space for top, leave first slot empty
      iv_vecIntroducedFeatures.clear();
      iv_vecIntroducedFeatures.resize(2);
      iv_vecTypeNames.resize(2);
      iv_vecTypeNames[1] = CAS::TYPE_NAME_TOP;
      iv_vecTypeNames[0] = TYPE_NAME_INVALID;
      iv_vecParents.resize(2);
      iv_vecParents[0] = TypeSystem::INVALID_TYPE;
      iv_vecParents[1] = iv_tyTop;
      iv_vecFeatureBaseNames.resize(1);
      iv_vecFeatureBaseNames[0] = FEATURE_NAME_INVALID;
      iv_vecFeatureCreatorIDs.resize(1);
      iv_vecFeatureCreatorIDs[0] = FEATURE_NAME_INVALID;
      iv_vecRangeTypes.resize(1);
      iv_vecMultiRefs.resize(1);


      size_t uiType;
      for (uiType =  0; uiType < NUMBEROF(gs_arBuiltinTypes) ; ++uiType) {
        createType( gs_arBuiltinTypes[uiType], ustrCREATOR_ID_SYSTEM );
      }

      size_t uiFeature;
      for (uiFeature = 0; uiFeature <  NUMBEROF(gs_arBuiltinFeatures) ; ++uiFeature) {
        createFeature( gs_arBuiltinFeatures[uiFeature], ustrCREATOR_ID_SYSTEM );
      }

      iv_bBuiltinTypesCreated = true;
    }

    TypeSystem::~TypeSystem() {}


    TyFSType TypeSystem::createType(TyFSType tyParent, icu::UnicodeString const & crusName, icu::UnicodeString const & crusCreatorID) {
      assert( !iv_bIsCommitted);
      assert( (tyParent > 0) && (tyParent < iv_vecTree.size()) );

      if (iv_bBuiltinTypesCreated && ! isAllowedToAddSubTypes(tyParent)) {
        ErrorMessage errMessage(UIMA_MSG_ID_EXCON_CREATING_TYPE, crusName);
        errMessage.addParam(getTypeName(tyParent) );

        UIMA_EXC_THROW_NEW(TypeCreationFailedException,
                           UIMA_ERR_TYPE_CREATION_FAILED,
                           UIMA_MSG_ID_EXC_TYPE_CREATION_FAILED_FINAL_TYPE,
                           errMessage,
                           ErrorInfo::recoverable);
      }
      return createTypeNoChecks(tyParent, crusName, crusCreatorID);
    }

    TyFSType TypeSystem::createTypeNoChecks(TyFSType tyParent, icu::UnicodeString const & crusName, icu::UnicodeString const & crusCreatorID) {
      assert( debugIsTreeConsistent() );

      if ( find(iv_vecTypeNames.begin(), iv_vecTypeNames.end(), crusName) != iv_vecTypeNames.end() ) {
        // typename already exists
        ErrorMessage errMessage(UIMA_MSG_ID_EXCON_CREATING_TYPE, crusName);
        errMessage.addParam(getTypeName(tyParent) );
        UIMA_EXC_THROW_NEW(TypeCreationFailedException,
                           UIMA_ERR_TYPE_CREATION_FAILED,
                           UIMA_MSG_ID_EXC_TYPE_ALREADY_EXISTS,
                           errMessage,
                           ErrorInfo::recoverable);
      }

      // add new type slots
      TyFSType newType = iv_vecTree.size();
      iv_vecTree.resize(newType + 1);
      iv_vecParents.resize(newType + 1);
      iv_vecIntroducedFeatures.resize(iv_vecTree.size());
      iv_vecTypeNames.resize(iv_vecTree.size());
      iv_vecTypeCreatorIDs.resize(iv_vecTree.size());

      // add all type specific data
      iv_vecParents[newType] = tyParent;
      vector<TyFSType>& children = iv_vecTree[tyParent];
      children.push_back( newType );
      iv_vecTypeNames[newType] = crusName;
      iv_vecTypeCreatorIDs[newType] = crusCreatorID;

      assert( debugIsTreeConsistent() );
      return newType;
    }


    TyFSFeature TypeSystem::createFeature(TyFSType tyIntro, TyFSType tyRange, bool multiRef, icu::UnicodeString const & crusName, icu::UnicodeString const & crusCreatorID) {
      UIMA_TPRINT("Creating feature " << crusName);
      assert( !iv_bIsCommitted);
      assert( (tyIntro > 0) && (tyIntro < iv_vecTree.size()) );
      assert( (tyRange > 0) && (tyRange < iv_vecTree.size()) );
      assert( isValidType(tyIntro) );
      assert( isValidType(tyRange) );

      if (iv_bBuiltinTypesCreated && ! isAllowedToCreateFeatures(tyIntro) ) {
        // throw exception
        ErrorMessage errMessage(UIMA_MSG_ID_EXCON_CREATING_FEATURE, crusName);
        errMessage.addParam(getTypeName(tyIntro) );

        UIMA_EXC_THROW_NEW(FeatureIntroductionFailedException,
                           UIMA_ERR_FEATURE_INTRO_FAILED,
                           UIMA_MSG_ID_EXC_FEATURE_INTRO_FAILED_FINAL_TYPE,
                           errMessage,
                           ErrorInfo::recoverable);
      }

      assert( debugIsTreeConsistent() );

      if ( getFeatureByBaseName( tyIntro, crusName ) != INVALID_FEATURE ) {
        // throw exception
        ErrorMessage errMessage(UIMA_MSG_ID_EXCON_CREATING_FEATURE, crusName);
        errMessage.addParam(getTypeName(tyIntro) );
        UIMA_EXC_THROW_NEW(FeatureIntroductionFailedException,
                           UIMA_ERR_FEATURE_INTRO_FAILED,
                           UIMA_MSG_ID_EXC_FEATURE_ALREADY_EXISTS,
                           errMessage,
                           ErrorInfo::recoverable);
      }

      // add feature specific data
      TyFSFeature newFeature = iv_vecFeatureBaseNames.size();
      iv_vecFeatureBaseNames.push_back(crusName);
      iv_vecRangeTypes.push_back(tyRange);
      iv_vecMultiRefs.push_back(multiRef);
      iv_vecIntroducedFeatures[tyIntro].push_back(newFeature);
      iv_vecFeatureCreatorIDs.push_back(crusName);

      assert( debugIsTreeConsistent() );
      return newFeature;
    }

    TyFSType TypeSystem::createType( StTypeInfo const & crTypeInfo, icu::UnicodeString const & crusCreatorID) {
      UIMA_TPRINT("Adding type " << crTypeInfo.iv_cpszName << " as son of " << crTypeInfo.iv_cpszParentName);
      assert( !iv_bIsCommitted);
      TyFSType tyParent = getTypeByName(icu::UnicodeString( crTypeInfo.iv_cpszParentName) );
      assert( tyParent != INVALID_TYPE );
      TyFSType tyNewType = createType( tyParent, crTypeInfo.iv_cpszName, crusCreatorID);
      assert( tyNewType != INVALID_TYPE );
      return tyNewType;
    }

    TyFSType TypeSystem::createStringSubtype(icu::UnicodeString const & crName,
        vector<icu::UnicodeString> const & crStrings,
        icu::UnicodeString const & crusCreatorID) {
      size_t uiNewEntry = iv_enumStrings.size();
      iv_enumStrings.push_back(crStrings);

      TyFSType stringType = getTypeByName(CAS::TYPE_NAME_STRING);
      TyFSType result = createTypeNoChecks(stringType, crName, crusCreatorID);
      iv_stringSubtypeToStringSet[result] = uiNewEntry;
      return result;
    }


    TyFSFeature TypeSystem::createFeature( StFeatureInfo const & crFeatureInfo, icu::UnicodeString const & crusCreatorID) {
      UIMA_TPRINT("Adding feature " << crFeatureInfo.iv_cpszName << " on intro " << crFeatureInfo.iv_cpszIntroTypeName << " with range " << crFeatureInfo.iv_cpszRangeTypeName );
      assert( !iv_bIsCommitted);
      TyFSType tyIntroType = getTypeByName(icu::UnicodeString(crFeatureInfo.iv_cpszIntroTypeName) );
      assert( tyIntroType != INVALID_TYPE );
      TyFSType tyRangeType = getTypeByName(icu::UnicodeString( crFeatureInfo.iv_cpszRangeTypeName) );
      assert( tyRangeType != INVALID_TYPE );
      TyFSFeature tyNewFeature = createFeature( tyIntroType, tyRangeType, crFeatureInfo.iv_multipleRefsAllowed, crFeatureInfo.iv_cpszName, crusCreatorID );
      assert( tyNewFeature != INVALID_FEATURE );
      return tyNewFeature;
    }

    /**
     * return true iff there is an edge which has node as the end point.
     */
    template <class T>
    bool hasPredecessor(vector< pair<T, T> > const & rAdjList, T const & node) {
      size_t i;
      for (i=0; i<rAdjList.size(); ++i) {
        if (rAdjList[i].second == node) {
          return true;
        }
      }
      return false;
    }

    /**
     * Delete all edges where the start or end node is node.
     */
    template <class T>
    void deleteAllEdges(vector< pair<T, T> > & rAdjList, T const & node) {
      vector< pair<T, T> > result;
      size_t i;
      for (i=0; i<rAdjList.size(); ++i) {
        assert(rAdjList[i].second != node);
        if (rAdjList[i].first != node) {
          result.push_back( rAdjList[i] );
        }
      }
      rAdjList = result;
    }

    /**
     * find a node in a graph with no predecessors and delete it.
     * return true if such a node exists. result is the output parameter
     * indiacting the node which could be deleted.
     */
    template <class T>
    bool deleteNodeWithoutPredecessor(vector<T> & rNodes, vector< pair<T, T> > & rAdjList, T& result) {
      typename vector<T>::iterator it;
      for (it = rNodes.begin(); it != rNodes.end(); ++it) {
        if (! hasPredecessor(rAdjList, *it)) {
#ifndef NDEBUG
          size_t oldAdjListLength = rAdjList.size();
#endif
          deleteAllEdges(rAdjList, *it);
          assert( rAdjList.size() <= oldAdjListLength );
          result = *it;
          rNodes.erase(it);
          return true;
        }
      }
      return false;
    }

    // sorts the graph given by nodes and adjList topologically. The output maps
    // elements to its number in the sort order.
    // The algorithm works as follows:
    // 1. select some node v without a predecessor,
    //    if no such node can be found and the graph is not empty->fail
    // 2. delete the v from the graph
    // 3. v is the next node in the topological ordering
    // 4. repeat 1,2,3 until graph is empty
    //
    // Note: the nodes and adjacency list are passed by-value because the algorithm
    //       is destructive on these data structures.
    template <class T>
    bool topologicalSort(vector<T> nodes, vector< pair<T, T> > adjList, map<T, size_t> & rTotalOrder) {
#ifndef NDEBUG
      vector<pair<T,T> > adjListCheck = adjList;
#endif
      rTotalOrder.clear();
      size_t i = 0;
      T node;
      while (1) {
        bool bNodeCouldBeSelected = deleteNodeWithoutPredecessor(nodes, adjList, node);
        if (!bNodeCouldBeSelected) {
          bool bIsSortable = nodes.empty();
#ifndef NDEBUG
          // check order
          if (bIsSortable) {
            size_t j;
            for (j=0; j<adjListCheck.size(); ++j) {
              assert( (*rTotalOrder.find(adjListCheck[j].first)).second <
                      (*rTotalOrder.find(adjListCheck[j].second)).second );
//                     assert( rTotalOrder[ adjListCheck[j].first ] < rTotalOrder[ adjListCheck[j].second ] );
            }
          }
#endif
          return bIsSortable;
        }
        rTotalOrder[node] = i;
        ++i;
      }
      assert(false);
      return false;
    }

    // checks if the graph consisting of nodes rNodes and the edges coded as an
    // adjacency list has a cycle.
    template< class T>
    bool hasCycle(vector<T> const & rNodes, vector< pair<T, T> > const & rAdjList) {
      map<T, size_t> order;
      bool bResult = topologicalSort(rNodes, rAdjList, order);
#ifdef DEBUG_VERBOSE
      UIMA_TPRINT("topological order:");
      map<T, size_t>::const_iterator cit;
      for (cit = order.begin(); cit != order.end() ;++ cit) {
        UIMA_TPRINT(" type: " << (*cit).first << ", number: " << (*cit).second);
      }
#endif
      return ! bResult;
    }


    bool TypeSystem::addTypePriority(TyFSType t1, TyFSType t2) {
      assert( isValidType(t1) );
      assert( isValidType(t2) );
      iv_customTypePrioritySet = true;
      // copy type priority pairs
      vector< pair<TyFSType, TyFSType> > typePriorities = iv_typePriorityList;
      typePriorities.push_back( pair<TyFSType, TyFSType>(t1, t2) );

#ifdef DEBUG_VERBOSE
      UIMA_TPRINT("Adding type priority between type " << getTypeName(t1) << " and " << getTypeName(t2) );
      size_t i;
      for (i=0; i<typePriorities.size(); ++i) {
        UIMA_TPRINT("   " << getTypeName(typePriorities[i].first) << " < " << getTypeName(typePriorities[i].second) );
      }
#endif

      vector<TyFSType> allTypes;
      getAllTypes(allTypes);

      // if type priorities are not cyclic
      // hasCycle() does a lot of work but this is done at initialization time only.
      // Furthermore, this has the big benefit that this method leaves a consistent state.
      if (! hasCycle(allTypes, typePriorities) ) {
        iv_typePriorityList.push_back( pair<TyFSType, TyFSType>(t1, t2) );
        return true;
      }
      return false;
    }

    size_t TypeSystem::getTypePriorityNumber(TyFSType t) const {
      assert( isValidType(t) );
      assert( iv_mapTypePriority.find(t) != iv_mapTypePriority.end() );
      return (*iv_mapTypePriority.find(t)).second;
    }


    bool TypeSystem::findInTree(TyFSType tyType, TyFSType tyTypeToBeFound) const {
      assert( isValidType(tyType) );
      assert( isValidType(tyTypeToBeFound) );
      if (tyType == tyTypeToBeFound) {
        return true;
      }
      size_t i;
      for (i=0; i < iv_vecTree[tyType].size(); ++i) {
        if (findInTree(iv_vecTree[tyType][i], tyTypeToBeFound)) {
          return true;
        }
      }
      return false;
    }


    bool TypeSystem::subsumes(TyFSType tyT1, TyFSType tyT2) const {
//         return findInTree(tyT1, tyT2);
      if (tyT1 == tyT2)
        return true;
      if (tyT2 >= iv_vecTree.size() || tyT2 < 1)
        return false;
      if (iv_tyTop == iv_vecParents[tyT2]) {
        if (tyT1 == iv_tyTop)
          return true;
        else
          return false;
      }
      return subsumes(tyT1, iv_vecParents[tyT2]);
    }


    void TypeSystem::computeApprop(TyFSType tyType) {
      assert( debugIsTreeConsistent() );
      assert( isValidType(tyType) );
      size_t i;
      for (i=0; i<iv_vecTree[tyType].size(); ++i) {
        TyFSType childType = iv_vecTree[tyType][i];

        size_t j;
        // add inherited features
        //   appropriateness
        for (j=0; j<iv_vecFeatureBaseNames.size(); ++j) {
          iv_vecApprop[ childType ][j] = iv_vecApprop[tyType][j];
        }
        //   feature numbers
        iv_vecFeatureNumber[ childType ] += iv_vecFeatureNumber[tyType];

        // compute offsets and numbers for new features
        UIMA_TPRINT("Checking introduced feature of type " << getTypeName( childType) );
        assert( childType < iv_vecIntroducedFeatures.size() );
        for (j=0; j<iv_vecIntroducedFeatures[ childType ].size(); ++j) {
          UIMA_TPRINT("Adding introduced feature of type " << childType);
          assert( j < iv_vecIntroducedFeatures[childType].size() );
          TyFSFeature introFeature = iv_vecIntroducedFeatures[ childType ][j];
          ++(iv_vecFeatureNumber[ childType ]);
          iv_vecFeatureOffset[ introFeature ] = iv_vecFeatureNumber[ childType ];
          assert( introFeature < iv_vecApprop[ childType ].size() );
          iv_vecApprop[ childType ][ introFeature ] = true;
        }
        UIMA_TPRINT("   Checking done");
        computeApprop(childType);
      }
      assert( debugIsTreeConsistent() );
    }

    void TypeSystem::computeTypeFeatureOffsetMapping() {
      assert( iv_vecApprop.size() > 0 );
      iv_vecTypeFeatureOffsetMapping.resize( iv_vecTypeNames.size() );
      size_t t,f;
      for (t=1; t<iv_vecTypeNames.size(); ++t) {
        assert( t < iv_vecApprop.size() );
        vector<TyFSFeature> & rOffsetFeatureMapping = iv_vecTypeFeatureOffsetMapping[t];
        for (f=1; f<iv_vecFeatureBaseNames.size(); ++f) {
          assert( f< iv_vecApprop[t].size() );
          if (iv_vecApprop[t][f]) {
            TyFeatureOffset tyOffset = iv_vecFeatureOffset[f];
            if (tyOffset >= rOffsetFeatureMapping.size()) {
              rOffsetFeatureMapping.resize(tyOffset + 1, INVALID_FEATURE);
            }
            rOffsetFeatureMapping[tyOffset] = f;
          }
        }
      }
    }


    void computePreOrder(vector<vector<TyFSType> > const & tree, size_t root, vector<TyFSType> & result) {
      // visit node
      result.push_back(root);

      // go through all children
      assert( root < tree.size() );
      vector<TyFSType> const & children = tree[root];
      size_t i;
      for (i=0; i<children.size(); ++i) {
        computePreOrder(tree, children[i], result);
      }
    }

    class TypePriorityCompare {
      map<TyFSType, size_t> iv_map;
    public:
      TypePriorityCompare(vector< pair<TyFSType, TyFSType> > const & priorityList,
                          vector< TyFSType> const & allTypes) {
        bool bIsSortable = topologicalSort(allTypes, priorityList, iv_map);
        assert( bIsSortable );
      }

      bool operator()(TyFSType t1, TyFSType t2) const {
        size_t p1 = (*iv_map.find(t1)).second;
        size_t p2 = (*iv_map.find(t2)).second;
        return p1 < p2;
      }
    };

    // the algorithm to combine the user defined type priorities with the Talent
    // view works as follows:
    //   1. take the type tree and sort all siblings w.r.t. the user defined priorities
    //   2. compute a pre order of this tree (this is a total order)
    //   3. try to insert the pairs of the total order into the type priority
    // if type priorities are only defined between siblings, this gives the Talent behaviour
    // if 3. fails, only the user defined ordering counts!
    void TypeSystem::combineUserDefinedPrioritiesWithSiblingPriority() {
      // copy tree
      vector< vector<TyFSType> > typeTree = iv_vecTree;

      // sort siblings in tree
      vector<TyFSType> allTypes;
      getAllTypes(allTypes);
      TypePriorityCompare compare( iv_typePriorityList, allTypes);
      size_t i;
      for (i=1; i<typeTree.size(); ++i) {
        vector<TyFSType> & siblings = typeTree[i];
        sort(siblings.begin(), siblings.end(), compare);
      }

      // determine tree pre order
      vector<TyFSType> typePreOrder;
      computePreOrder( typeTree, getTopType(), typePreOrder );
      assert( typePreOrder.size() == getNumberOfTypes() );

      vector< pair<TyFSType, TyFSType> > priorityListBackup = iv_typePriorityList;
      bool bOrdersAreCompatible = true;
      // try to insert w.r.t. pre order
      for (i=0; i<typePreOrder.size()-1; ++i) {
        if (! addTypePriority(typePreOrder[i], typePreOrder[i+1] )) {
          bOrdersAreCompatible = false;
          break;
        }
      }
      // restore old priorities if the user defined order is incompatible with Talent's
      if (!bOrdersAreCompatible) {
        iv_typePriorityList = priorityListBackup;
      }

    }


    void TypeSystem::computeTypePriorityClosure() {
      // prepare iv_typePriorityList

      // this routine does not scale well with a large type system ...
      // ... so skip it if possible
      if (iv_customTypePrioritySet) {
        combineUserDefinedPrioritiesWithSiblingPriority();
      }

      // compute actual type priority
      vector<TyFSType> allTypes;
      getAllTypes(allTypes);
      bool bIsSortable = topologicalSort(allTypes, iv_typePriorityList, iv_mapTypePriority);
      assert( bIsSortable );
    }


    void TypeSystem::commit() {
      assert( ! iv_bIsCommitted );
      iv_vecFeatureNumber.clear();
      iv_vecFeatureOffset.clear();
      iv_vecApprop.clear();

      // compute featureNumbers and offsets
      size_t featureNum = iv_vecFeatureBaseNames.size();
      size_t typeNum = iv_vecTypeNames.size();

      iv_vecFeatureNumber.resize(typeNum, 0);
      iv_vecFeatureOffset.resize(featureNum, 0);
      iv_vecApprop.resize(typeNum);

      size_t i;
      for (i=0; i<iv_vecApprop.size() ;++i) {
        iv_vecApprop[i].resize(featureNum, false);
      }

      UIMA_TPRINT("Computing Approp");
      computeApprop(iv_tyTop);
      UIMA_TPRINT("  Approp computed");

      computeTypeFeatureOffsetMapping();
      computeTypePriorityClosure();

      assert( debugIsConsistent() );
      iv_bIsCommitted = true;

      UIMA_TPRINT("TypeSystem::init() finished");
    }


    void TypeSystem::getSubsumedTypes(TyFSType tyType, vector<TyFSType>& rvecResult) const {
      assert( isValidType(tyType) );
      rvecResult.push_back(tyType);
      size_t i;
      for (i=0; i < iv_vecTree[tyType].size(); ++i) {
        getSubsumedTypes(iv_vecTree[tyType][i], rvecResult);
      }
    }



    TyFSType TypeSystem::getMostSpecificCommonSupertype(TyFSType t1, TyFSType t2) const {
      assert( isValidType(t1) );
      assert( isValidType(t2) );
      if (subsumes(t1, t2)) {
        return t1;
      }
      if (subsumes(t2, t1)) {
        return t2;
      }
      t1 = getParentType(t1);
      return getMostSpecificCommonSupertype(t1, t2);
    }


    TyFSType TypeSystem::getIntroType(TyFSFeature tyFeature) const {
      TyFSType t;
      for (t=1; t<iv_vecIntroducedFeatures.size(); ++t) {
        vector<TyFSFeature> const & rFeatures = iv_vecIntroducedFeatures[t];
        size_t i;
        for (i=0; i<rFeatures.size(); ++i) {
          if (rFeatures[i] == tyFeature) {
            return t;
          }
        }
      }
      assert(false);
      return INVALID_TYPE;
    }

    TyFSType TypeSystem::getParentType(TyFSType tyType) const {
      assert( isValidType(tyType) );
      TyFSType t;
      for (t=1; t<iv_vecTree.size(); ++t) {
        vector<TyFSType> const & rDaughters = iv_vecTree[t];
        size_t i;
        for (i=0; i<rDaughters.size(); ++i) {
          if (rDaughters[i] == tyType) {
            return t;
          }
        }
      }
      return INVALID_TYPE;
    }


    void TypeSystem::getAppropriateFeatures(TyFSType tyType, vector<TyFSFeature>& rResult) const {
      assert( iv_bIsCommitted );
      rResult.clear();
      assert( isValidType(tyType) );
      size_t i;
      for (i=1; i< iv_vecApprop[tyType].size(); ++i) {
        bool bIsApprop = iv_vecApprop[tyType][i];
        if (bIsApprop) {
          rResult.push_back( i );
        }
      }
    }

    void TypeSystem::getDirectSubTypes(TyFSType tyType, vector<TyFSType> & rResult) const {
      assert( isValidType(tyType) );
      rResult = iv_vecTree[tyType];
    }


    TyFSType TypeSystem::getTypeByName(icu::UnicodeString const & crusTypeName) const {
      int i = findIndex(iv_vecTypeNames, crusTypeName);
      assert( i != 0 );
      if (i < 1) {
        return INVALID_TYPE;
      }
      return i;
    }


    TyFSFeature TypeSystem::getFeatureByBaseName(TyFSType tyType, icu::UnicodeString const & crusFeatureName) const {
      // cannot call getAppropriateFeatures(tyType, vecFeatures)
      //  because type system may not be committed
      if ( !isValidType(tyType) ) {
        return INVALID_FEATURE;
      }
      while (tyType != iv_tyTop) {
        size_t i;
        assert( isValidType( tyType ) );
        vector<TyFSFeature> const & crFeatures = iv_vecIntroducedFeatures[tyType];
        for (i=0; i<crFeatures.size(); ++i) {
          TyFSFeature tyFeat = crFeatures[i];
          if (iv_vecFeatureBaseNames[ tyFeat ] == crusFeatureName ) {
            return tyFeat;
          }
        }
        tyType = getParentType(tyType);
      }
      return INVALID_FEATURE;
    }

    TyFSFeature TypeSystem::getFeatureByBaseName(icu::UnicodeString const & crusTypeName, icu::UnicodeString const & crusFeatureName) const {
      TyFSType tyType = getTypeByName(crusTypeName);
      if ( !isValidType(tyType) ) {
        return INVALID_FEATURE;
      }
      return getFeatureByBaseName( tyType, crusFeatureName );
    }


    TyFSFeature TypeSystem::getFeatureByFullName(icu::UnicodeString const & crusFeatureName) const {
      UChar sepChar( uima::TypeSystem::FEATURE_SEPARATOR );
      int32_t  iIndex = crusFeatureName.indexOf(sepChar);
      // if there is no or more than one separator
      if ( (iIndex == -1) || (iIndex != crusFeatureName.lastIndexOf(sepChar)) ) {
        return INVALID_FEATURE;
      }

      icu::UnicodeString featureBaseName;
      icu::UnicodeString typeName;

      crusFeatureName.extractBetween(0, iIndex, typeName);
      crusFeatureName.extractBetween(iIndex + 1, crusFeatureName.length(), featureBaseName);
      return getFeatureByBaseName( typeName, featureBaseName );
    }


    icu::UnicodeString TypeSystem::getFeatureName(TyFSFeature tyFeature) const {

      icu::UnicodeString const & crBaseName = getFeatureBaseName(tyFeature);
      icu::UnicodeString const & crTypeName = getTypeName( getIntroType(tyFeature) );
      icu::UnicodeString result( crTypeName );
      result.append((UChar) uima::TypeSystem::FEATURE_SEPARATOR);
      result.append(crBaseName);
      return result;
    }


    vector<icu::UnicodeString> const & TypeSystem::getStringsForStringSubtype(TyFSType type) const {
      assert( subsumes( getTypeByName(CAS::TYPE_NAME_STRING), type) );
      map<TyFSType, TyFeatureOffset>::const_iterator cit = iv_stringSubtypeToStringSet.find(type);
      assert( cit != iv_stringSubtypeToStringSet.end() );
      TyFeatureOffset uiEntry = (*cit).second;
      assert( uiEntry < iv_enumStrings.size() );
      return iv_enumStrings[uiEntry];
    }


    void TypeSystem::reset() {
      iv_bIsCommitted = false;
      iv_bBuiltinTypesCreated = false;
      iv_customTypePrioritySet = false;
      createPredefinedCASTypes();
      iv_mapTypePriority.clear();
      iv_typePriorityList.clear();
    }



    /**************************************************************************/

#ifndef NDEBUG

#define ASSERT_OR_RETURN_FALSE(x) assert(x)
//#define ASSERT_OR_RETURN_FALSE(x) if (!(x)) return false

    bool TypeSystem::debugIsTreeConsistent() const {
      ASSERT_OR_RETURN_FALSE(iv_vecTree.size() == iv_vecIntroducedFeatures.size() );
      ASSERT_OR_RETURN_FALSE(iv_vecTree.size() == iv_vecTypeNames.size() );
      ASSERT_OR_RETURN_FALSE(iv_vecRangeTypes.size() == iv_vecFeatureBaseNames.size());
      ASSERT_OR_RETURN_FALSE(iv_vecMultiRefs.size() == iv_vecFeatureBaseNames.size());

      size_t n = iv_vecTree.size();
      size_t i,j;
      for (i=0; i<n; ++i) {
        for (j=0; j<iv_vecTree[i].size(); ++j) {
          ASSERT_OR_RETURN_FALSE( !( (iv_vecTree[i][j] < 1) || (iv_vecTree[i][j] >= n) ) );
        }
      }

      for (i=1; i<iv_vecTypeNames.size(); ++i) {
        icu::UnicodeString const & name = iv_vecTypeNames[i];
        for (j=i+1; j<iv_vecTypeNames.size(); ++j) {
          ASSERT_OR_RETURN_FALSE(name != iv_vecTypeNames[j]);
        }
      }

      for (i=1; i<iv_vecFeatureBaseNames.size(); ++i) {
        ASSERT_OR_RETURN_FALSE( (0<iv_vecRangeTypes[i]) && (iv_vecRangeTypes[i]<iv_vecTree.size()) );
        // don't check for feature name uniqueness here
        // feature names are no longer unique
      }

      return true;
    }

    bool TypeSystem::debugIsConsistent() const {
      ASSERT_OR_RETURN_FALSE( debugIsTreeConsistent() );
      ASSERT_OR_RETURN_FALSE( iv_vecFeatureBaseNames.size() == iv_vecFeatureOffset.size() );
      ASSERT_OR_RETURN_FALSE( iv_vecFeatureNumber.size() == iv_vecTypeNames.size() );
      return true;
    }



    void TypeSystem::printTree(int tab, TyFSType type, ostream& os) const {
      int i,j;
      for (i=0; i<tab; ++i) os << " ";

      os << iv_vecTypeNames[type] << " (" << (int) type << ")" << endl;

      vector<TyFSFeature> const & features = iv_vecIntroducedFeatures[type];
      for (j=0; j<(int)features.size(); ++j) {
        for (i=0; i<tab; ++i) os << " ";
        os << " [" << iv_vecFeatureBaseNames[features[j]] << " (" << features[j] << "): " << iv_vecTypeNames[ iv_vecRangeTypes[features[j]] ] << "]" << endl;
      }

      for (j=0; j<(int)iv_vecTree[type].size(); ++j) {
        printTree(tab+4, iv_vecTree[type][j], os);
      }

    }

    void TypeSystem::print(ostream& os) const {
      os << "===============================================" << endl;
      os << "TREE: " << endl;
      assert( debugIsTreeConsistent() );
      printTree(0, iv_tyTop, os);
      os << "---------------------------------------------" << endl;
      size_t i,j;

      os << "FEATURE_NAMES" << endl;

      for (i=0; i<iv_vecFeatureBaseNames.size(); ++i) {
        os << "   " << i << ": " << iv_vecFeatureBaseNames[i] << endl;
      }

      os << "TYPE_NAMES:" << endl;
      for (i=0; i<iv_vecTypeNames.size(); ++i) {
        os << "   " << i << ": " << iv_vecTypeNames[i] << endl;
      }

      os << "FEATURENUMBER:" << endl;
      for (i=0; i<iv_vecFeatureNumber.size(); ++i) {
        os << "   " << i << ": " << iv_vecFeatureNumber[i] << "  (" << iv_vecTypeNames[i] << ")" << endl;
      }

      os << "FEATUREOFFSET: " << endl;
      for (i=0; i<iv_vecFeatureOffset.size(); ++i) {
        os << "   " << i << ": " << iv_vecFeatureOffset[i] << "  (" << iv_vecFeatureBaseNames[i] << ")" << endl;
      }

      os << "APPROP:" << endl;
      os << "   ";
      for (i=0; i<iv_vecFeatureBaseNames.size(); ++i) {
        os << i << "  ";
      }
      os << endl;
      for (i=0; i<iv_vecApprop.size(); ++i) {
        os << i << ": ";
        for (j=0; j<iv_vecApprop[i].size(); ++j) {
          os << iv_vecApprop[i][j] << "  ";
        }
        os << endl;
      }

      os << "TYPEFEATUREOFFSET:" << endl;
      for (i=0; i<iv_vecTypeFeatureOffsetMapping.size(); ++i) {
        os << i << ": ";
        for (j=0; j<iv_vecTypeFeatureOffsetMapping[i].size(); ++j) {
          os << iv_vecTypeFeatureOffsetMapping[i][j] << " ";
        }
        os << endl;
      }

    }

#else
    bool TypeSystem::debugIsConsistent() const {
      cerr << "WARNING: uima::lowlevel::TypeSystem::debugIsConsistent is unavailable in release builds" << endl;
      return true;
    }
    void TypeSystem::print(ostream& os) const {
      os << endl << "WARNING: Can only print the TypeSystem with a debug uima library" << endl;
      ;
    }
#endif

  }
}
/* ----------------------------------------------------------------------- */



