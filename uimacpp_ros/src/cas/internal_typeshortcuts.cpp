/** \file internal_typeshortcuts.cpp .
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
#include <uima/internal_typeshortcuts.hpp>
#include <uima/lowlevel_typesystem.hpp>
#include <uima/typesystem.hpp>
#include <uima/cas.hpp>
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
    bool areTypeShortcutsCorrect(uima::lowlevel::TypeSystem const & crTypeSystem) {
      bool bResult = true;
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_INTEGER) == gs_tyIntegerType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_FLOAT) == gs_tyFloatType);
      assert( bResult );

      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_STRING) == gs_tyStringType);
      assert( bResult );

      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_ARRAY_BASE) == gs_tyArrayBaseType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_FS_ARRAY) == gs_tyFSArrayType);
      assert( bResult );

      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_LIST_BASE) == gs_tyListBaseType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_FS_LIST) == gs_tyFSListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_EMPTY_FS_LIST) == gs_tyEListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_NON_EMPTY_FS_LIST) == gs_tyNEListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getFeatureByBaseName(gs_tyNEListType, CAS::FEATURE_BASE_NAME_HEAD) == gs_tyHeadFeature);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getFeatureByBaseName(gs_tyNEListType, CAS::FEATURE_BASE_NAME_TAIL) == gs_tyTailFeature);
      assert( bResult );
      bResult = bResult && (crTypeSystem.isAppropriateFeature( gs_tyNEListType, gs_tyHeadFeature ) );
      assert( bResult );
      bResult = bResult && (crTypeSystem.isAppropriateFeature( gs_tyNEListType, gs_tyTailFeature ) );
      assert( bResult );

      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_FLOAT_LIST) == gs_tyFloatListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_EMPTY_FLOAT_LIST) == gs_tyEFloatListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_NON_EMPTY_FLOAT_LIST) == gs_tyNEFloatListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getFeatureByBaseName(gs_tyNEFloatListType, CAS::FEATURE_BASE_NAME_HEAD) == gs_tyFloatHeadFeature);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getFeatureByBaseName(gs_tyNEFloatListType, CAS::FEATURE_BASE_NAME_TAIL) == gs_tyFloatTailFeature);
      assert( bResult );
      bResult = bResult && (crTypeSystem.isAppropriateFeature( gs_tyNEFloatListType, gs_tyFloatHeadFeature ) );
      assert( bResult );
      bResult = bResult && (crTypeSystem.isAppropriateFeature( gs_tyNEFloatListType, gs_tyFloatTailFeature ) );
      assert( bResult );
      assert( bResult );

      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_INTEGER_LIST) == gs_tyIntListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_EMPTY_INTEGER_LIST) == gs_tyEIntListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_NON_EMPTY_INTEGER_LIST) == gs_tyNEIntListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getFeatureByBaseName(gs_tyNEIntListType, CAS::FEATURE_BASE_NAME_HEAD) == gs_tyIntHeadFeature);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getFeatureByBaseName(gs_tyNEIntListType, CAS::FEATURE_BASE_NAME_TAIL) == gs_tyIntTailFeature);
      assert( bResult );
      bResult = bResult && (crTypeSystem.isAppropriateFeature( gs_tyNEIntListType, gs_tyIntHeadFeature ) );
      assert( bResult );
      bResult = bResult && (crTypeSystem.isAppropriateFeature( gs_tyNEIntListType, gs_tyIntTailFeature ) );
      assert( bResult );

      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_STRING_LIST) == gs_tyStringListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_NON_EMPTY_STRING_LIST) == gs_tyNEStringListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_EMPTY_STRING_LIST) == gs_tyEStringListType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getFeatureByBaseName(gs_tyNEStringListType, CAS::FEATURE_BASE_NAME_HEAD) == gs_tyStringHeadFeature);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getFeatureByBaseName(gs_tyNEStringListType, CAS::FEATURE_BASE_NAME_TAIL) == gs_tyStringTailFeature);
      assert( bResult );
      bResult = bResult && (crTypeSystem.isAppropriateFeature( gs_tyNEStringListType, gs_tyStringHeadFeature ) );
      assert( bResult );
      bResult = bResult && (crTypeSystem.isAppropriateFeature( gs_tyNEStringListType, gs_tyStringTailFeature ) );
      assert( bResult );

      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_ANNOTATION_BASE) == gs_tyAnnotationBaseType);
      bResult = bResult && (crTypeSystem.getTypeByName(CAS::TYPE_NAME_ANNOTATION) == gs_tyAnnotationType);
      assert( bResult );
      bResult = bResult && (crTypeSystem.getFeatureByBaseName(gs_tyAnnotationType, uima::CAS::FEATURE_BASE_NAME_BEGIN) == gs_tyBeginPosFeature);
      assert( bResult );
      bResult = bResult && (crTypeSystem.isAppropriateFeature( gs_tyAnnotationType, gs_tyBeginPosFeature ) );
      assert( bResult );
      bResult = bResult && (crTypeSystem.getFeatureByBaseName(gs_tyAnnotationType, uima::CAS::FEATURE_BASE_NAME_END) == gs_tyEndPosFeature);
      assert( bResult );
      bResult = bResult && (crTypeSystem.isAppropriateFeature( gs_tyAnnotationType, gs_tyEndPosFeature ) );
      assert( bResult );

      //TODO:: upadate for new primitive types and update for Sofa , AnnotatioBase types
      return bResult;
    }
  }
}


/* ----------------------------------------------------------------------- */



