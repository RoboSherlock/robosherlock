#ifndef UIMA_INTERNAL_TYPESHORTCUTS_HPP
#define UIMA_INTERNAL_TYPESHORTCUTS_HPP
/** \file internal_typeshortcuts.hpp .
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

   Description: This file contains shortcuts for builtin and
                predefined types/features

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/lowlevel_typedefs.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {
    /**
     * Here are some hardcoded shortcuts to CAS builtin types.
     * The function areTypeShortcutsCorrect() below checks that they are
     * in fact correct.
     */
    const uima::lowlevel::TyFSType gs_tyIntegerType = 2;
    const uima::lowlevel::TyFSType gs_tyFloatType = 3;
    const uima::lowlevel::TyFSType gs_tyStringType = 4;

    const uima::lowlevel::TyFSType gs_tyArrayBaseType = 5;
    const uima::lowlevel::TyFSType gs_tyFSArrayType = 6;
    const uima::lowlevel::TyFSType gs_tyFloatArrayType = 7;
    const uima::lowlevel::TyFSType gs_tyIntArrayType = 8;
    const uima::lowlevel::TyFSType gs_tyStringArrayType = 9;

    const uima::lowlevel::TyFSType gs_tyListBaseType = 10;
    const uima::lowlevel::TyFSType gs_tyFSListType = 11;
    const uima::lowlevel::TyFSType gs_tyEListType = 12;
    const uima::lowlevel::TyFSType gs_tyNEListType = 13;
    const uima::lowlevel::TyFSFeature                 gs_tyHeadFeature = 1;
    const uima::lowlevel::TyFSFeature                 gs_tyTailFeature = 2;

    const uima::lowlevel::TyFSType gs_tyFloatListType = 14;
    const uima::lowlevel::TyFSType gs_tyEFloatListType = 15;
    const uima::lowlevel::TyFSType gs_tyNEFloatListType = 16;
    const uima::lowlevel::TyFSFeature                 gs_tyFloatHeadFeature = 3;
    const uima::lowlevel::TyFSFeature                 gs_tyFloatTailFeature = 4;

    const uima::lowlevel::TyFSType gs_tyIntListType = 17;
    const uima::lowlevel::TyFSType gs_tyEIntListType = 18;
    const uima::lowlevel::TyFSType gs_tyNEIntListType = 19;
    const uima::lowlevel::TyFSFeature                 gs_tyIntHeadFeature = 5;
    const uima::lowlevel::TyFSFeature                 gs_tyIntTailFeature = 6;

    const uima::lowlevel::TyFSType gs_tyStringListType = 20;
    const uima::lowlevel::TyFSType gs_tyEStringListType = 21;
    const uima::lowlevel::TyFSType gs_tyNEStringListType = 22;
    const uima::lowlevel::TyFSFeature                         gs_tyStringHeadFeature = 7;
    const uima::lowlevel::TyFSFeature                         gs_tyStringTailFeature = 8;

    // Extended primitive types
    const uima::lowlevel::TyFSType gs_tyBooleanType = 23;
    const uima::lowlevel::TyFSType gs_tyByteType = 24;
    const uima::lowlevel::TyFSType gs_tyShortType = 25;
    const uima::lowlevel::TyFSType gs_tyLongType = 26;
    const uima::lowlevel::TyFSType gs_tyDoubleType = 27;
    const uima::lowlevel::TyFSType gs_tyBooleanArrayType = 28;
    const uima::lowlevel::TyFSType gs_tyByteArrayType = 29;
    const uima::lowlevel::TyFSType gs_tyShortArrayType = 30;
    const uima::lowlevel::TyFSType gs_tyLongArrayType = 31;
    const uima::lowlevel::TyFSType gs_tyDoubleArrayType = 32;

    const uima::lowlevel::TyFSType gs_tySofaType = 33;
    const uima::lowlevel::TyFSFeature                         gs_tySofaNumFeature = 9;
    const uima::lowlevel::TyFSFeature                         gs_tySofaIDFeature = 10;
    const uima::lowlevel::TyFSFeature                         gs_tySofaMimeFeature = 11;
    const uima::lowlevel::TyFSFeature                         gs_tySofaArrayFeature = 12;
    const uima::lowlevel::TyFSFeature                         gs_tySofaStringFeature = 13;
    const uima::lowlevel::TyFSFeature                         gs_tySofaURIFeature = 14;

    const uima::lowlevel::TyFSType gs_tyAnnotationBaseType = 34;
    const uima::lowlevel::TyFSType gs_tyAnnotationType = 35;
    const uima::lowlevel::TyFSFeature                         gs_tySofaRefFeature = 15;
    const uima::lowlevel::TyFSFeature                         gs_tyBeginPosFeature = 16;
    const uima::lowlevel::TyFSFeature                         gs_tyEndPosFeature = 17;

  }  // namespace internal
}  // namespace uima

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  namespace lowlevel {
    class TypeSystem;
  }
}


/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  namespace internal {
    /**
     * This function returns true if all the shortcuts are correct.
     * Called in CAS::commitTypeSystem().
     */
    bool areTypeShortcutsCorrect(uima::lowlevel::TypeSystem const &);
  }
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

