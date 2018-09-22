/** \file typenamespace.cpp .
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

#include <uima/typenamespace.hpp>

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
  TypeNameSpace::TypeNameSpace(uima::TypeSystem const & crTypeSystem, icu::UnicodeString const & crName)
      : iv_usName(crName),
      iv_cpTypeSystem( & crTypeSystem ) {}

  Type TypeNameSpace::getType(icu::UnicodeString const & crTypeBaseName) const {
    icu::UnicodeString typeName(iv_usName);
    typeName.append((UChar) TYPENAMESPACE_SEP);
    typeName.append(crTypeBaseName);
    return iv_cpTypeSystem->getType(typeName);
  }

  icu::UnicodeString const & TypeNameSpace::getName() const {
    return iv_usName;
  }

  void TypeNameSpace::getAllTypes(vector<Type> & rResult) const {
    rResult.clear();
    vector<Type> allTypes;
    iv_cpTypeSystem->getAllTypes(allTypes);
    size_t i;
    for (i=0; i<allTypes.size(); ++i) {
      Type t = allTypes[i];
      uima::UnicodeStringRef us = t.getName();
      int ix = us.lastIndexOf((UChar) TYPENAMESPACE_SEP);
      if (ix != -1) {
        icu::UnicodeString nameSpace;
        us.extract(0, ix, nameSpace);
        if (nameSpace == iv_usName) {
          rResult.push_back(t);
        }
      }
    }
  }

  TypeSystem const & TypeNameSpace::getTypeSystem() const {
    assert( EXISTS(iv_cpTypeSystem) );
    return *iv_cpTypeSystem;
  }


  ////////////////////////////////////////////////////////////


  TypeNameSpaceImport::TypeNameSpaceImport( uima::TypeSystem const & crTypeSystem)
      : iv_cpTypeSystem( & crTypeSystem) {}

  void TypeNameSpaceImport::addNameSpace(TypeNameSpace const & crNamespace) {
    iv_vecImports.push_back(crNamespace);
  }

  bool TypeNameSpaceImport::getType(icu::UnicodeString const & crRelativeTypeName, Type & rResult) const {
    rResult = Type();
    size_t i;
    for (i=0; i<iv_vecImports.size(); ++i) {
      TypeNameSpace nameSpace = iv_vecImports[i];

      Type t = nameSpace.getType(crRelativeTypeName);
      if (t.isValid()) {
        // if the type was found previously
        if (rResult.isValid()) {
          rResult = Type();
          return false;
        }
        rResult = t;
      }
    }
    return true;
  }

  void TypeNameSpaceImport::getAllTypes(vector<Type> & rResult) const {
    rResult.clear();
    size_t i,j;
    for (i=0; i<iv_vecImports.size(); ++i) {
      TypeNameSpace nameSpace = iv_vecImports[i];
      vector<Type> types;
      nameSpace.getAllTypes(types);
      for (j=0; j<types.size(); ++j) {
        rResult.push_back(types[j]);
      }
    }
  }


  TypeSystem const & TypeNameSpaceImport::getTypeSystem() const {
    assert( EXISTS(iv_cpTypeSystem) );
    return *iv_cpTypeSystem;
  }



}

/* ----------------------------------------------------------------------- */



