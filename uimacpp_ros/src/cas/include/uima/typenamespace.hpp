#ifndef UIMA_TYPENAMESPACE_HPP
#define UIMA_TYPENAMESPACE_HPP
/** \file typenamespace.hpp .
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

    \brief  Contains class uima::TypeNameSpace and uima::TypeNameSpaceImport

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/typesystem.hpp>
#include "unicode/unistr.h"
#include <vector>
/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */


namespace uima {

  /// The constant we use to seperate namespace parts: A period
  const char TYPENAMESPACE_SEP = '.';

  /**
   * An accessor object that gives access to all types that share a common
   * namespace.
   * For example if a TypeNameSpace object is created for the
   * namespace "uima.tt"
   * (<code> TypeNameSpace ttNameSpace(ts, "uima.tt");</code>)
   * you can then use this object to get type objects from types names
   * by using only short names e.g.
   * <code>Type t = ttNameSpace.getType("Lemma");</code>
   * would be equivalent to
   * <code>Type t = ts.getType("uima.tt.Lemma);</code>
   */
  class UIMA_LINK_IMPORTSPEC TypeNameSpace {
  private:
    icu::UnicodeString iv_usName;
    uima::TypeSystem const * iv_cpTypeSystem;
  public:
    /**
     * Construct a TypeNameSpace object by passing it a reference
     * the type system it lives in and the name of the namespace
     * it should represent.
     */
    TypeNameSpace(uima::TypeSystem const & crTypeSystem, icu::UnicodeString const & crName);

    /**
     * Find a type object by it's short name.
     * Implicitly this just prepends the namespace name to crTypeBaseName
     */
    Type getType(icu::UnicodeString const & crTypeBaseName) const;

    /// Return the name of this namespace object
    icu::UnicodeString const & getName() const;

    /// Return a container with type objects for all types in this namespace
    void getAllTypes(std::vector<Type> & rResult) const;

    /// Return the typesystem this namespace lives in
    TypeSystem const & getTypeSystem() const;
  };


  /**
   * This class is usefull to bundle several TypeNameSpace objects into
   * a lookup sequence that can be used to lookup unqualified type names.
   * The TypeNameSpaceImport object will try to find an unqualified type
   * name in each TypeNameSpace objects that has been added to it.
   * The lookup will be done in the sequence in which the TypeNameSpace
   * objects have been added.
   */
  class UIMA_LINK_IMPORTSPEC TypeNameSpaceImport {
  private:
    std::vector<TypeNameSpace> iv_vecImports;
    uima::TypeSystem const * iv_cpTypeSystem;
  public:
    /**
     * Construct a TypeNameSpaceImport object from an type system
     */
    TypeNameSpaceImport( uima::TypeSystem const & );

    /**
     * Add a TypeNameSpace object to this TypeNameSpaceImport object.
     * Note: The order of the addNameSpace() calls will be relevant
     * for a later lookup using getType()
     */
    void addNameSpace(TypeNameSpace const & crNamespace);

    /**
     * try to get the type with the specified relative name.
     * If there is a name conflict, false is returned, true otherwise.
     * @param crRelativeTypeName  input: not fully specified type name
     * @param rResult             output parameter
     */
    bool getType(icu::UnicodeString const & crRelativeTypeName, Type & rResult) const;

    /**
     * Return a container with type objects for all types of all namespace
     * that have been added to this TypeNameSpaceImport object
     */
    void getAllTypes(std::vector<Type> & rResult) const;

    /// Return the typesystem this TypeNameSpaceImport object lives in
    TypeSystem const & getTypeSystem() const;
  };
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

