#ifndef UIMA_OOTYPESYSTEM_HPP
#define UIMA_OOTYPESYSTEM_HPP
/** \file typesystem.hpp .
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

    \brief Contains class uima::Type uima::Feature and uima::TypeSystem

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp>

#include <vector>
#include <utility>

#include <uima/lowlevel_typedefs.hpp>
#include <uima/casexception.hpp>
#include <uima/unistrref.hpp>
#include <uima/internal_typeshortcuts.hpp>


#include "unicode/unistr.h"

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define UIMA_NAMESPACE_SEPARATOR "."
#define UIMA_NAMESPACE_SEPARATOR_CHAR '.'

#define UIMA_TYPE_FEATURE_SEPARATOR ":"
#define UIMA_TYPE_FEATURE_SEPARATOR_CHAR ':'

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class CAS;
  class Type;
  class TypeSystem;
  class XCASDeserializerHandler;
  class XmiDeserializerHandler;
  namespace internal {
    class FSPromoter;
  }
  namespace lowlevel {
    class TypeSystem;
    class DefaultFSIterator;
  }

}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  UIMA_EXC_CLASSDECLARE(TypeSystemAlreadyCommittedException, CASException);
  UIMA_EXC_CLASSDECLARE(InvalidFSFeatureObjectException, CASException);

  /**
   * This class represents a feature in the type hierarchy of the type system.
   */
  class UIMA_LINK_IMPORTSPEC Feature {
    friend class internal::FSPromoter;
  private:
    lowlevel::TyFSFeature iv_tyFeature;
    uima::lowlevel::TypeSystem * iv_typeSystem;
    Feature(lowlevel::TyFSFeature tyFeature, uima::lowlevel::TypeSystem & typeSystem);
    void checkValidity() const;

  public:
    Feature();
    bool isValid() const;

    bool operator==(Feature const & crOther) const;
    bool operator!=(Feature const & crOther) const;

    /**
     * get the name of the feature.
     * @throws InvalidFSFeatureObjectException
     */
    UnicodeStringRef getName() const;

    /**
     * get the creator ID of the feature.
     * See @link CreatorIDs separate section @endlink for details
     * @throws InvalidFSFeatureObjectException
     */
    UnicodeStringRef getCreatorID() const;

    /**
     * get the type where this feature was introduced.
     * @param rResult output parameter
     * @throws InvalidFSFeatureObjectException
     */
    void getIntroType(Type & rResult) const;

    /**
     * get the range (value type) of this feature
     * @param result output parameter
     * @throws InvalidFSFeatureObjectException
     */
    void getRangeType(Type & result) const;

    /**
     * get the type system this feature lives in.
     * @throws InvalidFSFeatureObjectException
     */
    uima::TypeSystem const & getTypeSystem() const;

	bool isMultipleReferencesAllowed() const;
  private:
    /* taph 03.12.2002:  do not use. This is a compiler test case*/
    Type getIntroType() const;
    Type getRangeType() const;
  };


  UIMA_EXC_CLASSDECLARE(InvalidFSTypeObjectException, CASException);


  /**
   * This class represents a type in the type hierarchy of the type system.
   */
  class UIMA_LINK_IMPORTSPEC Type {
    friend class internal::FSPromoter;
  private:
    lowlevel::TyFSType iv_tyType;
    uima::lowlevel::TypeSystem * iv_typeSystem;

    Type(lowlevel::TyFSType aType, uima::lowlevel::TypeSystem & typeSystem);
    void checkValidity() const;
  public:
    /**
     * Default constructor. Creates an invalid FSType object.
     */
    Type();

    /**
     * @return true if the object is valid
     */
    bool isValid() const;

    bool operator==(Type const & crOther) const;
    bool operator!=(Type const & crOther) const;
    bool operator<(Type const & other) const;

    /**
     * get the name of the type.
     * @throws InvalidFSTypeObjectException
     */
    UnicodeStringRef getName() const;

    /**
     * get the creator ID of the type.
     * See @link CreatorIDs separate section @endlink for details
     * @throws InvalidFSFeatureObjectException
     */
    UnicodeStringRef getCreatorID() const;

    /**
     * get the list of all features appropriate for this type.
     * @param result output parameter
     * @throws InvalidFSTypeObjectException
     */
    void getAppropriateFeatures(std::vector<Feature> & result) const;

    /**
     * return true if a feature f is appropriate for this type.
     * @param f feature to check
     * @throws InvalidFSTypeObjectException
     */
    bool isAppropriateFeature(Feature const & f) const;

    /**
     * Get all the direct subtypes of this type;
     * @throws InvalidFSTypeObjectException
     */
    void getDirectSubTypes(std::vector<Type> & rResult) const;

    /**
     * Get all the subtypes of this type;
     * @throws InvalidFSTypeObjectException
     */
    void getSubTypes(std::vector<Type> & rResult) const;

    /**
     * Get the feature with base name <code>crBaseName</code>.
     * Returns an invalid FSFeature object if no feature with
     * the specified base name is appropriate for this type.
     * @param crBaseName the base name of the feature to be found
     * @throws InvalidFSTypeObjectException
     */
    Feature getFeatureByBaseName(icu::UnicodeString const & crBaseName) const;

    /**
     * @return true if this object subsumes <code>crType</code>.
     * @throws InvalidFSTypeObjectException
     */
    bool subsumes(Type const & crType) const;

    /**
     * get the type system this type lives in.
     * @throws InvalidFSFeatureObjectException
     */
    uima::TypeSystem const & getTypeSystem() const;

    /**
     *
     */
    bool isStringSubType() const;

  };

  /**
   * Class TypeSystem represents all knowledge about types in the
   * FeatureStructure system.
   */
  class UIMA_LINK_IMPORTSPEC TypeSystem {
    friend class uima::CAS;
    friend class uima::lowlevel::TypeSystem;
    friend class uima::XCASDeserializerHandler;
	 friend class uima::XmiDeserializerHandler;
	 friend class XmiWriter;
    friend class lowlevel::DefaultFSIterator;
  protected:
    virtual uima::lowlevel::TypeSystem const & getLowlevelTypeSystem() const = 0;
    TypeSystem();
    //virtual ~TypeSystem();

    bool isFSArrayType(lowlevel::TyFSType tyType) const  {
      return (tyType == uima::internal::gs_tyFSArrayType);
    }



    bool isArrayType(lowlevel::TyFSType tyType) const  {
      return ( tyType == uima::internal::gs_tyIntArrayType )
             || ( tyType == uima::internal::gs_tyFloatArrayType )
             || ( tyType == uima::internal::gs_tyStringArrayType )
             || ( tyType == uima::internal::gs_tyByteArrayType )
             || ( tyType == uima::internal::gs_tyBooleanArrayType )
             || ( tyType == uima::internal::gs_tyShortArrayType )
             || ( tyType == uima::internal::gs_tyLongArrayType )
             || ( tyType == uima::internal::gs_tyDoubleArrayType )
             || (tyType == uima::internal::gs_tyFSArrayType);
    }
    
	 bool isListType(lowlevel::TyFSType tyType) const  {
      return ( tyType == uima::internal::gs_tyFSListType )
             || ( tyType == uima::internal::gs_tyEListType )
             || ( tyType == uima::internal::gs_tyNEListType )
			 || ( tyType == uima::internal::gs_tyIntListType )
		     || ( tyType == uima::internal::gs_tyEIntListType )
		     || ( tyType == uima::internal::gs_tyNEIntListType )
             || ( tyType == uima::internal::gs_tyFloatListType )
             || ( tyType == uima::internal::gs_tyEFloatListType )
             || ( tyType == uima::internal::gs_tyNEFloatListType )
             || ( tyType == uima::internal::gs_tyStringListType )
             || ( tyType == uima::internal::gs_tyEStringListType )
		     || ( tyType == uima::internal::gs_tyNEStringListType );
    }

    bool isFSType(lowlevel::TyFSType tyType) const  {
      return ( !isPrimitive(tyType) && !isArrayType(tyType)
		       && !isListType(tyType) );
    }

  public:

    static const char FEATURE_SEPARATOR;
    static const char NAMESPACE_SEPARATOR;
    virtual ~TypeSystem();

    /**
     * get the TOP type.
     */
    Type getTopType() const;

    /**
     * find a feature by its fully qualified name, i.e., "&lt;Type>:&lt;FeatureBaseName>"
     * @return the found feature, if a feature with that name could not be found, the returned object is not valid
     */
    Feature getFeatureByFullName(icu::UnicodeString const & crName) const;

    /**
     * find a type by its name
     * @return the found type, if a type with that name could not be found, the returned object is not valid
     */
    Type getType(icu::UnicodeString const & crName) const;

    /**
     * get all the types in this FSSystem.
     */
    void getAllTypes(std::vector<Type> & rResult) const;

    /**
     * get all the features in this FSSystem.
     */
    void getAllFeatures(std::vector<Feature> & rResult) const;


    bool isPrimitive(lowlevel::TyFSType tyType) const;
    bool isStringSubType(lowlevel::TyFSType tyType) const;

    /*------------------------------ Static Methods ------------------------------*/

    /**@name Static methods.
    The following static methods can be called on the <TT>TypeSystem</TT>
    class directly.
    */
    //@{

    /**
     * Release contents of vector container allocated by get methods
     * Useful when caller and callee use different heaps, 
     * e.g. when debug code uses a release library.
     */
    static void release(std::vector<uima::Type> & rResult);

    /**
     * Release contents of vector container allocated by get methods
     * Useful when caller and callee use different heaps, 
     * e.g. when debug code uses a release library.
     */
    static void release(std::vector<uima::Feature> & rResult);

  };


}
/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif


