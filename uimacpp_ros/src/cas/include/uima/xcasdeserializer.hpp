#ifndef UIMA_XCASDESERIALIZER_HPP
#define UIMA_XCASDESERIALIZER_HPP
/** \file xcasdeserializer.hpp .
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

    \brief Deserializes cas data from XCAS format into a CAS

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include <iostream>
#include <vector>
#include <set>

#include "xercesc/framework/LocalFileInputSource.hpp"
#include "xercesc/framework/MemBufInputSource.hpp"

#include <uima/featurestructure.hpp>
#include <uima/lowlevel_typedefs.hpp>
#include <uima/typesystem.hpp>


/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class AnnotatorContext;
  namespace lowlevel {
    class IndexABase;
    class FSHeap;
    class TypeSystem;
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/**
 * The XCASDeserializer deserializes cas data in XCAS format into a CAS.
 * Only FeatureStructures with types that are represented in the input CAS
 * TypeSystem will be deserialized into the CAS.  At this time support for
 * out of typesystem data is not supported.
 */
namespace uima {

  /**
   * The XCASDeserializer class provides static methods to deserializes cas data in XCAS 
   * format into a CAS.
   * Only FeatureStructures with types that are represented in the input CAS
   * TypeSystem will be deserialized into the CAS.  At this time support for
   * out of typesystem data is not supported.
   */

  class UIMA_LINK_IMPORTSPEC XCASDeserializer {
  private:


  public:
    /** Constructor **/
    XCASDeserializer();
    /** Destructor **/
    ~XCASDeserializer();

    /**
     * Deserialize given an XCAS filename and input CAS 
     */
    static void deserialize(char const * xcasfilename, CAS &);

    /**
     * Deserialize given an XCAS filename UnicodeString and input CAS 
     */
    static void deserialize(UnicodeString & xcasfilename, CAS &);

    /**
     * Deserialize given an XCAS filename, input CAS, and the 
     * AnnotatorContext.  The uima::AnnotatorContext will be used
     * for Sofa name mapping. 
     */
    static void deserialize(char const * xcasfilename, CAS &, uima::AnnotatorContext * const ctx);

    /**
     * Deserialize given an XCAS InputSource and input CAS 
     */
    static void deserialize(InputSource const & crInputSource, CAS &);

    /**
     * Deserialize given an XCAS InputSource, input CAS, and the 
     * AnnotatorContext.  The uima::AnnotatorContext will be used
     * for Sofa name mapping. 
     */
    static void deserialize(InputSource const & crInputSource, CAS &, uima::AnnotatorContext * const ctx);
  };

}


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif
