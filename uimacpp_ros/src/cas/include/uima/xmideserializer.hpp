#ifndef UIMA_XMIDESERIALIZER_HPP
#define UIMA_XMIDESERIALIZER_HPP
/** \file xmideserializer.hpp .
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

    \brief Deserializes cas data from XMI format into a CAS

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
#include <uima/xmishareddata.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
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
 * The XmiDeserializer deserializes cas data in XMI format into a CAS.
 * Only FeatureStructures with types that are represented in the input CAS
 * TypeSystem will be deserialized into the CAS.
 */
namespace uima {

  /**
   * The XmiDeserializer class provides static methods to deserializes cas data  
   * in XMI format into a CAS.
   */

  class UIMA_LINK_IMPORTSPEC XmiDeserializer {
  private:


  public:
    /** Constructor **/
    XmiDeserializer();
    /** Destructor **/
    ~XmiDeserializer();

    /**
     * Deserialize given an XMI filename and input CAS 
     */
    static void deserialize(char const * xmifilename, CAS &, bool lenient=false);

    /**
     * Deserialize given an XMI filename icu::UnicodeString and input CAS 
     */
    static void deserialize(icu::UnicodeString & xmifilename, CAS &, bool lenient=false);

    /**
     * Deserialize given an XMI InputSource and input CAS 
     */
    static void deserialize(InputSource const & crInputSource, CAS &, bool lenient=false);
  
    /**
     * Deserialize given an XMI InputSource and input CAS and XmiSerializationSharedData 
     */
    static void deserialize(InputSource const & crInputSource, CAS &, XmiSerializationSharedData & sharedData);
  
	/**
     * Deserialize given an XMI filename and input CAS , and
		 * XmiSerializationData.
     */
    static void deserialize(char const * xmifilename, CAS &, XmiSerializationSharedData & sharedData);

    /**
     * Deserialize given an XMI filename UnicodeString, input CAS, and
		 * XmiSerializationData.
     */
    static void deserialize(icu::UnicodeString & xmifilename, CAS &, XmiSerializationSharedData & sharedData);

  };

}


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif
