/** \file importdescription.hpp .
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

    \brief  Contains class uima::ImportDescription

   Description:

-----------------------------------------------------------------------------


   10/01/2004  Initial creation

-------------------------------------------------------------------------- */
#ifndef UIMA_IMPORTDESC_HPP
#define UIMA_IMPORTDESC_HPP

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>

#include <uima/taemetadata.hpp>
#include <uima/resmgr.hpp>
#include <vector>

namespace uima {

  /**
   * A Import object represents a pointer to a descriptor file to be imported. 
   * These are currently used to import type systems, indexes, and type
   * priorities, delegate analysis engine descriptors.  
   * Imports may be by location (relative URL) or name (a Java-style compound name, 
   * looked up in the classpath), but not both.
   **/
  class UIMA_LINK_IMPORTSPEC ImportDescription: public MetaDataObject {
  public:

    /**
     * Constructor
     */
    ImportDescription()
        :MetaDataObject(), iv_location(), iv_name() {
    }


    /**
     *  Sets the location of this import's target.
     *  @param aUri a URI specifying the location of this import's target.
     */
    TyErrorId setLocation(const icu::UnicodeString & aUri) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_location=aUri;
      return UIMA_ERR_NONE;
    }

    /**
     * Gets the location of this import's target.
    * 
    * @return a URI specifying the location of this import's target.
     */
    const icu::UnicodeString & getLocation() const {
      return iv_location;
    }


    /**
     * a Java-style compound name which specifies the target of this import.
    * This will be located by appending ".xml" to the name and searching the classpath.
     */

    TyErrorId setName(const icu::UnicodeString & aname) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_name=aname;
      return UIMA_ERR_NONE;
    }

    /**
     * a Java-style compound name which specifies the target of this import.
    * This will be located by appending ".xml" to the name and searching the classpath.
     */

    const icu::UnicodeString & getName() const {
      return iv_name;
    }

    const icu::UnicodeString  findAbsoluteUrl (icu::UnicodeString const & lastFilename) {
      return ResourceManager::resolveFilename(iv_location, lastFilename);
    }

  private:

    ImportDescription & operator=(const ImportDescription & crOther);

    icu::UnicodeString iv_location;
    icu::UnicodeString iv_name;


  };


}

#endif
