/** \file taemetadata.hpp .
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

    \brief Contains uima::MetaDataObject

   Description: Base class for configuration objects

-----------------------------------------------------------------------------


   01/30/2003  Initial creation

-------------------------------------------------------------------------- */
#ifndef UIMA_TAEMETADATA_HPP
#define UIMA_TAEMETADATA_HPP

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/exceptions.hpp>


#include <vector>
#include <algorithm>
namespace uima {
  UIMA_EXC_CLASSDECLARE(DuplicateConfigElemException, uima::Exception);
  UIMA_EXC_CLASSDECLARE(ValidationException, uima::Exception);

  template<class InputIterator, class T>
  size_t countValues( InputIterator it1, InputIterator it2, T val)  {
    size_t uiCount = 0;
    // SUN Workshop Pro only knows the first variant, MS .NET only the second
#if defined(__SUNPRO_CC)
    count(it1, it2, val, uiCount);
#else
    uiCount = count(it1, it2, val);
#endif
    return uiCount;
  }
  /**
  * The base class of all configuration objects.
  **/
  class UIMA_LINK_IMPORTSPEC MetaDataObject {
  public:
    MetaDataObject()
        :iv_bIsModifiable(true) {}

    virtual ~MetaDataObject() {}

    bool isModifiable() const {
      return iv_bIsModifiable;
    }

    /**
    * When this method is called on a #MetaDataObject# that must not be reconfigured after
    * the engine is created, #isModifiable()# will return false
    * and all subsequent calls to setter methods will return #UIMA_ERR_CONFIG_OBJECT_COMMITED#.
    * Subclasses of such MetaDataObjects must override this method to ensure that
    * commit is propagated to its members.
    * Note that configuration parameter values can be reconfigured after the engine is created.
    * Hence, the #extractValue# methods of the #AnnotatorContext# can be called anytime.
    **/
    virtual void commit() {
      iv_bIsModifiable = false;
    }

  protected:
    bool iv_bIsModifiable;
  };
}
#endif
