/** \file sofaid.hpp .
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

    \brief  Class to support sofaMapping

   Description: A SofaID object contains the handle to a particular Sofa in
                the CAS. An SofaId is obtained by calling
                uima::AnnotatorContext::mapToSofaID().


-----------------------------------------------------------------------------


   10/01/2004  Initial creation

-------------------------------------------------------------------------- */
#ifndef UIMA_SOFAID_HPP
#define UIMA_SOFAID_HPP

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>

#include <uima/taemetadata.hpp>
#include <vector>

namespace uima {

  /** A SofaID object contains the handle to a particular Sofa in the CAS.
      A SofaID is obtained by calling uima::AnnotatorContext::mapToSofaID(). 
  */
  class UIMA_LINK_IMPORTSPEC SofaID {
  public:

    //SofaID()
    //iv_sofaId(), iv_componentSofaName()
    //{
    //
    //}
    //SofaID(icu:UnicodeString & sofaid, icu:UnicodeString & componentsofaname) {
    //    iv_sofaid = sofaid;
    //    iv_componentSofaName = componentsofaname;
    //}

    /**
     * Set the Sofa ID string, the  string representing the Sofa name in the CAS.
     */
    TyErrorId setSofaId(const icu::UnicodeString & sofaid) {
      iv_sofaId=sofaid;
      return UIMA_ERR_NONE;
    }

    /**
     * Get the Sofa ID, the  string representing the Sofa name in the CAS.
     */
    const icu::UnicodeString & getSofaId() const {
      return iv_sofaId;
    }

    /**
     * Set the component Sofa name, the Sofa name as known to the component.
     */
    TyErrorId setComponentSofaName(const icu::UnicodeString & componentSofaName) {
      iv_componentSofaName=componentSofaName;
      return UIMA_ERR_NONE;
    }

    /**
     * Get the component Sofa name, the Sofa name as knnwn to the component.
     */
    const icu::UnicodeString & getComponentSofaName() const {
      return iv_componentSofaName;
    }



  private:

    SofaID & operator=(const SofaID & crOther);

    icu::UnicodeString iv_sofaId;
    icu::UnicodeString iv_componentSofaName;

  };
}

#endif
