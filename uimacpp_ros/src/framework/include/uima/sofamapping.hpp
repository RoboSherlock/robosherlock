/** \file sofamapping.hpp .
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

    \brief  Contains class uima::SofaMapping

   Description: A SofaMapping object represents mapping of a Sofa name assigned
                by a component to a Sofa name assigned by an aggregate which could
                be either an aggregate TAE or a CPE. This interface provides methods
                to set the attributes that define a mapping.

                Sofa Name mapping is required to connect the output Sofas from
                one component to the input Sofa of another component.
                This mapping enables the uima::UimaConext.mapToSofaID() method to
                return the SofaID that serves as the handle to a particular Sofa
                in the CAS.

                If the component Sofa Name is not set, it defaults to the Sofa Name
                of the Default Text Sofa. If no mapping is provided, a Sofa Name
                maps to itself.


-----------------------------------------------------------------------------


   10/01/2004  Initial creation

-------------------------------------------------------------------------- */
#ifndef UIMA_SOFAMAPPING_HPP
#define UIMA_SOFAMAPPING_HPP

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>

#include <uima/taemetadata.hpp>
#include <vector>

namespace uima {

  /**
      A SofaMapping object represents mapping of a Sofa name assigned 
               by a component to a Sofa name assigned by an aggregate which could 
               be either an aggregate TAE or a CPE. This interface provides methods 
               to set the attributes that define a mapping. 
               <br><br> 
               Sofa Name mapping is required to connect the output Sofas from 
               one component to the input Sofa of another component. 
               This mapping enables the uima::UimaConext.mapToSofaID() method to 
               return the SofaID that serves as the handle to a particular Sofa 
               in the CAS.
               <br><br> 
               If the component Sofa Name is not set, it defaults to the Sofa Name 
               of the Default Text Sofa. If no mapping is provided, a Sofa Name
               maps to itself.  
  **/
  class UIMA_LINK_IMPORTSPEC SofaMapping: public MetaDataObject {
  public:

    /**
     * Constructor
     */
    SofaMapping()
        :MetaDataObject(), iv_componentKey(), iv_componentSofaName(), iv_aggregateSofaName() {
    }


    /**
     * Set the name of the component to which this mapping applies.
     */
    TyErrorId setComponentKey(const icu::UnicodeString & componentKey) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_componentKey=componentKey;
      return UIMA_ERR_NONE;
    }

    /**
     * Get the name of the component to which this mapping applies.
     */
    const icu::UnicodeString & getComponentKey() const {
      return iv_componentKey;
    }


    /**
     * Set the Sofa name as known to the component.
     */

    TyErrorId setComponentSofaName(const icu::UnicodeString & componentSofaName) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_componentSofaName=componentSofaName;
      return UIMA_ERR_NONE;
    }

    /**
     * Get the Sofa name as known to the component.
     */

    const icu::UnicodeString & getComponentSofaName() const {
      return iv_componentSofaName;
    }

    /**
     * set the Sofa name assigned by the aggregate.
     */

    TyErrorId setAggregateSofaName(const icu::UnicodeString & aggregateSofaName) {
      if (! isModifiable()) {
        return UIMA_ERR_CONFIG_OBJECT_COMITTED;
      }
      iv_aggregateSofaName=aggregateSofaName;
      return UIMA_ERR_NONE;
    }

    /**
     * Get the Sofa name assigned by the aggregate.
     */
    const icu::UnicodeString & getAggregateSofaName() const {
      return iv_aggregateSofaName;
    }

  private:

    SofaMapping & operator=(const SofaMapping & crOther);

    icu::UnicodeString iv_componentKey;
    icu::UnicodeString iv_componentSofaName;
    icu::UnicodeString iv_aggregateSofaName;

  };
}

#endif
