/** \file sofastream.hpp .
-----------------------------------------------------------------------------



   \brief  Contains class uima::CASPool

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

-------------------------------------------------------------------------- */
#ifndef UIMA_CASPOOL_HPP
#define UIMA_CASPOOL_HPP

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>
#include <uima/taespecifier.hpp>
#include <uima/cas.hpp>
#include <uima/internal_casimpl.hpp>
#include <uima/casdefinition.hpp>

#include <vector>

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class AnalysisEngineDescription;
  //class internal::CASDefinition;
}


/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
namespace uima {

  /**------------------------------------------------
  * CASPool 
  * This class manages  pool of CAS objects
  *------------------------------------------------
  */

  UIMA_EXC_CLASSDECLARE(CASPoolException, uima::Exception);

  class UIMA_LINK_IMPORTSPEC CASPool {
  private:
    std::vector<CAS *> iv_vecAllInstances;
    std::vector<CAS *> iv_vecFreeInstances;
    size_t    iv_numInstances;
    uima::internal::CASDefinition * iv_pCasDef;

  public:

    /** Constructor
    * Creates the specified number of CAS instances based on CAS definition
    * as specified in the TAE specifier.
    */
    CASPool(const AnalysisEngineDescription & taeSpec, size_t numInstances);

    /** Destructor */
    ~CASPool(void);

    /**
     * Check out a free CAS intance.
    * @returns a pointer to a CAS or null if there are no free CASs available.
    */
    CAS & getCAS(void);


    /**
        * Checks in a CAS to the pool.  This automatically calls the 
     * {@link CAS#reset()} method, to ensure that when the CAS is later
        * retrieved from the pool it will be ready to use.
        *
        * @param aCas the Cas to release
        */
    void releaseCAS(CAS & );
  };

}

#endif
