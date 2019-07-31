/** \file sofastream.hpp .
-----------------------------------------------------------------------------



   \brief  Contains class uima::CASIterator

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

  \brief  Contains class uima::CasIterator
-------------------------------------------------------------------------- */
#ifndef UIMA_CASITERATOR_HPP
#define UIMA_CASITERATOR_HPP

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>
#include <uima/engine.hpp>
#include <uima/cas.hpp>

#include <vector>

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class AnalysisEngine;
  class TextAnalysisEngine;
  namespace internal {
    class EngineBase;
  }
}


/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
namespace uima {

  /**------------------------------------------------
  * CASIterator
  * 
  *------------------------------------------------
  */

  UIMA_EXC_CLASSDECLARE(CASIteratorException, uima::Exception);

  class UIMA_LINK_IMPORTSPEC CASIterator {
    friend class uima::AnalysisEngine;
    friend class uima::TextAnalysisEngine;
    friend class uima::internal::EngineBase;
  private:
    AnalysisEngine * iv_engine;

  protected:




    /** Constructor
    * Creates the specified number of CAS instances based on CAS definition
    * as specified in the TAE specifier.
    */
    CASIterator(AnalysisEngine * pEngine);
  public:
    /** Destructor */
    ~CASIterator(void);


    /**
    * Checks if there are more CASes to be returned by the iterator.
    * 
    * @return true if there are more CASes to be returned, false if not
    * 
    */
    bool hasNext(void);


    /**
     * Gets the next CAS from the iterator.
    * @returns a pointer to a CAS.
    */
    CAS & next(void);


    /**
        * Releases any CASes owned by this CasIterator.  You only need to call this method 
     * if you stop using a CasIterator before you have iterated all the way through.
        */
    void release(void);
  };

}

#endif
