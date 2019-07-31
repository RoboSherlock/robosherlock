#ifndef UIMA_CASWRITER_ABASE_HPP
#define UIMA_CASWRITER_ABASE_HPP
/** \file caswriter_abase.hpp .
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

   \brief An abstract base class for CAS writers

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp>

#include <iostream>
#include <uima/types.h>
#include <uima/unistrref.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class CAS;
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  /**
   * This is an abstract base class for CAS writers, i.e., a class which
   * can write a CAS to an ostrema. Examples are 
   * XCAS writers and XML dump writers (dump annotator).
   */
  class UIMA_LINK_IMPORTSPEC CASWriterABase {
  protected:
    CAS const & iv_cas;
    bool iv_addDocument;
  public:
    CASWriterABase(CAS const & crCAS, bool bAddDocument);
    virtual ~CASWriterABase();

    virtual void write(std::ostream& os) = 0;
  };


  class UIMA_LINK_IMPORTSPEC XMLWriterABase : public CASWriterABase {
  public:
    static void normalize(UnicodeStringRef const & in, icu::UnicodeString& out);
  public:
    XMLWriterABase(CAS const & crCAS, bool bAddDocBuffer);
    virtual ~XMLWriterABase();

    virtual void write(std::ostream& os) = 0;
  };

}


/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif
