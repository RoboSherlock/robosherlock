#ifndef UIMA_DOTTYPESYSTEMWRITER_HPP
#define UIMA_DOTTYPESYSTEMWRITER_HPP
/** \file dottypesystemwriter.hpp .
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

    \brief Contains class DotTypeSystemWriter

   Description: A class which outputs the type system in the dot graphical language.

-----------------------------------------------------------------------------


   09/20/2002  Initial creation

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <iostream>
#include <uima/types.h>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class CAS;
  class Type;
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  /**
   * A class which outputs the type system in the dot graphical
   * language.
   * The output must be compiled with ATTs dot tool from the Graphwiz
   * package (see http://www.graphviz.org)
   * E.g. call <code>dot -Tgif types.dot -o types.gif</code>
   * to produce a GIF file for output file <code>types.dot</code>.
   */
  class UIMA_LINK_IMPORTSPEC DotTypeSystemWriter {
  private:
    uima::CAS const & iv_crCAS;

    void writeType(std::ostream &, Type const &, size_t uiIndent) const;
  public:
    DotTypeSystemWriter(CAS const &, bool bNoCASTypes , bool bOnlyCASTypes );
    void write(std::ostream &) const;
    bool iv_bNoCASTypes;
    bool iv_bOnlyCASTypes;
  };

}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

