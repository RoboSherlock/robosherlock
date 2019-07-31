#ifndef UIMA_ANNOTATOR_DUMP_H$
#define UIMA_ANNOTATOR_DUMP_H$
/** \file annotator_dump.hpp .
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

-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include "uima/api.hpp"                               /* UIMA API */

#include <fstream>
#include <vector>
#include <deque>


#include "uima/filename.hpp"

using namespace uima;
using namespace std;
/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */


/** @name AnnotatorDump
   The class <TT>AnnotatorDump</TT> is used to .
   Example:
   \code
   \endcode
   @see
*/
class AnnotatorDump : public TextAnnotator {
public:
  /** @name Constructors */
  /*@{*/
  /** Default Constructor:
  */
  AnnotatorDump();
  /*@}*/

  ~AnnotatorDump(void);  //lint !e1908 !e1509: base class destructor for class 'AnnotatorABase' is not virtual : 'virtual' assumed for ~AnnotatorDump() (inherited from base class AnnotatorABase)

  /** @name Annotator Processing Functions */
  /*@{*/
  /** call the UIMA Annotator to initialize itself based on a UIMA engine
      and a UIMA Configuration section and return a UIMA error code */
  TyErrorId
  initialize(
    AnnotatorContext & rclAnnotatorContext
  );  //lint !e1909: 'virtual' assumed, see: AnnotatorABase::init(Engine &, ConfigAnnotator &) (line 79, file g:\projects\UIMAcurrent\code\engine\include\annotator_abase.hpp)

  TyErrorId typeSystemInit(uima::TypeSystem const &);

  /** call the UIMA Annotator to deinitialize itself based on a UIMA engine
      and return a UIMA error code */
  TyErrorId
  destroy();  //lint !e1909: 'virtual' assumed, see: AnnotatorABase::deInit(Engine &) (line 83, file g:\projects\UIMAcurrent\code\engine\include\annotator_abase.hpp)

  /** call the UIMA Annotator to reconfigure itself based on a UIMA Configuration
      section and return a UIMA error code */
  TyErrorId
  reconfigure(
  );  //lint !e1909: 'virtual' assumed, see: AnnotatorABase::config(ConfigAnnotator &) (line 87, file g:\projects\UIMAcurrent\code\engine\include\annotator_abase.hpp)

  /** call the UIMA Annotator to perform its doc related duty based on a UIMA engine
      and return a UIMA error code */
  TyErrorId
  process(
    CAS & tcas,
    ResultSpecification const &
  );  //lint !e1909: 'virtual' assumed, see: AnnotatorABase::processDocument(Engine &, const TargetSetAT &, const TargetSetTT &) (line 91, file g:\projects\UIMAcurrent\code\engine\include\annotator_abase.hpp)

  /*@}*/

  /** @name Properties */
  /*@{*/
  /*@}*/
  /** @name Miscellaneous */
  /*@{*/
  /*@}*/
protected:
private:

  /* --- types ---------------------------------------------------------------*/

  enum EnOutputStyle {
    Xml, XCas
  };

  /* --- variables -------------------------------------------------------- */
  util::Filename iv_clOutputFilename;
  bool     iv_bDumpDocBuffer; // When set to 'True', the Annotator dumps the Doc Buffer
  bool     iv_bSaveDocBuffer; // When set to 'True', the Annotator dumps the Doc Buffer in binary format, too

  ofstream iv_clOutputStream;
  bool     iv_bAppendFile;

  EnOutputStyle iv_enOutputStyle;

  //vector<uima::Type> iv_vecOutputTypes;

  // The annotator may be invoked in several sections within one config-file.
  // If all output gets dumped into one file, the names of the sections serve
  // as headers. We can't access the ConfigAnnotator-Object in 'processDocument',
  // hence, we need a member var.
  string iv_cpszSectionName;
  /* --- functions -------------------------------------------------------- */

  TyErrorId
  openOutputFile( void );

  void
  closeOutputFile( void );

  void outputDocBuffer(UnicodeStringRef const & crclDoc);


  AnnotatorDump & operator=(
    const AnnotatorDump &
  );

  AnnotatorDump(
    const AnnotatorDump &
  );

}
; /* AnnotatorDump */

/* ----------------------------------------------------------------------- */
#endif

/* <EOF> */

