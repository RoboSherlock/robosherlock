/** \file filename annotator_dump.cpp

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

// this is included ONCE for the main source file of each binary
#include "uima/pragmas.hpp"
#include "uima/annotator_dump.hpp"
#include "uima/xmlwriter.hpp"

#include <iostream>
#include <algorithm>
#ifdef _MSC_VER
#include <minmax.h> // for min
#endif
using namespace std;

#include "uima/assertmsg.h"
#include "uima/macros.h"
#include "uima/trace.hpp"

#include "uima/strconvert.hpp"
//#include "uima/fixed_vector.hpp"

//config parameter names
/* ----------------------------------------------------------------------- */
/*           Constants                                                     */
/* ----------------------------------------------------------------------- */

#define ANNOTATOR_DUMP_PARAM_OUTFILE          _TEXT("OutputFile")
#define ANNOTATOR_DUMP_PARAM_APPEND           _TEXT("AppendFile")
#define ANNOTATOR_DUMP_PARAM_DUMP_DOCBUFFER   _TEXT("DumpDocBuffer")
#define ANNOTATOR_DUMP_PARAM_SAVE_DOCBUFFER   _TEXT("SaveDocBuffer")
#define ANNOTATOR_DUMP_PARAM_STYLE            _TEXT("OutputStyle")
#define ANNOTATOR_DUMP_OUTPUT_TYPES           _TEXT("OutputTypes")

//error codes

const int ANNOTATOR_DUMP_ERROR_OFFSET           = 100;
const int ANNOTATOR_DUMP_ERROR_OPEN             = (0 + ANNOTATOR_DUMP_ERROR_OFFSET);
const int ANNOTATOR_DUMP_ERROR_AT               = (1 + ANNOTATOR_DUMP_ERROR_OFFSET);

const int ANNOTATOR_DUMP_WARN_OFFSET            = 200;
const int ANNOTATOR_DUMP_WARN_TET               = (0 + ANNOTATOR_DUMP_WARN_OFFSET);

const int ANNOTATOR_DUMP_MSG_OFFSET             = 300;
const int ANNOTATOR_DUMP_MSG_STYLE              = (0 + ANNOTATOR_DUMP_MSG_OFFSET);
const int ANNOTATOR_DUMP_MSG_TYPES              = (1 + ANNOTATOR_DUMP_MSG_OFFSET);

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

// Default Constructor

AnnotatorDump::AnnotatorDump(void) :
    iv_clOutputStream() {}

AnnotatorDump::~AnnotatorDump(void) {
  ;
}

TyErrorId
AnnotatorDump::initialize(
  AnnotatorContext & rclAnnotatorContext) {
  TyErrorId                  tyErrId;
  string                     strFileName;
  int                     uiOutputStyle;


  // Default Values

  // in append mode all data in a session/collection is dumped into one file
  // otherwise the same dump file is deleted and rewritten for each document
  // in the session/collection
  // default is false
  iv_bAppendFile = false;

  // we don't dump the Document Buffer
  iv_bDumpDocBuffer = false;
  iv_bSaveDocBuffer = false;

  // the representation will be in Xml-Format
  iv_enOutputStyle = Xml;

  // Reading the Values from the Config-Section

  //Filename for the Output-Stream
  icu::UnicodeString us;
  tyErrId = rclAnnotatorContext.extractValue(ANNOTATOR_DUMP_PARAM_OUTFILE, us);

  // Convert filename to default encoding
  UnicodeStringRef usr(us);
  usr.extract(strFileName);

  if (tyErrId != UIMA_ERR_NONE) {
    getAnnotatorContext().getLogger().logError(
      _TEXT("Required option '" ANNOTATOR_DUMP_PARAM_OUTFILE "' not found"),
      (long)ANNOTATOR_DUMP_ERROR_OPEN);
    return(UIMA_ERR_USER_ANNOTATOR_CONFIG_INVALID_PARAM);
  }

  iv_clOutputFilename = strFileName.c_str();
  (void) rclAnnotatorContext.extractValue(ANNOTATOR_DUMP_PARAM_DUMP_DOCBUFFER, iv_bDumpDocBuffer);
  (void) rclAnnotatorContext.extractValue(ANNOTATOR_DUMP_PARAM_SAVE_DOCBUFFER, iv_bSaveDocBuffer);
  (void) rclAnnotatorContext.extractValue(ANNOTATOR_DUMP_PARAM_APPEND, iv_bAppendFile);

  // in append mode all data in a session/collection is dumped into one file
  // otherwise the same dump file is deleted and rewritten for each document
  // in the session/collection
  if (iv_bAppendFile) {
    tyErrId = openOutputFile();
    if (tyErrId != UIMA_ERR_NONE) {
      return tyErrId;
    }
  }

  //Output Style
  uiOutputStyle = 0;
  tyErrId = rclAnnotatorContext.extractValue(ANNOTATOR_DUMP_PARAM_STYLE,  uiOutputStyle);
  if (tyErrId != UIMA_ERR_CONFIG_OPTION_NOT_FOUND) {
    switch (uiOutputStyle) {
    case 0:
      iv_enOutputStyle = Xml;
      break;
    case 1:
      iv_enOutputStyle = XCas;
      break;
    default:
      getAnnotatorContext().getLogger().logWarning(
        "Invalid Output Style. Use Default",
        (long) ANNOTATOR_DUMP_MSG_STYLE);
      break;
    }
  }

  /*
     int iDummy;
     if (   rclAnnotatorContext.extractValue(ANNOTATOR_DUMP_OUTPUT_TYPES, iDummy)){
       getAnnotatorContext().getLogger().logWarning((long) ANNOTATOR_DUMP_MSG_TYPES,
         _TEXT("Specification of output types currently not suported. All types will be dumped."));
     }
  */

  // Test getting a multi-valued parameter

  vector<string*> vecOutputTypes;
  tyErrId = rclAnnotatorContext.extractValue(ANNOTATOR_DUMP_OUTPUT_TYPES,  vecOutputTypes);
  if (tyErrId != UIMA_ERR_CONFIG_OPTION_NOT_FOUND) {
    size_t i, len = 0;
    for (i=0; i<vecOutputTypes.size(); ++i) {
      string * rString = vecOutputTypes[i];
      len += rString->length();
    }
    cout << "  parameter OutputTypes has "<<i<<" values with a total of "<<len<<" characters." << endl;
    // Release contents of vector allocated by extractValue in case library uses a different heap.
    rclAnnotatorContext.release(vecOutputTypes);
  }

  // Release string buffer allocated by library in case it uses a different heap.
  usr.release(strFileName);

  return(TyErrorId)UIMA_ERR_NONE;
}


TyErrorId AnnotatorDump::typeSystemInit(uima::TypeSystem const & typeSystem) {

  // Test that can get all types in a vector
  std::vector<Type> allTypes;
  typeSystem.getAllTypes(allTypes);
  cout << "  typeSystem has " << allTypes.size() << " types -" << endl << "      from '"
  << allTypes[0].getName() << "' to '" << allTypes[allTypes.size()-1].getName() << "'" << endl;
  typeSystem.release(allTypes);                // Release storage allocated from library's heap

  std::vector<Feature> allFeats;
  typeSystem.getAllFeatures(allFeats);
  cout << "  typeSystem has " << allFeats.size() << " features -" << endl << "      from '"
  << allFeats[0].getName() << "' to '" << allFeats[allFeats.size()-1].getName() << "'" << endl;
  typeSystem.release(allFeats);                // Release storage allocated from library's heap

  return UIMA_ERR_NONE;
}


TyErrorId
AnnotatorDump::openOutputFile( void ) {
  iv_clOutputStream.open(iv_clOutputFilename.getAsCString());     //overwrite

  if (iv_clOutputStream.good()) {
    return UIMA_ERR_NONE;
  }
  getAnnotatorContext().getLogger().logError(
    string("Failed to open Output File ")+ iv_clOutputFilename.getAsCString(),
    (long) ANNOTATOR_DUMP_ERROR_OPEN);
  return(UIMA_ERR_USER_ANNOTATOR_IO_WRITE_PROBLEM);
}

void
AnnotatorDump::closeOutputFile( void ) {
  iv_clOutputStream.close();
}

TyErrorId
AnnotatorDump::destroy(
) {
  if (iv_bAppendFile) {
    closeOutputFile();
  }

  return(TyErrorId)UIMA_ERR_NONE;
}

TyErrorId
AnnotatorDump::reconfigure(
) {
  return(TyErrorId)UIMA_ERR_NONE;
}

TyErrorId
AnnotatorDump::process(CAS & tcas,
                       const ResultSpecification &
                      ) {
  TyErrorId tyErrId;

  // in append mode all data in a session/collection is dumped into one file
  // otherwise the same dump file is deleted and rewritten for each document
  // in the session/collection
  if (!iv_bAppendFile) {
    tyErrId = openOutputFile();
    if (tyErrId != UIMA_ERR_NONE) {
      return tyErrId;
    }
  }

  assert(iv_clOutputStream.good());

  if (iv_bDumpDocBuffer) {
    //    Dumping the Document Buffer
    UnicodeStringRef doc = tcas.getDocumentText();

    outputDocBuffer(doc);
  }

  uima::CASWriterABase * writer = NULL;
  switch (iv_enOutputStyle) {
  case Xml:
    writer = new uima::XMLDumpWriter(tcas, iv_bDumpDocBuffer);
    break;
  case XCas:
    writer = new uima::XCASWriter(tcas, iv_bDumpDocBuffer);
    break;
  default:
    assert(false);
  }
  assert( EXISTS(writer) );
  unique_ptr<CASWriterABase> apWriter( writer );
  apWriter->write(iv_clOutputStream);

  // in append mode all data in a session/collection is dumped into one file
  // otherwise the same dump file is deleted and rewritten for each document
  // in the session/collection
  if (!iv_bAppendFile) {
    closeOutputFile();
  }

  return(TyErrorId)UIMA_ERR_NONE;
}

void AnnotatorDump::outputDocBuffer(UnicodeStringRef const & crclDoc) {
  assert(crclDoc.length() > 0);
  uima::util::Filename        clFilename(iv_clOutputFilename);
  ofstream                   clOutStream;
  TyDocIndex                 tyIndexFirst = 0;
  TyDocIndex                 tyIndexLast = crclDoc.length() - 1;

  clFilename.setNewExtension(_TEXT(".asc"));
  if (iv_bAppendFile) {
    clOutStream.open(clFilename.getAsCString(), ios::out | ios::app);   //append (create, resp.)
  } else {
    clOutStream.open(clFilename.getAsCString());    //overwrite
  }

  if (iv_clOutputStream.good()) {
    clOutStream << "Document Buffer Status:" << endl
    << "=======================" << endl;

    clOutStream << endl
    << "Document buffer length...........: " << crclDoc.length() << endl
    << "Document buffer size in memory...: " << (crclDoc.length() * sizeof(UChar)) << endl
    << "Document buffer index first......: " << tyIndexFirst << endl
    << "Document buffer index last.......: " << tyIndexLast << endl;

    clOutStream << endl
    << "Document Buffer Dump:" << endl
    << "=====================" << endl;
    assertWithMsg(sizeof(WORD16) == sizeof(UChar), "Port required");
    DUMPHEX(clOutStream, (WORD16 const *)crclDoc.getBuffer(), crclDoc.length());
    clOutStream << endl;
  }
  if (iv_bSaveDocBuffer) {
    ofstream                clOutStream;

    clFilename.setNewExtension(_TEXT(".ucs"));

    clOutStream.open(clFilename.getAsCString(), ios::binary);   //overwrite
    /** include byte-order-mark ??
       clOutStream << CosClConverterABase::getUCS2HostEndianId();
    **/
    clOutStream.write((const char *) crclDoc.getBuffer(), (crclDoc.length() * sizeof(UChar)));
  }
}


/* ----------------------------------------------------------------------- */
/*   Mapping for generic C API wrapper                                     */
/* ----------------------------------------------------------------------- */

typedef AnnotatorDump UserDefinedAnnotator;
// define for error/exception info in annotator_generic.inl
#define UIMA_ANNOTATOR_NAME "annotator_dump"

/* ----------------------------------------------------------------------- */
/*   Include generic C API wrapper                                         */
/* ----------------------------------------------------------------------- */

///#include "uima/annotator_generic.inl"
MAKE_AE(AnnotatorDump);
/* <EOF> */

