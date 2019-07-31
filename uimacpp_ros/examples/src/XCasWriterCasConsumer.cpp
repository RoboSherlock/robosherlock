/*
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
 */

#include "uima/api.hpp"
#include "uima/dirwalk.hpp"
#include "uima/xmlwriter.hpp"
using namespace std;
using namespace uima;


class XCasWriterCasConsumer : public Annotator {
private:
  UnicodeString usOutDir;
  std::string   strOutDir;
  bool writeFile;
  int docnum;

  /* We have a separate function getConfigValues()
     for initialize() and reconfigure()
  */
  TyErrorId getConfigValues() {
    return (TyErrorId)UIMA_ERR_NONE;
  }


public:

  XCasWriterCasConsumer(void) {
    cout << "XCasWriterCasConsumer: Constructor" << endl;
  }

  ~XCasWriterCasConsumer(void) {
    cout << "XCasWriterCasConsumer: Destructor" << endl;
  }

  /** */
  TyErrorId initialize(AnnotatorContext & rclAnnotatorContext) {
    cout << "XCasWriterCasConsumer: initialize()" << endl;

    if (rclAnnotatorContext.isParameterDefined("OutputDirectory") &&
        rclAnnotatorContext.extractValue("OutputDirectory", usOutDir) == UIMA_ERR_NONE)  {

      /* log the configuration parameter setting */
      rclAnnotatorContext.getLogger().logMessage("OutputDirectory = '" + usOutDir + "'");

      /* create the directory if it does not exist */
      strOutDir = UnicodeStringRef(usOutDir).asUTF8();
      util::Location location(strOutDir.c_str());

      if (location.isExistent() ) {
        writeFile=true;
      } else {
        location.makeDirectory();
      }
      writeFile = true;

      cout << "XCasWriterCasConsumer::initialize() .. creates xcas files in directory "
      << usOutDir << endl;
    } else {
      rclAnnotatorContext.getLogger().logMessage("OutputDirectory not specified. XCAS will be written to stdout.");
      writeFile=false;
      cout << "XCasWriterCasConsumer::initialize() .. writes xcas to standard out. "
      << endl;
    }
    docnum=0;


    return (TyErrorId)UIMA_ERR_NONE;
  }


  TyErrorId typeSystemInit(TypeSystem const & crTypeSystem) {
    cout << "XCasWriterCasConsumer: typeSystemInit()" << endl;
    return(TyErrorId)UIMA_ERR_NONE;
  }

  /** */
  TyErrorId destroy() {
    cout << "XCasWriterCasConsumer: destroy()" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }


  TyErrorId process(CAS & rCAS, const ResultSpecification& spec) {
    cout << "XCasWriterCasConsumer: process() begins" << endl;

    XCASWriter writer(rCAS, true);

    if (writeFile) {
      /* construct a filename */
      stringstream outfn;
      outfn << strOutDir << "/doc" << docnum++ << ".xcas";
      outfn << flush;
      cout << "XCasWriterCasConsumer::process() xcas file " << outfn.str() << endl;

      //open a file stream for output xcas
      ofstream file;
      file.open (outfn.str().c_str(), ios::out | ios::binary);
      if ( !file ) {
        cerr << "XCasWriterCasConsumer: Error opening output xcas: " << outfn.str() << endl;
        return(TyErrorId) UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
      }

      writer.write(file);
      file.close();
    } else {
      cout << endl;
      writer.write(cout);
      cout << endl;
    }

    cout << "XCasWriterCasConsumer: process() ends" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }


  TyErrorId batchProcessComplete() {
    cout << "XCasWriterCasConsumer: batchProcessComplete()" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }

  TyErrorId collectionProcessComplete() {
    cout << "XCasWriterCasConsumer: collectionProcessComplete()" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }


};

// This macro exports an entry point that is used to create the annotator.

MAKE_AE(XCasWriterCasConsumer);
