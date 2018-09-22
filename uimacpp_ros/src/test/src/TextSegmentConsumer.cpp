/** \file TextSegmentConsumer.cpp .

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

#include "uima/api.hpp"

using namespace std;
using namespace uima;

 
class TextSegmentConsumer : public Annotator {
private:
  
  int numProcessed; 
  int numSegments;

  


public:

  TextSegmentConsumer(void)
  {
    cout << "TextSegmentConsumer: Constructor" << endl;
  }

  ~TextSegmentConsumer(void)
  {
    cout << "TextSegmentConsumer: Destructor" << endl;
  }
  
  /** */
  TyErrorId initialize(AnnotatorContext & rclAnnotatorContext)
  {
    cout << "TextSegmentConsumer: initialize()" << endl;
    numProcessed=0;
	numSegments=0;
    if (rclAnnotatorContext.isParameterDefined("TotalNumberOfSegments") &&
		rclAnnotatorContext.extractValue("TotalNumberOfSegments", numSegments) == UIMA_ERR_NONE)  {
	    if (numSegments < 1) {
 			UIMA_EXC_THROW_NEW(Exception,
	                  UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT,
                      UIMA_MSG_ID_EXCON_CONFIG_VALUE_EXTRACT,
                      ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, "Invalid value for 'TotalNumberOfSegments'"),
                      ErrorInfo::unrecoverable);
        }    
     } else {
	   numSegments=3;	
	 }
	 cout << "TextSegmentConsumer::initialize() .. Total number of segments expected: " 
	             << numSegments << endl;    	
    return (TyErrorId)UIMA_ERR_NONE;
  }


  TyErrorId typeSystemInit(TypeSystem const & crTypeSystem) {
    cout << "TextSegmentConsumer: typeSystemInit()" << endl;
    return(TyErrorId)UIMA_ERR_NONE;
  }

  /** */
  TyErrorId destroy()
  {
    cout << "TextSegmentConsumer: destroy()" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }

  
  TyErrorId process(CAS & rCAS, const ResultSpecification& spec)
  {
   cout << "TextSegmentConsumer: process() begins" << endl;

   if (rCAS.getDocumentText().length() > 0) {
         cout << "TextSegmentConsumer::process() Got a text segment " << rCAS.getDocumentText() << endl;
	   numProcessed++;
   } else {
	   UIMA_EXC_THROW_NEW(Exception,
		   UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS,
		   UIMA_MSG_ID_EXCON_CONFIG_VALUE_EXTRACT,
		   ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, "getDocumentText() returned string of zero length."),
		   ErrorInfo::unrecoverable);
   }

   cout << "TextSegmentConsumer: process() ends" << endl;
   return (TyErrorId)UIMA_ERR_NONE;
  }

  TyErrorId batchProcessComplete()
  {
  cout << "TextSegmentConsumer: batchProcessComplete()" << endl;
  return (TyErrorId)UIMA_ERR_NONE;
  }

  TyErrorId collectionProcessComplete()
  {
  cout << "TextSegmentConsumer: collectionProcessComplete() num processed: " << numProcessed << endl;

  if (numProcessed != numSegments) {
	  UIMA_EXC_THROW_NEW(Exception,
		   UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS,
		   UIMA_MSG_ID_EXCON_CONFIG_VALUE_EXTRACT,
		   ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, "getDocumentText() returned string of zero length."),
		   ErrorInfo::unrecoverable);
  }
  
  return (TyErrorId)UIMA_ERR_NONE;
  }


};

// This macro exports an entry point that is used to create the annotator.

MAKE_AE(TextSegmentConsumer);
