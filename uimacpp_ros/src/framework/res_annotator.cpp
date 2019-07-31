/** @name res_annotator.cpp
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

   Description: This file contains class

-----------------------------------------------------------------------------


   9/9/1999  Initial creation

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/res_annotator.hpp>
#include <limits.h>
#include <uima/resmgr.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define UIMA_RESOURCE_ANNOTATOR_FILENAME_EXTENSION   _TEXT("$PLG")


/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {

  namespace internal {

    void ResourceAnnotatorFile::init(uima::ErrorInfo & rErrInfo)   {

      icu::UnicodeString const & crAnnotatorFileName = getKey();

      // first we must convert our unicode filename to single byte
      size_t nameLen = crAnnotatorFileName.length();
      char* buf = new char[1+nameLen];
      crAnnotatorFileName.extract(0, nameLen, buf);
      util::Filename filename(buf);
      delete[] buf;

      /* No longer optionally remove the path before loading, or add "itu" prefix.
         Now load only what we're given, i.e. a full filename with path,
         or a partial name, letting the OS search the appropriate paths. */

      assert(iv_pAnnotatorFile == NULL);
      iv_pAnnotatorFile = new util::DllProcLoaderFile(filename);
	  if ( ! (EXISTS(iv_pAnnotatorFile)) ) {
		rErrInfo.setErrorId(UIMA_ERR_RESMGR_COULD_NOT_LOAD_RESOURCE);
		ResourceManager::getInstance().getLogger().logError(rErrInfo);
	  } else if ( ! iv_pAnnotatorFile->isValid() ) {
		rErrInfo.setErrorId(iv_pAnnotatorFile->getErrorId());
		ErrorMessage msg(iv_pAnnotatorFile->getErrorMsgId(), 
					     iv_pAnnotatorFile->getErrorMsg());
		msg.addParam(filename.getAsCString());
		msg.addParam(iv_pAnnotatorFile->getErrorMsg());
		rErrInfo.setMessage(msg);
        ResourceManager::getInstance().getLogger().logError(rErrInfo);
	  } else {
        setNewKey(filename.getAsCString());
		std::string msg = "Loaded ";
		msg.append(filename.getAsCString());
		ResourceManager::getInstance().getLogger().logMessage(msg);
	  }    
    }

    void ResourceAnnotatorFile::deInit(void) {
      delete iv_pAnnotatorFile;
      iv_pAnnotatorFile = 0;
    }



    ////////////////////////////////////////////////////////////////

    ResourceAnnotatorFileFactory::ResourceAnnotatorFileFactory(void)
        : ResourceFactoryABase(UIMA_RESOURCE_ANNOTATOR_FILENAME_EXTENSION) {}


    ResourceABase * ResourceAnnotatorFileFactory::createResource(icu::UnicodeString const & crKey) const {
      ResourceAnnotatorFile *  pclRetVal;

      pclRetVal = new ResourceAnnotatorFile(crKey, getKind() );
      assert(EXISTS(pclRetVal));
      return(pclRetVal);
    }

  }

}



/* <EOF> */

