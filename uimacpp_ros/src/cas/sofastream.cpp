
/** @name sofastream.cpp
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

   Description: This file contains class {\tt SofaDataSteram}.


-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */
//#define DEBUG_VERBOSE

#include <uima/pragmas.hpp> //must be included first to disable warnings
#include <uima/err_ids.h>
#include <uima/msg.h>
#include <uima/assertmsg.h>
#include <uima/trace.hpp>
#include <uima/resmgr.hpp>
#include <uima/sofastream.hpp>
#include <uima/lowlevel_fsheap.hpp>
#include <uima/arrayfs.hpp>
#include <uima/casexception.hpp>
#include <uima/uima_endian.h>
#include <uima/sofastreamhandler.hpp>
namespace uima {



// ----------------------------------------------------------- */
//       RemoteSofaDataStream implementation                   */
// ----------------------------------------------------------- */
  //Constructor
  RemoteSofaDataStream::RemoteSofaDataStream(SofaFS& aSofaFS) : iv_psofafs(NULL),
      iv_pHandler(NULL), iv_pHandlerDll(NULL) {
    iv_psofafs =  (SofaFS*) new FeatureStructure(aSofaFS.iv_tyFS,aSofaFS.getCAS());
    assert(iv_psofafs->isValid());
    assert(iv_psofafs->getType().getName().compare( UnicodeStringRef(CAS::TYPE_NAME_SOFA)) == 0 );
  }

  //Destructor
  RemoteSofaDataStream::~RemoteSofaDataStream() {
    if (iv_pHandler != NULL) {
      iv_pHandler->closeStream();
      delete iv_pHandler;
      iv_pHandler = NULL;
    }
    if (iv_pHandlerDll != NULL) {
      delete iv_pHandlerDll;
      iv_pHandlerDll=NULL;
    }
    if (iv_psofafs != NULL) {
      delete iv_psofafs;
    }
  }



  //open - minbufsize optional
  int RemoteSofaDataStream::open(size_t minbufsize) {
    std::string sofauri;
    std::string urischeme;
    util::Filename const * pDllFile=NULL;

    //get the sofa uri as UTF-8 string
    iv_psofafs->getSofaURI().extract(0,
                                     iv_psofafs->getSofaURI().length(),
                                     sofauri,
                                     CCSID::getDefaultSBCSInputCCSID() );
    //get the uri scheme
    int32_t pos = sofauri.find(":");
    if (pos != -1) {
      //get the uri scheme
      urischeme = sofauri.substr(0,pos);
    } else {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_INVALID_SOFAURI);
      errMsg.addParam(sofauri);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }

    //look up sofa stream handler for uri scheme
    if (urischeme.size() >  0)  {
      pDllFile = uima::ResourceManager::getInstance().getStreamHandlerForURIScheme(urischeme);
      if (pDllFile == NULL) {
        ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SCHEMEHANDLER_NOT_REGISTERED);
        errMsg.addParam(urischeme);
        errMsg.addParam(sofauri);
        UIMA_EXC_THROW_NEW(SofaDataStreamException,
                           UIMA_ERR_SOFADATASTREAM,
                           errMsg,
                           UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                           ErrorInfo::unrecoverable);
      }
    } else {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_INVALID_SOFAURI);
      errMsg.addParam(sofauri);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }

    //load the handler
    iv_pHandlerDll  = new util::DllProcLoaderFile(*pDllFile);
    //assert(EXISTS(iv_pHandlerDll));
    if (iv_pHandlerDll == NULL) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SCHEMEHANDLER_LOAD);
      errMsg.addParam(pDllFile->getName());
      errMsg.addParam(sofauri);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }

    uima::TyMakeStreamHandler procMaker = (TyMakeStreamHandler) iv_pHandlerDll->getProcedure(UIMA_SOFASTREAM_MAKE_HANDLER);
    assert(EXISTS(procMaker));
    if (NOTEXISTS(procMaker)) {
      ResourceManager::getInstance().getLogger().logError("RemoteSofaDataStream::open() Could not get procedure makeHandler() ");
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SCHEMEHANDLER_GETPROC);
      errMsg.addParam(pDllFile->getName());
      errMsg.addParam(sofauri);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }
    /* first we need to create the handler */
    iv_pHandler = (procMaker) ();

    if (NOTEXISTS(iv_pHandler)) {
      ResourceManager::getInstance().getLogger().logError("RemoteSofaDataStream::open() Could not instantiate stream handler. ");
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SCHEMEHANDLER_GETPROC);
      errMsg.addParam(pDllFile->getName());
      errMsg.addParam(sofauri);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }

    /* open the stream */
    if (minbufsize > 0) {
      iv_pHandler->openStream(sofauri.data(), minbufsize);
    } else {
      iv_pHandler->openStream(sofauri.data());
    }

    return 0;
  }

  // Have a lot of near-identical error code ... Could we make it simpler with a single error routine?  BLL

  //getTotalStreamSize
  INT64 RemoteSofaDataStream::getTotalStreamSizeInBytes() {
    if (iv_pHandler == NULL) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SOFADATASTREAM_NOTOPEN);
      errMsg.addParam("getTotalStreamSize");
      errMsg.addParam(iv_psofafs->getSofaURI());
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }
    return (iv_pHandler->getTotalStreamSize());
  }

  //howManyBytesAvailable
  INT64 RemoteSofaDataStream::howManyBytesAvailable() {
    if (iv_pHandler == NULL) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SOFADATASTREAM_NOTOPEN);
      errMsg.addParam("HowManyBytesAvailable");
      errMsg.addParam(iv_psofafs->getSofaURI());
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }
    return ( iv_pHandler->howManyAvailable());
  }

  //read
  int RemoteSofaDataStream::read( void * pbuffer, int elementSize, size_t numElements) {

    if (iv_pHandler == NULL) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SOFADATASTREAM_NOTOPEN);
      errMsg.addParam("read");
      errMsg.addParam(iv_psofafs->getSofaURI());
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }
    INT64 numbytesread = elementSize*numElements;

    //read bytes
    numbytesread = iv_pHandler->getNext(numbytesread,pbuffer);

    //swap bytes if needed (data read is assumed to be BE)
    if (numbytesread > 0) {
      if ( elementSize==2 || elementSize==4 || elementSize == 8) {
        // Convert BE data to host byte order
        UIMA_HIBYTEFIRST(pbuffer, elementSize, numbytesread/elementSize);
        return numbytesread/elementSize;        // return number of elements read
      }
    }
    return numbytesread;                            // assume elementSize = 1
  }

  //seek
  int RemoteSofaDataStream::seek(INT64 offset, int origin) {
    if (iv_pHandler == NULL) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SOFADATASTREAM_NOTOPEN);
      errMsg.addParam("seek");
      errMsg.addParam(iv_psofafs->getSofaURI());
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }
    return (iv_pHandler->seek(offset,origin));
  }

  //close
  void RemoteSofaDataStream::close() {
    if (iv_pHandler != NULL) {
      iv_pHandler->closeStream();
    }
  }

  //getDataPointer
  const TySofaDataPointer RemoteSofaDataStream::getDataPointer() {

    if (iv_pHandler == NULL) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SOFADATASTREAM_NOTOPEN);
      errMsg.addParam("getDataPointer");
      errMsg.addParam(iv_psofafs->getSofaURI());
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }
    return (iv_pHandler->getDataPointer());

  }

  // ----------------------------------------------------------- */
  //       LocalSofaDataStream implementation                    */
  // ----------------------------------------------------------- */

  //Constructor
  LocalSofaDataStream::LocalSofaDataStream(SofaFS& aSofaFS) : iv_psofafs(NULL),
      iv_psofadata(NULL), iv_pstringsofadata(NULL), iv_size(0),
      iv_isstring(false), iv_isintarray(false),
      iv_isfloatarray(false), iv_isbooleanarray(false),
      iv_isbytearray(false),iv_isshortarray(false),
      iv_islongarray(false),iv_isdoublearray(false),
      iv_curpos(0) {
    iv_psofafs =  (SofaFS*) new FeatureStructure(aSofaFS.iv_tyFS,aSofaFS.getCAS());
    assert(iv_psofafs->isValid());
    assert(iv_psofafs->getType().getName().compare( UnicodeStringRef(CAS::TYPE_NAME_SOFA)) == 0 );
  }

  //Destructor
  LocalSofaDataStream::~LocalSofaDataStream() {
    close();
    if (iv_psofafs != NULL)
      delete iv_psofafs;
  }

  //open - minbufsize ignored
  int LocalSofaDataStream::open(size_t minbufsize) {

    Type type = iv_psofafs->getType();
    Feature stringfeat = type.getFeatureByBaseName(CAS::FEATURE_BASE_NAME_SOFASTRING);
    Feature arrayfeat = type.getFeatureByBaseName(CAS::FEATURE_BASE_NAME_SOFAARRAY);

    iv_curpos=0;
    if  ( !iv_psofafs->isUntouchedFSValue(stringfeat) ) {     //local sofa - data in string
      //convert string to UTF-8
      UErrorCode errorcode = U_ZERO_ERROR ;
      iv_size=0;
      int32_t len;
      iv_psofadata = u_strToUTF8(NULL, 0,
                                 &len,
                                 iv_psofafs->getStringValue(stringfeat).getBuffer(),
                                 iv_psofafs->getStringValue(stringfeat).length(),
                                 &errorcode);
      if (!U_SUCCESS(errorcode)  && errorcode != U_BUFFER_OVERFLOW_ERROR ) {
        ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_LOCAL_SOFADATA_UNSUPPORTED_TYPE);
        errMsg.addParam(stringfeat.getName());
        UIMA_EXC_THROW_NEW(SofaDataStreamException,
                           UIMA_ERR_SOFADATASTREAM,
                           errMsg,
                           UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                           ErrorInfo::unrecoverable);
      }
      iv_pstringsofadata = new char[len];
      iv_size = len;
      errorcode = U_ZERO_ERROR ;
      iv_psofadata = u_strToUTF8(iv_pstringsofadata, iv_size,
                                 &len,
                                 iv_psofafs->getStringValue(stringfeat).getBuffer(),
                                 iv_psofafs->getStringValue(stringfeat).length(),
                                 &errorcode);
      if (!U_SUCCESS(errorcode)  ) {
        ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_LOCAL_SOFADATA_UNSUPPORTED_TYPE);
        errMsg.addParam(stringfeat.getName());
        UIMA_EXC_THROW_NEW(SofaDataStreamException,
                           UIMA_ERR_SOFADATASTREAM,
                           errMsg,
                           UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                           ErrorInfo::unrecoverable);
      }
      iv_isstring=true;
    } else if ( !iv_psofafs->isUntouchedFSValue(arrayfeat) ) {  //local sofa - data in fs array

      FeatureStructure arrayFS = iv_psofafs->getFSValue(arrayfeat);
      Type rangeType = arrayFS.getType();
      UnicodeStringRef rangeTypeName = rangeType.getName();

      if (rangeTypeName.compare(CAS::TYPE_NAME_INTEGER_ARRAY) ==0) {
        iv_psofadata =  (char*) iv_psofafs->getCAS().getHeap()->getCArrayFromFS(arrayFS.iv_tyFS);
        iv_size = iv_psofafs->getIntArrayFSValue(arrayfeat).size()*sizeof(int);
        iv_isintarray = true;
      } else if (rangeTypeName.compare( CAS::TYPE_NAME_FLOAT_ARRAY) ==0) {
        iv_psofadata =  (char*) iv_psofafs->getCAS().getHeap()->getCArrayFromFS(arrayFS.iv_tyFS);
        iv_size= iv_psofafs->getFloatArrayFSValue(arrayfeat).size()*sizeof(float);
        iv_isfloatarray = true;
      } else if (rangeTypeName.compare( CAS::TYPE_NAME_BOOLEAN_ARRAY) ==0) {
        iv_psofadata =  (char*) iv_psofafs->getCAS().getHeap()->get8BitArray(arrayFS.iv_tyFS);
        iv_size= iv_psofafs->getBooleanArrayFSValue(arrayfeat).size()*sizeof(char);
        iv_isbooleanarray = true;
      } else if (rangeTypeName.compare( CAS::TYPE_NAME_BYTE_ARRAY) ==0) {
        iv_psofadata =  (char*) iv_psofafs->getCAS().getHeap()->get8BitArray(arrayFS.iv_tyFS);
        iv_size= iv_psofafs->getByteArrayFSValue(arrayfeat).size()*sizeof(char);
        iv_isbytearray = true;
      } else if (rangeTypeName.compare( CAS::TYPE_NAME_SHORT_ARRAY) ==0) {
        iv_psofadata =  (char*) iv_psofafs->getCAS().getHeap()->get16BitArray(arrayFS.iv_tyFS);
        iv_size= iv_psofafs->getShortArrayFSValue(arrayfeat).size()*sizeof(short);
        iv_isshortarray = true;
      } else if (rangeTypeName.compare( CAS::TYPE_NAME_LONG_ARRAY) ==0) {
        iv_psofadata =  (char*) iv_psofafs->getCAS().getHeap()->get64BitArray(arrayFS.iv_tyFS);
        iv_size= iv_psofafs->getLongArrayFSValue(arrayfeat).size()*sizeof(INT64);
        iv_islongarray = true;
      } else if (rangeTypeName.compare( CAS::TYPE_NAME_DOUBLE_ARRAY) ==0) {
        iv_psofadata =  (char*) iv_psofafs->getCAS().getHeap()->get64BitArray(arrayFS.iv_tyFS);
        iv_size= iv_psofafs->getDoubleArrayFSValue(arrayfeat).size()*sizeof(INT64);
        iv_isdoublearray = true;
      } else {
        ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_LOCAL_SOFADATA_UNSUPPORTED_TYPE);
        errMsg.addParam(rangeTypeName);
        UIMA_EXC_THROW_NEW(SofaDataStreamException,
                           UIMA_ERR_SOFADATASTREAM,
                           errMsg,
                           UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                           ErrorInfo::unrecoverable);
      }
    } else {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_LOCAL_SOFADATA_NOTSET);
      errMsg.addParam(type.getName());
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }

    return 0;
  }

  //getTotalStreamSize
  INT64 LocalSofaDataStream::getTotalStreamSizeInBytes() {
    if (iv_size==0) {
      return SOFASTREAMHANDLER_EOF;
    } else return iv_size;
  }

  //howManyBytesAvailable
  INT64 LocalSofaDataStream::howManyBytesAvailable() {
    return (getTotalStreamSizeInBytes()-iv_curpos);
  }


  //read
  int LocalSofaDataStream::read( void * pbuffer, int elementSize, size_t numElements) {
    //check valid sofa data
    if (iv_psofadata == NULL) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_LOCAL_SOFADATA_NOTSET);
      errMsg.addParam(iv_psofafs->getType().getName());
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }

    //check for eof
    if (iv_curpos == iv_size) {
      return SOFASTREAMHANDLER_EOF;
    }

    //validate element size vis-a-vis datatype of sofa data
    if (iv_isstring && elementSize != 1) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_LOCAL_SOFADATA_ELEMENTSIZE);
      errMsg.addParam(elementSize);
      errMsg.addParam(CAS::TYPE_NAME_STRING);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    } else if (iv_isintarray && elementSize != sizeof(int)) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_LOCAL_SOFADATA_ELEMENTSIZE);
      errMsg.addParam(elementSize);
      errMsg.addParam(CAS::TYPE_NAME_INTEGER_ARRAY);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    } else if (iv_isfloatarray && elementSize != sizeof(float) ) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_LOCAL_SOFADATA_ELEMENTSIZE);
      errMsg.addParam(elementSize);
      errMsg.addParam(CAS::TYPE_NAME_FLOAT_ARRAY);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }

    INT64 numbytesread=elementSize*numElements;
    if (numbytesread > howManyBytesAvailable() ) {
      numbytesread= howManyBytesAvailable();
    }

    //read bytes
    memcpy(pbuffer,iv_psofadata+iv_curpos,numbytesread);

    //move current read position
    iv_curpos += numbytesread;

    return numbytesread;
  }

  //seek
  int LocalSofaDataStream::seek(INT64 offset, int origin) {
    if (iv_psofadata == NULL) {
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_LOCAL_SOFADATA_NOTSET);
      errMsg.addParam(iv_psofafs->getType().getName());
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }

    switch (origin) {
    case SEEK_SET:
      iv_curpos = offset;
      break ;
    case SEEK_CUR:
      iv_curpos += offset;
      break ;
    case SEEK_END:
      iv_curpos = getTotalStreamSizeInBytes() - offset;
      break ;
    default:
      ErrorMessage errMsg = ErrorMessage(UIMA_MSG_ID_EXC_SOFADATASTREAM_INVALID_SEEK_ORIGIN);
      errMsg.addParam(origin);
      UIMA_EXC_THROW_NEW(SofaDataStreamException,
                         UIMA_ERR_SOFADATASTREAM,
                         errMsg,
                         UIMA_MSG_ID_EXCON_SOFADATASTREAM,
                         ErrorInfo::unrecoverable);
    }
    return 0;
  }

  //close
  void LocalSofaDataStream::close() {
    if (iv_pstringsofadata != NULL)
      delete[] iv_pstringsofadata;
    iv_pstringsofadata = NULL;
    iv_curpos=0;
    iv_psofadata = NULL;
    iv_size=0;
    iv_isstring=false;
    iv_isintarray=false;
    iv_isfloatarray=false;
  }

  //getDataPointer
  const TySofaDataPointer LocalSofaDataStream::getDataPointer() {
    TySofaDataPointer dataptr=NULL;
    if (iv_psofadata != NULL) {
      dataptr = (TySofaDataPointer) iv_psofadata;
    }
    return dataptr;
  }


} //namespace uima
