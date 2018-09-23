/** @name log.cpp
------------------------------------------------------------------F-----------

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

   Description: This file contains class LogFacility

-----------------------------------------------------------------------------


   6/24/1999   Initial creation
   7/19/1999   logError(const ErrorInfo & crclErrorInfo) added

-------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <time.h>
#include <uima/log.hpp>

#include <uima/trace.hpp>
#include <uima/comp_ids.h>
#include <uima/msg.h>
#include <uima/resmgr.hpp>
#include <uima/exceptions.hpp>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

#define UIMA_LOG_APPLICATION_KEY_UNKNOWN      _TEXT("???")
const size_t                  UIMA_LOG_STATIC_CONVERSION_BUFSIZE = 1024;
static apr_pool_t         * logPool=0;
static apr_thread_mutex_t * logMutex=0;
using namespace std;
/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Private Implementation                                            */
/* ----------------------------------------------------------------------- */
namespace uima {
  FileLogger::FileLogger(std::string filename) : iv_logfile(0) {
     iv_logfile = fopen(filename.c_str(),"a");
     if (iv_logfile == NULL) {   //Need to handle this better
        //cerr << "Could not open the log file " << cpszLogFile << endl;
        string str = "Could not open the log file ";
        str += filename;
        UIMA_EXC_THROW_NEW(Uima_runtime_error,
                           UIMA_MSG_ID_LITERAL_STRING,
                           UIMA_MSG_ID_LITERAL_STRING,
                           ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, str.c_str()),
                           ErrorInfo::unrecoverable);
      } 
  }

  void FileLogger::log(LogStream::EnEntryType enType, 
                      string cpszClass,
                      string cpszMethod,
                      string cpszMsg, 
                      long lUserCode) 
  /* ----------------------------------------------------------------------- */
  {
     //write to local log file if one is available
     string message = cpszClass + " ";
     if (cpszMethod.length() >0) {
       message += cpszMethod + " ";
     }
     message += cpszMsg;
     message = format(enType, message, lUserCode);
     fwrite(message.c_str(), 1, message.size(), iv_logfile);
     fflush(iv_logfile);
  }

   std::string  FileLogger::format(LogStream::EnEntryType enType, 
                                  const string & cpszMsg, 
                                  long lUserCode)  {
    
    time_t rawtime;
    time ( &rawtime );
    string currts = ctime(&rawtime);

    stringstream str;
    str << currts.substr(0,currts.length()-1);

    //map enType to string
    switch (enType) {
    case LogStream::EnWarning :
      str << " WARNING: ";
      str << " RC=" << lUserCode << " ";
      break;
    case LogStream::EnError :
      str << " SEVERE: ";
      str << " RC=" << lUserCode << " ";
      break;
    default:
      str << " INFO: ";
      if (lUserCode !=0) {
        str << " RC=" << lUserCode << " ";
      }
    }
    
    str << cpszMsg << endl;
   
    return str.str();
  }

  TyMessageId LogFacility::getTypeAsMessageId(LogStream::EnEntryType enType) const
  /* ----------------------------------------------------------------------- */
  {
    switch (enType) {
    case LogStream::EnMessage :
      return(UIMA_MSG_ID_LOG_MESSAGE);
    case LogStream::EnWarning :
      return(UIMA_MSG_ID_LOG_WARNING);
    case LogStream::EnError   :
      return(UIMA_MSG_ID_LOG_ERROR);
    default:
      assertWithMsg(false, _TEXT("Unknown EnLogEntryType"));  //lint !e506: Constant value Boolean
    }
    return(0);                                         /* shutup compiler */
  }

  void LogFacility::doLog(LogStream::EnEntryType enType, const TCHAR * cpszMsg, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    apr_thread_mutex_lock(logMutex);
    string method="";
    if (isLoggable(enType)) {
      for (int i=0; i < vecLoggers.size(); i++) {
        vecLoggers.at(i)->log(enType,this->iv_strOrigin,
                                     method,cpszMsg,lUserCode);
      }
    }
    apr_thread_mutex_unlock(logMutex);
  }

  bool LogFacility::isLoggable(LogStream::EnEntryType enType) const {
    ///LogStream::EnEntryType minLogLevel = ResourceManager::getInstance().getLoggingLevel();
    if (enType <  iv_logLevel) {
      return false;
    }
    return true;
  }

  void LogFacility::setLogLevel(LogStream::EnEntryType enType)  {
    ///LogStream::EnEntryType minLogLevel = ResourceManager::getInstance().getLoggingLevel();
    iv_logLevel = enType;
  }

  void LogFacility::log(LogStream::EnEntryType enType, const TCHAR * cpszMsg, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    doLog(enType, cpszMsg, lUserCode);

    if (enType == LogStream::EnError) {
      LogFacility * pclThis = CONST_CAST(LogFacility *, this);
      assert(pclThis == this);
      //remember last error as string/number
      pclThis->iv_lLastUserCode = lUserCode;
      pclThis->iv_strLastError  = string(cpszMsg);
      //reset error info object
      pclThis->iv_errInfo.reset();
    }
  }


  void  LogFacility::log(LogStream::EnEntryType enType, ErrorInfo const & errInfo) const
  /* ----------------------------------------------------------------------- */
  {
    doLog(enType, errInfo.asString().c_str());

    LogFacility * pclThis = CONST_CAST(LogFacility *, this);
    assert(pclThis == this);
    //remember last error as error info object
    pclThis->iv_errInfo = errInfo;
    //reset string/number
    // MSV6 compile error: pclThis->iv_strLastError.clear();
    pclThis->iv_strLastError = std::string();
    pclThis->iv_lLastUserCode = 0;
  }


  void LogFacility::log(LogStream::EnEntryType enType, const icu::UnicodeString & ustrMsg, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    string strMsg;
    UnicodeStringRef ref(ustrMsg);
    ref.extract(strMsg, CCSID::getDefaultName());
    log(enType, strMsg.c_str(), lUserCode);
  }

  /* ----------------------------------------------------------------------- */
  /*       Implementation                                                    */
  /* ----------------------------------------------------------------------- */

  LogStream & LogStream::flush() {
    this->stringstream::flush();
    iv_pLogFacility->flushLogStream();
    str("");
    return (*this);
  }

  LogStream & LogFacility::getLogStream(LogStream::EnEntryType enLogEntryType) {
    iv_logStream.iv_enLogEntryType = enLogEntryType;
    flushLogStream();
    return iv_logStream;
  }

  void LogFacility::flushLogStream() {
    string strMsg = iv_logStream.str();
    if (strMsg.size() == 0) {
      return;
    }
    log(iv_logStream.iv_enLogEntryType, strMsg.c_str());
  }


  LogFacility::LogFacility(icu::UnicodeString const & crEngineName):
      iv_lLastUserCode(0),
      iv_logStream(*this, LogStream::EnMessage),
      vecLoggers(ResourceManager::getInstance().getLoggers()),
      iv_logLevel(ResourceManager::getInstance().getLoggingLevel()) {
   if (logPool == NULL) {
      apr_status_t rv = apr_pool_create( &logPool,NULL );
      if ( rv == APR_SUCCESS ) {
        UnicodeStringRef ref(crEngineName);
        ref.extract(iv_strOrigin, CCSID::getDefaultName()  );
        apr_thread_mutex_create(&logMutex,APR_THREAD_MUTEX_UNNESTED, logPool);
      } else {
        UIMA_EXC_THROW_NEW(ExcOutOfMemory,
          UIMA_ERR_ENGINE_OUT_OF_MEMORY,
          UIMA_MSG_ID_EXC_OUT_OF_MEMORY,
          ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_POOL_FOR_CLASS,"uima::LogFacility"),
          ErrorInfo::unrecoverable);
      }
   }
  }

  LogFacility::LogFacility(icu::UnicodeString const & crEngineName,
                           LogStream::EnEntryType crLoggingLevel):
      iv_lLastUserCode(0),
      iv_logStream(*this, crLoggingLevel),
      vecLoggers(ResourceManager::getInstance().getLoggers()),
      iv_logLevel(crLoggingLevel) {

    if (logPool == NULL) {
      apr_status_t rv = apr_pool_create( &logPool,NULL );
      if ( rv == APR_SUCCESS ) {
        UnicodeStringRef ref(crEngineName);
        ref.extract(iv_strOrigin, CCSID::getDefaultName()  );
        apr_thread_mutex_create(&logMutex,APR_THREAD_MUTEX_UNNESTED, logPool);
      } else {
        UIMA_EXC_THROW_NEW(ExcOutOfMemory,
          UIMA_ERR_ENGINE_OUT_OF_MEMORY,
          UIMA_MSG_ID_EXC_OUT_OF_MEMORY,
          ErrorMessage(UIMA_MSG_ID_EXCON_CREATING_POOL_FOR_CLASS,"uima::LogFacility"),
          ErrorInfo::unrecoverable);
      }
    }
  }

  LogFacility::~LogFacility() {
    flushLogStream();
  }

  const TCHAR * LogFacility::getLastErrorAsCStr(void) const {
    /* ----------------------------------------------------------------------- */
    LogFacility * pclThis = CONST_CAST(LogFacility *, this);
    ErrorInfo errInfo = getLastError();
    if (errInfo.getErrorId() == UIMA_ERR_NONE) {
      return (NULL);
    }
    pclThis->iv_strLastError = getLastError().asString();
    return(iv_strLastError.c_str());
  }

  ErrorInfo const & LogFacility::getLastError() const {
    /* ----------------------------------------------------------------------- */
    // check if we have a string or info based information on last error
    if (iv_strLastError.length() > 0) {
      // if we have error string the error object must be clear
      assert(iv_errInfo.getErrorId() == UIMA_ERR_NONE);
      // now set up the object with the string information
      LogFacility * pclThis = CONST_CAST(LogFacility *, this);
      assert(pclThis == this);
      if (iv_lLastUserCode == 0) {
        pclThis->iv_errInfo.setErrorId(UIMA_ERR_USER_ANNOTATOR_ERROR_UNSPECIFIED);
      } else {
        pclThis->iv_errInfo.setErrorId(iv_lLastUserCode);
      }
      ErrorMessage msg(UIMA_MSG_ID_LOG_TO_ERROR_INFO, iv_strOrigin);
      msg.addParam(iv_strLastError);
      pclThis->iv_errInfo.setMessage(msg);
      pclThis->iv_errInfo.setSeverity(ErrorInfo::unrecoverable);
      // now that the information is transferred into the object clear string
      pclThis->iv_lLastUserCode = 0;
      // MSV6 compile error:
      //pclThis->iv_strLastError.clear();
      pclThis->iv_strLastError = std::string();
    }
    assert(iv_strLastError.length() == 0);
    assert(iv_lLastUserCode == 0);
    return iv_errInfo;
  }


  void LogFacility::logWarning(const ErrorInfo & errInfo) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnWarning, errInfo);
  }


  void LogFacility::logError(const ErrorInfo & errInfo) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnError, errInfo);
  }

} //namespace uima
/* <EOF> */




