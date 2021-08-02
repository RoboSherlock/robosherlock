/*
-------------------------------------------------------------------------------

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

-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
*/

#if defined(_MSC_VER)
#include <stdio.h>
#include <eh.h>
#include <windows.h>
#include <winbase.h>
#endif

#include <uima/pragmas.hpp> //must be included first to disable warnings
#include <uima/msg.h>
#include <string>
#include <sstream>

#include <uima/macros.h>
#include <uima/trace.hpp>

#include <uima/strconvert.hpp>
#include <uima/unistrref.hpp>
#include <uima/comp_ids.h>
#include <uima/exceptions.hpp>
#include <uima/msgstrtab.h>

using namespace std;

namespace uima {

///Constructor with just the message id
  ErrorMessage::ErrorMessage(
    TyMessageId                          utMsgId
  ) :
      iv_utMsgId(utMsgId) {
    if ( iv_utMsgId == 0) {
      iv_utMsgId = UIMA_MSG_ID_NO_MESSAGE_AVAILABLE;
    }
  }
///Constructor with a single char * parameter
  ErrorMessage::ErrorMessage(
    TyMessageId                          utMsgId,
    const char *                            cpszParam1
  ) :
      iv_utMsgId(utMsgId) {
    assert( iv_utMsgId != 0 );
    iv_vecParams.push_back((string)cpszParam1);
  }

///Constructor with a single string parameter
  ErrorMessage::ErrorMessage(
    TyMessageId           utMsgId,
    const string &           crstrParam1
  ) :
      iv_utMsgId(utMsgId) {
    assert( iv_utMsgId != 0 );
    iv_vecParams.push_back(crstrParam1);
  }

///Constructor with a single UChar * parameter
  ErrorMessage::ErrorMessage(
    TyMessageId                         utMsgId,
    const UChar *                          cpuszParam1
  ) :
      iv_utMsgId(utMsgId) {
    assert( iv_utMsgId != 0 );
    string s;
    UnicodeStringRef(cpuszParam1).extract(s);         // Convert to default encoding for platform
    iv_vecParams.push_back(s);
  }

///Constructor with a single icu::UnicodeString parameter
  ErrorMessage::ErrorMessage(
    TyMessageId                         utMsgId,
    const icu::UnicodeString &             crustrParam1
  ) :
      iv_utMsgId(utMsgId) {
    assert( iv_utMsgId != 0 );
    string s;
    UnicodeStringRef(crustrParam1).extract(s);         // Convert to default encoding for platform
    iv_vecParams.push_back(s);
  }

///Constructor with a single int parameter
  ErrorMessage::ErrorMessage(
    TyMessageId           utMsgId,
    int                      iParam1
  ) :
      iv_utMsgId(utMsgId) {
    assert( iv_utMsgId != 0 );
    string s;
    iv_vecParams.push_back(long2String(iParam1, s));
  }

///Constructor with a single unsigned int parameter
  ErrorMessage::ErrorMessage(
    TyMessageId           utMsgId,
    unsigned int             uiParam1
  ) :
      iv_utMsgId(utMsgId) {
    assert( iv_utMsgId != 0 );
    string s;
    iv_vecParams.push_back(long2String((int) uiParam1, s));
  }


///Constructor with a single long parameter
  ErrorMessage::ErrorMessage(
    TyMessageId           utMsgId,
    long                     lParam1
  ) :
      iv_utMsgId(utMsgId) {
    assert( iv_utMsgId != 0 );
    string s;
    iv_vecParams.push_back(long2String(lParam1, s));
  }

///Constructor with a single unsigned long parameter
  ErrorMessage::ErrorMessage(
    TyMessageId           utMsgId,
    unsigned long            ulParam1
  ) :
      iv_utMsgId(utMsgId) {
    assert( iv_utMsgId != 0 );
    string s;
    iv_vecParams.push_back(long2String((long) ulParam1, s));
  }

///Constructor with a single double parameter
  ErrorMessage::ErrorMessage(
    TyMessageId           utMsgId,
    const double             dParam1
  ) :
      iv_utMsgId(utMsgId) {
    assert( iv_utMsgId != 0 );
    string s;
    iv_vecParams.push_back(double2String(dParam1, s));
  }

///Constructor with a full parameter vector
  ErrorMessage::ErrorMessage(
    TyMessageId           utMsgId,
    const vector<string> &   crvecParams
  ) :
      iv_utMsgId(utMsgId),
      iv_vecParams(crvecParams) {
    assert( iv_utMsgId != 0 );
  }

  
  /*------------------------------- Constructors -------------------------------*/

  ErrorContext::ErrorContext(
    const ErrorMessage &     crclMessage,
    const char*                   pszFilename,
    const char*                   pszFunction,
    unsigned long                 ulLineNumber
  ):
      iv_clMessage(crclMessage),
      iv_pszFilename(pszFilename),
      iv_pszFunction(pszFunction),
      iv_ulLineNo(ulLineNumber) {}

#ifdef OS_STL
  Exception::~Exception()
#else
  Exception::~Exception() UIMA_THROW0()
#endif
  {}

  /*------------------------------ Output Support ------------------------------*/

  string
  ErrorMessage::asString() const {

    size_t numparams = getMessageParams().size();

    // Check for unknown message id
    if (iv_utMsgId < 0 || iv_utMsgId > UIMA_MSG_ID_SIGNATURE_END) {
      return string("Unknown message id " + iv_utMsgId);
    }

    //parameter substitution

    //locate message in message table
    const TCHAR ** messages = gs_aszMessageStringTable;
    const TCHAR * msg = messages[iv_utMsgId];

    TCHAR * buf = new TCHAR[UIMA_MSG_MAX_STR_LEN];
    memset(buf,'\0',UIMA_MSG_MAX_STR_LEN);

    int numwritten=0;

    TCHAR * trg = buf;
    while (*msg) {
      if (numwritten > UIMA_MSG_MAX_STR_LEN) {
        break;
      }
      if (*msg == UIMA_MSG_REPLACE_CHAR) {
        msg = msg+1;
        if (*msg == UIMA_MSG_REPLACE_CHAR) {
          *trg = *msg;
          trg = trg +1;
          ;
          msg = msg+1;
          numwritten++;
        } else {  //replace %n with the corresponding param
          unsigned long index;
          int len;
          string arg;
          // determine the number of the specified argument ...
          index = (unsigned long) atol(msg);
          if (index > numparams)       // param not defined
          {
            arg = "???";      // replace it by "dont-know"
          } else {
            // ... the indexed argument ...
            arg = iv_vecParams[index - 1]; // zero-based array!
            //assert(arg.length() > 0);
          }
          len = arg.length();
          // ... and then copy the argument
          if (UIMA_MSG_MAX_STR_LEN-numwritten < len) {
            len = UIMA_MSG_MAX_STR_LEN - numwritten;
          }
          if (len > 0) {
            strncpy(trg, arg.c_str(), len);
            trg = trg+len;
          }
          msg = msg+1;
          while ( isdigit(*msg) ) { //handle arg number 10 or more
            msg = msg+1;
          }
        }
      } else {
        *trg = *msg;
        trg = trg+1;
        msg = msg+1;
      }
    }
    //cout << buf << endl;
   string target(buf);
   delete[] buf ;
   return target;
	 //string target;
	 //target.assign(buf, UIMA_MSG_MAX_STR_LEN);                   // Copy the result to the string
   //delete buf;
	 //return target;
  }

  ostream &
  operator << (
    ostream &            os,
    const ErrorMessage & errorMsg
  ) {
    os << errorMsg.asString();
    return(os);
  }

  string
  ErrorContext::asString() const {
    string s;
    s += getMessage().asString();
    if (getFileName() != NULL) {
      s += string("\n") + ErrorInfo::getGlobalErrorInfoIndent() + "     File     : " + getFileName();
    }
    if (getFunctionName() != NULL) {
      s += string("\n") + ErrorInfo::getGlobalErrorInfoIndent() + "     Function : " + getFunctionName();
    }
    if (getLineNumber() != 0) {
      string sNum;
      long2String(getLineNumber(), sNum);
      s += string("\n") + ErrorInfo::getGlobalErrorInfoIndent() + "     Line     : " + sNum;
    }

    return(s);
  }

  ostream &
  operator << (
    ostream            & os,
    const ErrorContext & errContext
  ) {
    os << errContext.asString();
    return (os);
  }

  void ErrorMessage::reset(void) {
    iv_utMsgId = UIMA_MSG_ID_NO_MESSAGE_AVAILABLE;
    iv_vecParams.clear();
  }

  /* ----------------------------------------------------------------------- */
  /*   ErrorInfo                                                        */
  /* ----------------------------------------------------------------------- */

  /*------------------------------- Statics  -----------------------------------*/
//initialize static member var to an initial value

  const char * ErrorInfo::cv_cpszContextPrefix   = "   While      : ";
  const char * ErrorInfo::cv_cpszIndent          = "";


  /*------------------------------- Constructors -------------------------------*/

  ErrorInfo::ErrorInfo(
    const ErrorMessage     & rclMessage,
    TyErrorId                ulErrorId,
    ErrorInfo::EnSeverity    enSeverity
  ) :
      iv_clErrorMsg(rclMessage),
      iv_ulErrorId(ulErrorId),
      iv_enSeverity(enSeverity),
      iv_vecContexts() {}

  ErrorInfo::ErrorInfo( void ) :
      iv_clErrorMsg(),
      iv_ulErrorId(UIMA_ERR_NONE),
      iv_enSeverity(recoverable),
      iv_vecContexts() {}

  ErrorInfo::~ErrorInfo() {}

//constructor (copy)
//? Exception::Exception(
//?   const Exception&            rclException
//? )
//? {
//? }

  /*---------------------------- Exception Context ----------------------------*/
  void
  ErrorInfo::addContext( const ErrorContext & crclContext ) {
    iv_vecContexts.push_back(crclContext);
  }

  const ErrorContext *
  ErrorInfo::contextPtrAtIndex( size_t uiContextIndex ) const {
    if (uiContextIndex >= iv_vecContexts.size()) {
      return(NULL);
    }
    return(&iv_vecContexts[uiContextIndex]);
  }

  /*------------------------------ Output Support ------------------------------*/
  string
  ErrorInfo::asString() const {
    string s;
    if (getErrorId() == UIMA_ERR_NONE) {
      s += string("No Error\n");
      return (s);
    }
    if (getErrorId() != UIMA_ERR_NONE) {
      s += string("\n") + ErrorInfo::getGlobalErrorInfoIndent() + "Error number  : ";
      string sErr;
      long2String( getErrorId(), sErr);
      s += sErr;
    }
    s += string("\n") + ErrorInfo::getGlobalErrorInfoIndent() + "Recoverable   : " + (isRecoverable() ? "Yes" : "No");
    s += string("\n") + ErrorInfo::getGlobalErrorInfoIndent() + "Error         : " + getMessage().asString();
    for (size_t i = 0; i < contextCount(); ++i) {
      s += string("\n") + ErrorInfo::getGlobalErrorInfoIndent() + ErrorInfo::getGlobalErrorInfoContextPrefix();
      assert(EXISTS(contextPtrAtIndex(i)));   //lint !e666: Expression with side effects passed to repeated parameter 1 in macro EXISTS
      s += contextPtrAtIndex(i)->asString();
    }
    s += string("\n");

    return (s);
  }

///output support for streams
  ostream &
  operator << (
    ostream         & os,
    const ErrorInfo & errInfo
  ) {
    os << errInfo.asString();
    return(os);
  }

  void ErrorInfo::reset(void) {
    iv_clErrorMsg.reset();
    iv_ulErrorId = UIMA_ERR_NONE;
    iv_enSeverity = recoverable;
    iv_vecContexts.clear();
  }

  /* ----------------------------------------------------------------------- */
  /*   Exception                                                        */
  /* ----------------------------------------------------------------------- */

  Exception::Exception(
    const ErrorMessage    & rclMessage,
    TyErrorId                 ulErrorId,
    ErrorInfo::EnSeverity   enSeverity
  ) :
      EXCEPTION_BASE_CLASS(),
      iv_clErrorInfo(rclMessage, ulErrorId, enSeverity) {}

  Exception::Exception(
    const ErrorInfo & crclErrorInfo
  ) :
      EXCEPTION_BASE_CLASS(),
      iv_clErrorInfo(crclErrorInfo) {}

  /*------------------------------ Exception Type ------------------------------*/

  const char * Exception::getName() const {
    return(_TEXT("unspecified exception"));
  }

  /*------------------------- Application Termination --------------------------*/
  void Exception::terminate() {
    exit(iv_clErrorInfo.getErrorId());  //lint !e713: Loss of precision (arg. no. 1) (unsigned long to int)
  }

  /*------------------------------ Throw Support -------------------------------*/
  void Exception::assertParameter(
    const char*       /*exceptionText*/,
    ErrorContext   /*context*/ ) {
    assert(false);
    //not tested yet (and not used anywhere!)
  }

  /*------------------------- Logging Support ----------------------------------*/

  void Exception::logExceptionData() {
    util::Trace clTrace(util::enTraceDetailLow, UIMA_TRACE_ORIGIN, UIMA_TRACE_COMPID_EXCEPTIONS);

    clTrace.dump(_TEXT("Exception occured"), asString().c_str());
    //not implemented yet
  }

  /*------------------------------ Output Support ------------------------------*/
  string
  Exception::asString() const {
    //output the (class) name of the exception and its error info after it.
    return string("\n") +
           ErrorInfo::getGlobalErrorInfoIndent() +
           "Exception     : " +
           getName() +
           "\n" +
           getErrorInfo().asString() +
           "\n";
  }



  ostream &
  operator << (
    ostream            & os,
    const Exception & exception
  ) {
    //output the (class) name of the exception and its error info after it.
    os << exception.asString();
    return(os);
  }

  /*------------------------------ Static Method -------------------------------*/

  // Release contents of string container allocated by asString method

  void
  Exception::release(std::string & msg) {
    msg.clear();               // Empty string
    msg.reserve(1);            // Reduce capacity so will use internal buffer & free external one
  }

//private:
  /*----------------------------- Hidden Functions -----------------------------*/
  Exception& Exception::operator=( const Exception&   /*exc*/ ) {
    assert(false);
    return(*this);                                     //lint !e527  unreachable
  }  //lint !e1745: member not assigned by private assignment operator

  /* ----------------------------------------------------------------------- */
  /*   Implementations of predefined exceptions                              */
  /* ----------------------------------------------------------------------- */


  /*
    The following classes reimplement the ANSI standard exception hierarchy from
    stdexcept.h as UIMACPP exceptions with context and message id support
  */
  UIMA_EXC_CLASSIMPLEMENT(Uima_logic_error        ,Exception);
  UIMA_EXC_CLASSIMPLEMENT(Uima_runtime_error      ,Exception);

  UIMA_EXC_CLASSIMPLEMENT(Uima_domain_error       ,Uima_logic_error);
  UIMA_EXC_CLASSIMPLEMENT(Uima_invalid_argument   ,Uima_logic_error);
  UIMA_EXC_CLASSIMPLEMENT(Uima_length_error       ,Uima_logic_error);
  UIMA_EXC_CLASSIMPLEMENT(Uima_out_of_range       ,Uima_logic_error);

  UIMA_EXC_CLASSIMPLEMENT(Uima_range_error        ,Uima_runtime_error);
  UIMA_EXC_CLASSIMPLEMENT(Uima_overflow_error     ,Uima_runtime_error);
  UIMA_EXC_CLASSIMPLEMENT(Uima_underflow_error    ,Uima_runtime_error);


  /**
    The following exceptions are used by helper test classes that are no longer in the UIMACPP library:
          CommandLineDriver DocBuffer ParseHandlers
  */
  UIMA_EXC_CLASSIMPLEMENT(ConsoleAbortExc        ,Exception);
  UIMA_EXC_CLASSIMPLEMENT(ParseHandlerExc        ,Exception);
  UIMA_EXC_CLASSIMPLEMENT(ExcDocBuffer           ,Uima_out_of_range);

  /** code page conversion errors */
  UIMA_EXC_CLASSIMPLEMENT(CodePageConversionException, uima::Exception);
  /**
    The following exception is used to report failures from APR functions
  */
  UIMA_EXC_CLASSIMPLEMENT(AprFailureException, Exception);

  /*
    The following classes provide a starting point for an exception hierarchy
  */
//? UIMA_EXC_CLASSIMPLEMENT(ExcAssertionFailure, Exception);
  UIMA_EXC_CLASSIMPLEMENT(ExcIllFormedInputError , Uima_runtime_error);
  UIMA_EXC_CLASSIMPLEMENT(ExcInvalidParameter    , Uima_invalid_argument);
  UIMA_EXC_CLASSIMPLEMENT(ExcIndexOutOfRange     , Uima_out_of_range);
//? UIMA_EXC_CLASSIMPLEMENT(ExcDeviceError      ,Uima_runtime_error);
  UIMA_EXC_CLASSIMPLEMENT(ExcInvalidRequest      , Uima_runtime_error);
  UIMA_EXC_CLASSIMPLEMENT(ExcResourceExhausted   , Uima_runtime_error);
  UIMA_EXC_CLASSIMPLEMENT(ExcOutOfMemory         , ExcResourceExhausted);
//? UIMA_EXC_CLASSIMPLEMENT(ExcOutOfSystemResource, ResourceExhausted);
//? UIMA_EXC_CLASSIMPLEMENT(ExcOutOfWindowResource, ResourceExhausted);
  UIMA_EXC_CLASSIMPLEMENT(ExcFileNotFoundError   , Uima_runtime_error);

// Windows specific CException
  ExcWinCException::ExcWinCException(
    ErrorMessage             clMessage,
    TyErrorId                  ulErrorId,
    ErrorInfo::EnSeverity    enSeverity
  )
      : Uima_runtime_error (clMessage, ulErrorId, enSeverity) {
    ;
  }

  const char*
  ExcWinCException :: getName() const {
    return( "ExcWinCException" );
  }

  ExcWinCException::~ExcWinCException () CHILD_DESTRUCT_THROW0() {
    ;
  }

  ExcWinCException::ExcWinCException (const ExcWinCException & a) : Uima_runtime_error (a) {
    ;
  }


  // Windows exceptions can be mapped only when compiled with MS VC++
#if defined(_MSC_VER)

  void translation_func( unsigned int u, _EXCEPTION_POINTERS* pExp ) {
    const char * cpszMsg = NULL;
    switch (u) {
    case EXCEPTION_ACCESS_VIOLATION:
      cpszMsg ="\"ACCESS VIOLATION\"";
      break;
    case EXCEPTION_DATATYPE_MISALIGNMENT:
      cpszMsg ="\"DATATYPE MISALIGNMENT\"";
      break;
    case EXCEPTION_BREAKPOINT:
      cpszMsg ="\"BREAKPOINT\"";
      break;
    case EXCEPTION_SINGLE_STEP:
      cpszMsg ="\"SINGLE STEP\"";
      break;
    case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:
      cpszMsg ="\"ARRAY BOUNDS EXCEEDED\"";
      break;
    case EXCEPTION_FLT_DENORMAL_OPERAND:
      cpszMsg ="\"FLT DENORMAL OPERAND\"";
      break;
    case EXCEPTION_FLT_DIVIDE_BY_ZERO:
      cpszMsg ="\"FLT DIVIDE_BY ZERO\"";
      break;
    case EXCEPTION_FLT_INEXACT_RESULT:
      cpszMsg ="\"FLT INEXACT_RESULT\"";
      break;
    case EXCEPTION_FLT_INVALID_OPERATION:
      cpszMsg ="\"FLT INVALID OPERATION\"";
      break;
    case EXCEPTION_FLT_OVERFLOW:
      cpszMsg ="\"FLT OVERFLOW\"";
      break;
    case EXCEPTION_FLT_STACK_CHECK:
      cpszMsg ="\"FLT STACK_CHECK\"";
      break;
    case EXCEPTION_FLT_UNDERFLOW:
      cpszMsg ="\"FLT UNDERFLOW\"";
      break;
    case EXCEPTION_INT_DIVIDE_BY_ZERO:
      cpszMsg ="\"INT DIVIDE BY ZERO\"";
      break;
    case EXCEPTION_INT_OVERFLOW:
      cpszMsg ="\"INT OVERFLOW\"";
      break;
    case EXCEPTION_PRIV_INSTRUCTION:
      cpszMsg ="\"PRIV INSTRUCTION\"";
      break;
    case EXCEPTION_IN_PAGE_ERROR:
      cpszMsg ="\"IN PAGE_ERROR\"";
      break;
    case EXCEPTION_ILLEGAL_INSTRUCTION:
      cpszMsg ="\"ILLEGAL INSTRUCTION\"";
      break;
    case EXCEPTION_NONCONTINUABLE_EXCEPTION:
      cpszMsg ="\"NONCONTINUABLE EXCEPTION\"";
      break;
    case EXCEPTION_STACK_OVERFLOW:
      cpszMsg ="\"STACK OVERFLOW\"";
      break;
    case EXCEPTION_INVALID_DISPOSITION:
      cpszMsg ="\"INVALID DISPOSITION\"";
      break;
    case EXCEPTION_GUARD_PAGE:
      cpszMsg ="\"GUARD PAGE\"";
      break;
    case EXCEPTION_INVALID_HANDLE:
      cpszMsg ="\"INVALID HANDLE\"";
      break;
    case CONTROL_C_EXIT:
      cpszMsg ="\"CONTROL C EXIT\"";
      break;
    default:
      cpszMsg = "Unknows Windows C Exception";
      break;
    }
    // throw our own type of exception so we know at least what was going on
    // instead of just getting an unknown ... C++ excepition
    throw ExcWinCException( ErrorMessage(UIMA_MSG_ID_EXC_WINDOWS_EXCEPTION, cpszMsg),
                            (TyErrorId)UIMA_ERR_ENGINE_WINDOWS_EXCEPTION,
                            ErrorInfo::unrecoverable);
  }

#define UIMA_ENVVAR_DONT_MAP_WINDOWS_EXCEPTIONS     "UIMA_DONT_MAP_WINDOWS_EXCEPTIONS"

  // _set_se_translator should be called at the beginning of main
  // since we have no access to main here, we declare a static var of a dummy
  // type which does the _set_se_translator call in it's constructor
  // Note: this only works if the compiler properly executes the ctors of
  // such static vars in DLLs
  class Dummy {
  public:
    Dummy( void ) {
      if ( getenv(UIMA_ENVVAR_DONT_MAP_WINDOWS_EXCEPTIONS) == NULL) {
        _set_se_translator( translation_func );
      }
    }
  };
  // static var of our dummy type
  Dummy clDummy;
#endif

}
