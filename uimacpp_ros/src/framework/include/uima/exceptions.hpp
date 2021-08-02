#ifndef UIMA_EXCEPTIONS_HPP
#define UIMA_EXCEPTIONS_HPP
/** \file exceptions.hpp .
******************************************************************************


    \brief  This file contains the base class for all exceptions used in UIMACPP.

 DESCRIPTION:
   Declaration of the classes:
     ErrorMessage
     ExceptionDescription
     Exception
     AccessError
     AssertionFailure
     DeviceError
     InvalidParameter
     InvalidRequest
     ResourceExhausted
     OutOfMemory
     OutOfSystemResource
     OutOfWindowResource

   This file also contains many of the macros used to implement the
   library exception handling mechanism.  This includes the UIMA_ASSERT,
   UIMA_EXC_THROW, UIMA_EXC_RETHROW, UIMA_EXC_CONTEXT,
   UIMA_EXCEPTION_DESCRIPTION, UIMA_EXC_CLASSDECLARE, UIMA_EXC_CLASSIMPLEMENT,
   macros.

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



********************************************************************************

This file contains the base class for all exceptions used in UIMACPP
project.  The design goal was to allow for an easy way to provide
rich context information with the exception without introducing the
need to write lots of lines of code every time you use an exception.

The way this is accomplished here is by defining a rich exception
class and then providing macros for its everyday use that hide its
complexity and automate a lot of processes.

A UIMACPP exception has an error info object as its main informational
member. Such an error info object containes a text, which is supposed to
be a simple description of the error with out context
(e.g. "Could not open file") an error number and an error severity
(recoverable or unrecoverable).  To this basic information a <EM>list</EM> of
error contexts can be added (a context has a text and a location, see class
<TT>ErrorContext</TT> above).  A context specifies what the program
was trying to do, when the error occurred (e.g. "Trying to open file
XYZ.ABC for reading") since very often not the whole context a user
might need to understand the error is locally available the exception
can be re-throw after adding a context. At the point where it is
caught more context can be added and again it can be re-thrown until
finally the exception can be taken care of (examined/displayed).

This process of defining/catching, adding context and then
(re)throwing is what is very convenient to do via the macros defined
below.

*/

#include <uima/pragmas.hpp> //must be included first to disable warnings
#include <string>
#include <vector>
#include <stdexcept>

#include <iostream>
#include <sstream>
#include <string>
#include <uima/strtools.hpp>
#include <uima/err_ids.h>
#include <uima/types.h>
#include "unicode/uchar.h"
#include "unicode/unistr.h"
#include <uima/unistrref.hpp>

#define UIMA_MSG_MAX_STR_LEN        4096
#define UIMA_MSG_REPLACE_CHAR       '%'

namespace uima {

  /** Class ErrorMessage: this is a helper class for main class
      <TT>ErrorInfo</TT> and <TT>ErrorContext</TT>. It bundles a message id
      and optional a list of string parameters for the message.
  */

  class UIMA_LINK_IMPORTSPEC ErrorMessage {
  public:
    /*------------------------------- Constructors -------------------------------*/
    ///Constructor with just the message id
    ErrorMessage(
      TyMessageId                      utMsgId = 0
    );

    ///Constructor with a single long parameter
    ErrorMessage(
      TyMessageId                      utMsgId,
      long                             lParam1
    );

    ///Constructor with a single unsigned long parameter
    ErrorMessage(
      TyMessageId                      utMsgId,
      unsigned long                    ulParam1
    );

    ///Constructor with a single int parameter
    ErrorMessage(
      TyMessageId                      utMsgId,
      int                              iParam1
    );

    ///Constructor with a single unsigned int parameter
    ErrorMessage(
      TyMessageId                      utMsgId,
      unsigned int                     uiParam1
    );

    ///Constructor with a single char * parameter
    ErrorMessage(
      TyMessageId                      utMsgId,
      const char *                     cpszParam1
    );

    ///Constructor with a single string parameter
    ErrorMessage(
      TyMessageId                      utMsgId,
      const std::string &              crstrParam1
    );

    ///Constructor with a single UChar * parameter
    ErrorMessage(
      TyMessageId                      utMsgId,
      const UChar *                    cpuszParam1
    );

    ///Constructor with a single icu::UnicodeString parameter
    ErrorMessage(
      TyMessageId                      utMsgId,
      const icu::UnicodeString &       crustrParam1
    );

    ///Constructor with a single double parameter
    ErrorMessage(
      TyMessageId                      utMsgId,
      const double                     dParam1
    );

    ///Constructor with a full parameter vector
    ErrorMessage(
      TyMessageId                      utMsgId,
      const std::vector< std::string > &  crvecParams
    );

    /*------------------------- addParams --------------------------------*/

	template <class T>
	void addParam(T p) {
	  std::ostringstream message;
      message << p;
      iv_vecParams.push_back(message.str());
	}
    /*------------------------- Attributes --------------------------------*/
    ///accessor for the message id
    TyMessageId                   getMessageID() const {
      return(iv_utMsgId);
    }

    ///accessor for the file name
    const std::vector< std::string > &  getMessageParams() const {
      return(iv_vecParams);
    }

    /*----------------------- Output Support ------------------------------*/
    ///formatted for error output of the context to string
    std::string asString() const;

    ///Reset method for clearing any error notifications
    void reset(void);
  private:
    /*-------------------------- Private ----------------------------------*/
    TyMessageId                  iv_utMsgId;   //The message id
    std::vector< std::string >   iv_vecParams; //An (optional) list of parameters
  }
  ;// ErrorMessage


  /**
  <TT>ErrorMessage</TT> output support for streams.

  Assuming your message is <TT>"Trying to load function %s"</TT> and
  the first argument is <TT>"begindoc"</TT>, your output should look like this:
  \code
  Trying to load function begindoc.
  \endcode
  */
  UIMA_LINK_IMPORTSPEC std::ostream &
  operator << (
    std::ostream & rclOStream,
    const ErrorMessage & crclExceptionMessage
  );





  /** Class ErrorContext: this is a helper class for main class
  <TT>Exception</TT>. It stores context information about an information:
  What the program tried, when an error occurred, the file where this
  occured etc. in short: where, why, how it went wrong.

  It should rarely be necessary to use this class directly, since most
  of the exceptions are designed to be used via macros, that will
  take care of creating context objects for you.  */

  class UIMA_LINK_IMPORTSPEC ErrorContext {
  public:

    /*------------------------------- Constructors -------------------------------*/
    ///Constructor
    ErrorContext(
      const ErrorMessage & clMsg            = ErrorMessage(),
      const char*                  pszFilename   = 0,
      const char*                  pszFunction   = 0,
      unsigned long                 ulLineNumber = 0
    );


    /*-------------------------------- Attributes --------------------------------*/
    ///accessor for the text
    const ErrorMessage &         getMessage() const;
    ///accessor for the file name
    const char *                 getFileName() const;
    ///accessor for the function name
    const char *                 getFunctionName() const;
    ///accessor for the line number
    unsigned long                getLineNumber() const;

    /*------------------------------ Output Support ------------------------------*/
    ///formatted for error output of the context to string
    std::string asString() const;

  private:
    /*--------------------------------- Private ----------------------------------*/
    ErrorMessage     iv_clMessage;
    const char *     iv_pszFilename;
    const char *     iv_pszFunction;
    unsigned long    iv_ulLineNo;

  }
  ;                                                    // ErrorContext


  /**
  <TT>ErrorContext</TT> output support for streams.
  This will first stream out the message part of the context and then (indented
  by spaces) the file name, function name and line number info (if present).
  This means you will typically get a multi-line text!

  If <TT>NDEBUG</TT> is defined your output should look like this:
  \code
  Trying to load function begindoc.
  \endcode

  In debug mode it will look like:
  \code
  Trying to load function begindoc.
        File    : extract.cpp
        Function: ExternalAnnotator::tryGetProcAddress(char*)
        Line    : 87
  \endcode
  */
  UIMA_LINK_IMPORTSPEC std::ostream &
  operator << (
    std::ostream & rclOStream,
    const ErrorContext & crclExceptionContext
  );

  /* ----------------------------------------------------------------------- */
  /*                                                                         */
  /* ----------------------------------------------------------------------- */


  /** Class <TT>ErrorInfo</TT>: This is the base class for all error information
  used in UIMACPP.

  An error info has a text, that is supposed to be a simple
  description of the error with out context (e.g. "Could not open file")
  and error number an error code groups and an error severity
  (recoverable or unrecoverable).  To this basic information a LIST of
  error contexts can be added (a context has a text and a location, see class
  <TT>ErrorContext</TT> above).

  This class is the main informational part in an <TT>Exception</TT>.
  But can be also used outside of exceptions in the case where you need
  to return detailed error information.
  For this second use we need the public setxxx methods for message, errorid and
  severity. The setxxx functions should not be used for <TT>ErrorInfo</TT> objects
  inside exceptions.
  */
  class UIMA_LINK_IMPORTSPEC ErrorInfo {
  public:
    ///enum used to specify severity
    enum EnSeverity {
      ///error is not recoverable
      unrecoverable,
      ///error is recoverable
      recoverable
    };

    /*------------------------------- Constructors -------------------------------*/
    /**
       Constructor.
       @param  rclMessage Message id describing <TT>what</TT> went wrong.
       @param  ulErrorId  Error number.
       @param  enSeverity Recoverability info.

       Note that the <TT>clMessage</TT> parameter should just state the plain info what went
       wrong (e.g. "Could not open file X").
       Use an exception context to add information why, how it went wrong
       (e.g.  "While trying to open application ini-file").
    */
    ErrorInfo(
      const ErrorMessage     & rclMessage,
      TyErrorId                ulErrorId,
      EnSeverity               enSeverity
    );

    /**
       Default Constructor (use outside exceptions only!).
       For uses of class ErrorInfo ouside exceptions you will want to
       define an empty (neutral, no error) ErrorInfo object which might
       get filled with error information by failing functions.
       This default constructor will create such a no-error ErrorInfo
       object for you.
    */
    ErrorInfo( void );

    ///Destructor
    virtual ~ErrorInfo();

    /*---------------------------- Exception Message -----------------------------*/
    ///Accessor for exception message
    const ErrorMessage & getMessage(void) const {
      return(iv_clErrorMsg);
    }
    ///Accessor for exception message
    void setMessage(const ErrorMessage & rclMessage);

    /*---------------------------- Exception Severity ----------------------------*/
    ///Accessor for exception severity: query
    virtual bool isRecoverable() const;


    ///Accessor for exception severity
    void setSeverity ( EnSeverity enSeverity );

    /*----------------------------- Error Identifier -----------------------------*/
    ///Accessor for error numbers: query
    TyErrorId getErrorId() const;

    ///Accessor for error numbers
    void setErrorId( TyErrorId ulErrorId );

    /*---------------------------- Exception Context ----------------------------*/
    ///Accessor for contexts: add
    virtual void addContext( const ErrorContext & crclContext );

    ///Accessor for contexts: query number
    size_t contextCount() const;

    ///Accessor for contexts: query a specific context
    const ErrorContext * contextPtrAtIndex( size_t uiContextIndex ) const;

    /*------------------------------ Output Support ------------------------------*/
    ///formatted for error output of the exception to a string
    std::string asString() const;

    ///Reset method for clearing any error notifications
    void reset(void);

    /*------------------------------ Static Methods ------------------------------*/

    /**@name Static methods.
    The following static methods can be called on the <TT>ErrorInfo</TT>
    class directly.
    */
    //@{

    ///Static method: set the error info context prefix. Default is" <TT>"\n   While: "</TT>
    static void setGlobalErrorInfoContextPrefix(const char* cpszContextPrefix);
    ///Static method: retrieve the error info context prefix.
    static const char * getGlobalErrorInfoContextPrefix();

    ///Static method: set the error info indent. Default is the empty string
    static void setGlobalErrorInfoIndent(const char* cpszIndent);
    ///Static method: retrieve the exception context prefix.
    static const char * getGlobalErrorInfoIndent();
    //@}

  private:

    /*--------------------------------- Private ----------------------------------*/

    ErrorMessage                 iv_clErrorMsg;
    TyErrorId                    iv_ulErrorId;
    EnSeverity                   iv_enSeverity;
    std::vector< ErrorContext >  iv_vecContexts;

    /*--------------------------------- Static Private ---------------------------*/

    static const char *                cv_cpszContextPrefix;
    static const char *                cv_cpszIndent;

    /*--------------------------------- Private functions ------------------------*/
  }
  ;                                                    // ErrorInfo

  /**
  <TT>ErrorInfo</TT> output support for streams.

  This will stream out:

  <UL>
  <LI> Name of the error
  <LI> Recoverability info
  <LI> Error number
  <LI> Error message
  <LI> The <TT>ErrorContext</TT> part of the exception
  </UL>
  This will give you a multi line message!

  If <TT>NDEBUG</TT> is defined your output should look like this:
  \code
  Recoverable   : No
  Error number  : -1
  Error         : Cannot find function address in DLL/library
     While      : Trying to load function begindoc
     While      : Trying to load Annotator IXTalent Test
     While      : Trying to initialize UIMACPP.
  \endcode

  In debug mode it will look like:
  \code
  Error number  : -1
  Recoverable   : No
  Error         : Cannot find function address in DLL/library
     While      : Trying to load function begindoc
        File    : extract.cpp
        Function: ExternalAnnotator::tryGetProcAddress(char*)
        Line    : 87
     While      : Trying to load Annotator IXTalent Test
        File    : extracts.cpp
        Function: Annotators::init(AnnotatorsConfig&)
        Line    : 116
     While      : Trying to initialize UIMACPP.
        File    : framewrk.cpp
        Function: Framework::init(const IString&)
        Line    : 128
  \endcode
  */
  UIMA_LINK_IMPORTSPEC std::ostream &
  operator << (
    std::ostream & rclOStream,
    const ErrorInfo & crclErrorInfo
  );



  /* ----------------------------------------------------------------------- */
  /*                                                                         */
  /* ----------------------------------------------------------------------- */

#if defined(OS_STL) // Object Space STL (older compilers)

  /** We have to take care of the name of the base class for all exceptions.
      ANSI named it simply <TT>exception</TT>, but this conflicts with some existing
      objects in older compilers (e.g. in <TT>math.h</TT> on unix).
      So for older stuff (Object Space STL based) we don't use the ANSI name
      of the base class, but the internal name of the Object Space base class.
  */
#if defined(__xlC__) && (__IBMCPP__  >  310)

  /** Define for base class of Exception for old IBM compiler*/
#define EXCEPTION_BASE_CLASS exception
#else

  /** Define for base class of Exception for Object Space STL */
#define EXCEPTION_BASE_CLASS os_exception
#endif
#else

  /** Define for base class of Exception ANSI compliant STL implementations (newer compilers GNU etc.) */
#define EXCEPTION_BASE_CLASS std::exception
#endif


  /**
     Class Exception: This is the base class for all exceptions
     used in UIMACPP group projects.

     An exception can be constructed from atext, that is supposed to be a
     simple description of the error with out context (e.g. "Could not open file")
     error number and an error severity (recoverable or unrecoverable).
     From this basic information a inital error info object is created which
     is a publicly avaiable member of the exception class.
     To this error info contexts can be added (
     a context has a text and a location, see class <TT>ErrorInfo</TT> above).
  */
  class UIMA_LINK_IMPORTSPEC Exception :
        public EXCEPTION_BASE_CLASS {
  public:
    /*------------------------------- Constructors -------------------------------*/
    /**
       Constructor.
       @param  rclMessage Message id describing <TT>what</TT> went wrong.
       @param  ulErrorId  Error number.
       @param  enSeverity Recoverability info.

       Note that the <TT>clMessage</TT> parameter should just state the plain info what went
       wrong (e.g. "Could not open file X").
       Use an exception context to add information why, how it went wrong
       (e.g.  "While trying to open application ini-file").
    */
    Exception(
      const ErrorMessage     & rclMessage,
      TyErrorId                ulErrorId,
      ErrorInfo::EnSeverity    enSeverity
    );

    /**
       Constructor from an existing error info object
    */
    Exception(const ErrorInfo & crclErrorInfo);

    ///Destructor
#ifdef OS_STL
    virtual ~Exception();
#else
    virtual ~Exception() UIMA_THROW0();
#endif


    /*------------------------------ Exception Type ------------------------------*/
    ///name of the exception (automatically set to the name of the class by our macros)
    virtual const char * getName() const;

    ///This is the ANSI standard exceptions way of returning the exception text
    virtual const char* what() const UIMA_THROW0() {
      return(getName());
    }

    /*------------------------- Application Termination --------------------------*/
    /**
       Called to terminate the program instead of actually throwing an exception
       if exception support is turned off*/
    virtual void terminate();

    /*------------------------- Logging Support ----------------------------------*/

    /**
       Called to write out the exception error message before terminate the program
       if exception support is turned off*/
    virtual void logExceptionData();

    /*------------------------------ Throw Support -------------------------------*/
    static void assertParameter(
      const char *         cpszExceptionText,
      ErrorContext clContext
    );

    /*------------------------------ Output Support ------------------------------*/
    ///formatted for error output of the exception to a string
    std::string asString() const;

    /*------------------------------ Access to Error Info  -----------------------*/
    /// return a reference to the error info member object
    ErrorInfo &
    getErrorInfo();
    /// return a const reference to the error info member object
    const ErrorInfo &
    getErrorInfo() const ;

    /*------------------------------ Static Method -------------------------------*/

    /**@name Static methods.
    The following static method can be called on the <TT>Exception</TT>
    class directly.
    */
    //@{

    /// Release contents of string container allocated by asString methods
    static void release(std::string & msg);

  private:
    /*----------------------------- Hidden Functions -----------------------------*/

    Exception &operator=( const Exception & exc );

    /*--------------------------------- Private ----------------------------------*/

    ErrorInfo   iv_clErrorInfo;

  }
  ;                                                    // Exception

  /**
  <TT>Exception</TT> output support for streams.

  This will stream out:

  <UL>
  <LI> Name of the exception
  <LI> Error number
  <LI> Recoverability info
  <LI> Exception message
  <LI> The <TT>ErrorContext</TT> part of the exception
  </UL>
  This will give you a multi line message!

  If <TT>NDEBUG</TT> is defined your output should look like this:
  \code
  Exception     : AnnotatorException
  Error number  : -1
  Recoverable   : No
  Error         : Cannot find function address in DLL/library
     While      : Trying to load function begindoc
     While      : Trying to load Annotator IXTalent Test
     While      : Trying to initialize UIMACPP.
  \endcode

  In debug mode it will look like:
  \code
  Exception     : AnnotatorException
  Error number  : -1
  Recoverable   : No
  Error         : Cannot find function address in DLL/library
     While      : Trying to load function begindoc
        File    : extract.cpp
        Function: ExternalAnnotator::tryGetProcAddress(char*)
        Line    : 87
     While      : Trying to load Annotator IXTalent Test
        File    : extracts.cpp
        Function: Annotators::init(AnnotatorsConfig&)
        Line    : 116
     While      : Trying to initialize UIMACPP.
        File    : framewrk.cpp
        Function: Framework::init(const IString&)
        Line    : 128
  \endcode
  */
  UIMA_LINK_IMPORTSPEC std::ostream &
  operator << (
    std::ostream & rclOStream,
    const Exception & crclException
  );



  /* ----------------------------------------------------------------------- */
  /*                                                                         */
  /* ----------------------------------------------------------------------- */

//no function and line information in ship version
#if defined( NDEBUG )

  /**
  This macro an exception context to be a <TT>ErrorMessage</TT>
  with automaticaly added file name, function name and line number information.
  The file and line info is optional (not present in <TT>NDEBUG</TT>).
  The function name is only used if the compiler supports the <TT>__FUNCTION__</TT> macro
  (of course) and can be turned of explicitely by defining
  <TT>UIMA_EXC_NO_FUNCTION_NAMES</TT>.
  */
#define UIMA_EXC_CONTEXT(cntxt) \
   uima::ErrorContext((uima::ErrorMessage)(cntxt), 0, 0, 0)
#elif defined ( UIMA_EXC_NO_FUNCTION_NAMES )
#define UIMA_EXC_CONTEXT(cntxt) \
   uima::ErrorContext((uima::ErrorMessage)(cntxt), __FILE__, 0, __LINE__)
#elif defined ( __FUNCTION__ )
#define UIMA_EXC_CONTEXT(cntxt)  \
   uima::ErrorContext((uima::ErrorMessage)(cntxt), __FILE__, __FUNCTION__, __LINE__)
#else
#define UIMA_EXC_CONTEXT(cntxt)   \
   uima::ErrorContext((uima::ErrorMessage)(cntxt), __FILE__, 0, __LINE__)
#endif

  /**This macro is intended to be used for adding context to exceptions.
     It adds a the contexts <TT>cntxt</TT>
     (file name, function name and line number are automatically added
     in debug mode = unless <TT>NDEBUG</TT> is defined)
     The context can be specified as a single message id or as Message id plus
     parameters. In the first case just write down the id. In the second case
     write this parameter like this <TT>ErrorMessage(utMsgId, cpszParam1)</TT>.
     Note that this macro is not needed if you use the <TT>UIMA_EXC_THROW</TT>,
     <TT>UIMA_EXC_RETHROW</TT> or <TT>UIMA_EXC_THROW_NEW</TT> macros. You only need it to add context
     without throwing the exception (e.g. just before printing the exception).
     @param exc   The exception (as defined in catch())
     @param cntxt The context string to add (string/char*)*/
#define UIMA_EXC_ADD_CONTEXT(exc, cntxt)\
  exc.getErrorInfo().addContext(UIMA_EXC_CONTEXT((ErrorMessage)(cntxt)))


  /**This macro is intended to be used for throwing exceptions that are
     already defined. It adds a the contexts string <TT>cntxt</TT>
     (file name, function name and line number are automatically added
     in debug mode = unless <TT>NDEBUG</TT> is defined)
     The context can be specified as a single message id or as Message id plus
     parameters. In the first case just write down the id. In the second case
     write this parameter like this <TT>ErrorMessage(utMsgId, cpszParam1)</TT>.
     @param exc   The exception (must already be defined)
     @param cntxt The context message to add */
#define UIMA_EXC_THROW(exc, cntxt)\
      exc.getErrorInfo().addContext(UIMA_EXC_CONTEXT(cntxt)),\
      exc.logExceptionData(),\
      throw(exc)

  /**This macro is intended to be used for re-throwing caught exceptions with
     added context. It adds a the contexts string <TT>cntxt</TT>
     (file name, function name and line number are automatically added
     in debug mode = unless <TT>NDEBUG</TT> is defined)
     The context can be specified as a single message id or as Message id plus
     parameters. In the first case just write down the id. In the second case
     write this parameter like this <TT>ErrorMessage(utMsgId, cpszParam1)</TT>.
     @param exc   The exception (as defined in catch())
     @param cntxt The context message to add */
#define UIMA_EXC_RETHROW(exc, cntxt)\
  exc.getErrorInfo().addContext(UIMA_EXC_CONTEXT(cntxt)),\
  exc.logExceptionData(),\
  throw exc

  /**This macro is intended to be used for throwing exceptions that are
     not yet defined. It defines an exception of type <TT>extype</TT> with
     error number <TT>errorNbr</TT>, error message <TT>errorMsg</TT>, context <TT>exContext</TT>
     and severity <TT>recoverable</TT>. (file name, function name and line number
     are automatically added to the context in debug mode = unless <TT>NDEBUG</TT>
     is defined)
     The context can be specified as a single message id or as Message id plus
     parameters. In the first case just write down the id. In the second case
     write this parameter like this <TT>ErrorMessage(utMsgId, cpszParam1)</TT>.
     @param exType      The type of the exception (e.g <TT>Exception</TT> or
                        <TT>FileNotFoundError</TT> see below)
     @param errorNbr    The error number  (long)
     @param errorMsg    The error message
     @param exContext   The context message to add
     @param recoverable The severity (<TT>Exception::recoverable</TT> or
                        <TT>Exception::unrecoverable</TT>)
     */
#define UIMA_EXC_THROW_NEW(exType, errorNbr, errorMsg, exContext, recoverable)\
   exType exc(errorMsg, errorNbr, recoverable); \
   UIMA_EXC_THROW(exc, exContext)

#if defined(DEBUG)
#define UIMA_EXC_ASSERT_EXCEPTION(test)\
   if(!(test))\
   {\
      Exception::assertParameter(   \
_TEXT("The following expression must be true, but evaluated to false: ") #test,\
      UIMA_EXC_EXCEPTION_CONTEXT());\
   }
#else
#define UIMA_EXC_ASSERT_EXCEPTION(test)
#endif


  // OS STL does not define exception destructor as exception::~exception() throw();
#ifdef OS_STL
#  define CHILD_DESTRUCT_THROW0()
#else
#  define CHILD_DESTRUCT_THROW0() UIMA_THROW0()
#endif
  /**This macro is intended to derive new exceptions from the base class
     (or an already derived exception class).
     This will typically be used in .h/.hpp files. For each use of
     <TT>UIMA_EXC_CLASSDECLARE</TT> in an .h/.hpp file there must be one use of
     <TT>UIMA_EXC_CLASSIMPLEMENT</TT> (see below) in the corresponding .c/.cpp file
     @param child  The new class name to define
     @param parent The class to derive the new one from */
#define UIMA_EXC_CLASSDECLARE(child,parent)                                    \
/*lint -save -e1932 -e1901 -e1931 -e754 -e19  */                              \
class UIMA_LINK_IMPORTSPEC child : public parent {                                                 \
public:                                                                       \
  child (                                                                     \
     uima::ErrorMessage          clMessage,                                    \
     uima::TyErrorId             ulErrorId,                                    \
     uima::ErrorInfo::EnSeverity enSeverity = uima::ErrorInfo::unrecoverable    \
  );                                                                          \
  child (                                                                     \
     uima::ErrorInfo             clErrInfo                                     \
  );                                                                          \
  virtual const char* getName() const;                                        \
  virtual ~child() CHILD_DESTRUCT_THROW0();                                   \
  child (const child &);                                                      \
private:                                                                      \
  child &operator = ( const child & );                                        \
}
  /*lint -restore */


  /**This macro is intended to derive new exceptions from the base class
     (or an already derived exception class).
     This will typically be used in .c/.cpp files. For each use of
     <TT>UIMA_EXC_CLASSIMPLEMENT</TT> in an .h/.hpp file there must be one use of
     <TT>UIMA_EXC_CLASSDECLARE</TT> (see above) in the corresponding .h/.hpp file
     @param child  The new class name to define
     @param parent The class to derive the new one from */
#define UIMA_EXC_CLASSIMPLEMENT(child, parent)             \
/*lint -save -e1901 -e1911 -e1746 -e1917 -e1902 */        \
  child :: child (                                        \
     uima::ErrorMessage             clMessage,             \
     uima::TyErrorId                ulErrorId,             \
     uima::ErrorInfo::EnSeverity    enSeverity             \
   )                                                      \
    : parent (clMessage, ulErrorId, enSeverity)           \
  {;}                                                     \
  child :: child (                                        \
     uima::ErrorInfo                clInfo                 \
   )                                                      \
    : parent (clInfo)                                     \
  {;}                                                     \
  const char* child :: getName() const {                  \
     return ( # child);                                   \
  }                                                       \
  child :: ~ child () CHILD_DESTRUCT_THROW0() {;}         \
  child::child (const child & a) : parent (a) {;}         \
/*lint -restore */



  /**@name Predefined Exceptions (beginning of an error hierarchy).
  The following predefined exceptions are supposed to structure the use
  of exceptions, help with avoiding multiple definitions of common
  exceptions (e.g. file not found) and provide a start for an exception
  hierarchy.
  */
//@{
  /**
    The following classes reimplement the ANSI standard exception hierarchy from
    stdexcept.h as exceptions with context and message id support.
    This is the reason why they have the "unconventional" names: Uima_<ansi name>
  */
///logic error
  UIMA_EXC_CLASSDECLARE(Uima_logic_error,      Exception);
///runtime error
  UIMA_EXC_CLASSDECLARE(Uima_runtime_error,    Exception);

///domain error
  UIMA_EXC_CLASSDECLARE(Uima_domain_error,     Uima_logic_error);
///invalid argument
  UIMA_EXC_CLASSDECLARE(Uima_invalid_argument, Uima_logic_error);
///length error
  UIMA_EXC_CLASSDECLARE(Uima_length_error,     Uima_logic_error);
///out of range
  UIMA_EXC_CLASSDECLARE(Uima_out_of_range,     Uima_logic_error);

///range error
  UIMA_EXC_CLASSDECLARE(Uima_range_error,      Uima_runtime_error);
///overflow error
  UIMA_EXC_CLASSDECLARE(Uima_overflow_error,   Uima_runtime_error);
///underflow error
  UIMA_EXC_CLASSDECLARE(Uima_underflow_error,  Uima_runtime_error);


  /**
    The following exceptions are used by helper test classes that are no longer in the UIMACPP library:
          CommandLineDriver DocBuffer ParseHandlers
  */
  UIMA_EXC_CLASSDECLARE(ConsoleAbortExc,         Exception);
  UIMA_EXC_CLASSDECLARE(ExcDocBuffer,            Uima_out_of_range);
  UIMA_EXC_CLASSDECLARE(ParseHandlerExc,         Exception);

  /**
    The following classes provide a starting point for an exception hierarchy
  */
///Access Error
//? UIMA_EXC_CLASSDECLARE(ExcAccessError,      Uima_runtime_error);
///Assertion Failure
//? UIMA_EXC_CLASSDECLARE(ExcAssertionFailure, Exception);
///Ill formed input
  UIMA_EXC_CLASSDECLARE(ExcIllFormedInputError,  Uima_runtime_error);
///Invalid Parameter
  UIMA_EXC_CLASSDECLARE(ExcInvalidParameter,     Uima_invalid_argument);
///Index out of Range
  UIMA_EXC_CLASSDECLARE(ExcIndexOutOfRange,      Uima_out_of_range);
///Device Error
//? UIMA_EXC_CLASSDECLARE(ExcDeviceError,      _runtime_error);
///Invalid Request
  UIMA_EXC_CLASSDECLARE(ExcInvalidRequest,       Uima_runtime_error);
///Resource Exhausted
  UIMA_EXC_CLASSDECLARE(ExcResourceExhausted,    Uima_runtime_error);
///Out of Memory
  UIMA_EXC_CLASSDECLARE(ExcOutOfMemory,          ExcResourceExhausted);
///Out of System Resource
//? UIMA_EXC_CLASSDECLARE(ExcOutOfSystemResource, ResourceExhausted);
///Out of Window Resource
//? UIMA_EXC_CLASSDECLARE(ExcOutOfWindowResource, ResourceExhausted);
///File not Found
  UIMA_EXC_CLASSDECLARE(ExcFileNotFoundError,    Uima_runtime_error);
// Windows specific CException
  UIMA_EXC_CLASSDECLARE(ExcWinCException,        Uima_runtime_error);

  /** Used to report code page conversion errors */
  UIMA_EXC_CLASSDECLARE(CodePageConversionException, Exception);

//@}


  /**
    The following exception is used to report failures from APR functions
  */
  UIMA_EXC_CLASSDECLARE(AprFailureException, Exception);


  /**
  @name Example of using the Exception class/macros
  The following example might help to understand how to use the Exception
  class/macros:

  One of the first things to do during initialization of your program is to set
  the message catalog used for exception messages. lets assume <TT>crclMessageCatalog</TT>
  is your initialized and valid message catalog object. So in your init code
  somewhere you should add:
  \code

     Exception::setGlobalExceptionMessageCatalog(crclMessageCatalog);

  \endcode
  This must happen BEFORE THE FIRST EXCEPTION IS THROWN. Otherwise all you will get
  is just a "Message catalog not found" instead of your intended error message,

  To declare a new exception class, use the <TT>UIMA_EXC_CLASSDECLARE</TT> macro in an .h/.hpp
  file:
  \code
     #include "uima/exceptions.hpp"

     UIMA_EXC_CLASSDECLARE(AnnotatorException, InvalidRequest);
  \endcode
  You can use the <TT>Exception</TT> class as it is, or use one of the predefined
  classes for you exceptions. But you can also define your own exceptions.
  To define a new exception class use the <TT>UIMA_EXC_CLASSIMPLEMENT</TT> macro in the
  corresponding .c/.cpp file:
  \code
     #include "uima/exceptions.hpp"

     UIMA_EXC_CLASSIMPLEMENT(AnnotatorException, InvalidRequest);
  \endcode
  Later you can use the new exception like this:
  \code
     if (pFunction == NULL) {
        UIMA_EXC_THROW_NEW(AnnotatorException,           //exception class defined above
                         UIMA_Rc_ErrorDLLNotFound,         //error number defined somewhere
                         UIMA_MsgId_ErrorDLLNotFound,      //error message msg id (what went wrong)
                         ErrorMessage(UIMA_MsgId_ContextFunctionLoad, cpszFuncName), //exception context for this error (where, why, how it went wrong)
                         Exception::unrecoverable);  //recoverability info
      }
  \endcode
  This will create a new <TT>AnnotatorException</TT> object, with the error code
  <TT>UIMA_Rc_Error</TT>, a text "Cannot find function address in DLL/library", the
  context "Trying to load function xxxx" and the severity unrecoverable and
  throw this new exception.
  Note: Since we have a parameter for the context message we write it as
  <TT>ErrorMessage(msgid, param)</TT>

  You can catch this function and add to its context in a calling function like
  that:
  \code
     try {
        elementAt(cursor)->setConfig(config);  //this where UIMA_EXC_THROW_NEW is used
     }
     catch (Exception &e) {
        UIMA_EXC_RETHROW(e, ErrorMessage(UIMA_MsgId_ContextAnnotatorLoad, elementAt(cursor)->getIdString()));
     }
  \endcode
  This will do nothing to correct the error, or inform the user (that might
  not be possible or appropriate at this point) but it will add the context
  "Trying to load Annotator xxxx" to the exception.
  Note: Since we have a parameter for the context message we write it as
  <TT>ErrorMessage(msgid, param)</TT>

  In a calling function of this code you could finally write:
  \code
     try {
        mExecutionModules.init(mConfig);
     }
     catch (Exception &e) {
        UIMA_EXC_ADD_CONTEXT(e, UIMA_MsgId_ContextUimaInit);
        cerr << e << endl;
        return e.getErrorId();
     }
  \endcode
  This will add the context "Trying to init UIMACPP" to the exception.
  Note: Since we do NOT have a parameter for the context message we can simply
  write it as <TT>msgid</TT>


  If <TT>NDEBUG</TT> is defined your output should look like this:

  ...
  \code
  Exception     : AnnotatorException
  Recoverable   : No
  Error number  : -1
  Error         : Cannot find function address in DLL/library
     While      : Trying to load function begindoc
     While      : Trying to load Annotator IXTalent Test
     While      : Trying to initialize UIMACPP.
  \endcode

  In debug mode it will look like:
  \code
  Exception     : AnnotatorException
  Recoverable   : No
  Error number  : -1
  Error         : Cannot find function address in DLL/library
     While      : Trying to load function begindoc
        File    : extract.cpp
        Function: ExternalAnnotator::tryGetProcAddress(char*)
        Line    : 87
     While      : Trying to load Annotator IXTalent Test
        File    : extracts.cpp
        Function: Annotators::init(AnnotatorsConfig&)
        Line    : 116
     While      : Trying to initialize UIMACPP.
        File    : framewrk.cpp
        Function: Framework::init(const IString&)
        Line    : 128
	\endcode
  */
//@{
//@}


  /* ----------------------------------------------------------------------- */
  /*       INLINE Implementation                                             */
  /* ----------------------------------------------------------------------- */

  inline const ErrorMessage &
  ErrorContext::getMessage() const {
    return(iv_clMessage);
  }

  inline const char *
  ErrorContext::getFileName() const {
    return(iv_pszFilename);
  }

  inline const char *
  ErrorContext::getFunctionName() const {
    return(iv_pszFunction);
  }

  inline unsigned long
  ErrorContext::getLineNumber() const {
    return(iv_ulLineNo);
  }

  /* ----------------------------------------------------------------------- */
  /*   ErrorInfo inline functions                                       */
  /* ----------------------------------------------------------------------- */



  inline void
  ErrorInfo::setGlobalErrorInfoContextPrefix(
    const char* cpszContextPrefix
  ) {
    cv_cpszContextPrefix = cpszContextPrefix;
  }

  inline const char *
  ErrorInfo::getGlobalErrorInfoContextPrefix() {
    return(cv_cpszContextPrefix);
  }

  inline void
  ErrorInfo::setGlobalErrorInfoIndent(
    const char* cpszIndent
  ) {
    cv_cpszIndent = cpszIndent;
  }

  inline const char *
  ErrorInfo::getGlobalErrorInfoIndent() {
    return(cv_cpszIndent);
  }

  inline void ErrorInfo::setMessage( const ErrorMessage & rclMessage ) {
    iv_clErrorMsg = rclMessage;
  }

  inline void ErrorInfo::setSeverity ( EnSeverity enSeverity ) {
    iv_enSeverity = enSeverity;
  }

  inline bool ErrorInfo::isRecoverable() const {
    return(iv_enSeverity == ErrorInfo::recoverable);
  }

  inline void ErrorInfo::setErrorId( TyErrorId ulErrorId ) {
    iv_ulErrorId = ulErrorId;
  }

  inline TyErrorId ErrorInfo::getErrorId() const {
    return(iv_ulErrorId);
  }

  inline size_t ErrorInfo::contextCount() const {
    return(iv_vecContexts.size());
  }

  /* ----------------------------------------------------------------------- */
  /*   Exception inline functions                                       */
  /* ----------------------------------------------------------------------- */


  inline ErrorInfo &
  Exception::getErrorInfo() {
    return iv_clErrorInfo;
  }

  inline const ErrorInfo &
  Exception::getErrorInfo() const {
    return iv_clErrorInfo;
  }

}
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */


#endif /* UIMA_EXCEPTIONS_HPP */

/* <EOF> */

