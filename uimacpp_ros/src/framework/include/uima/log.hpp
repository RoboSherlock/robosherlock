/** \file log.hpp .
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

    \brief  Contains LogFacility a logging facility for (Annotator) users

   Description:

-----------------------------------------------------------------------------


   6/24/1999   Initial creation
   7/19/1999   logError(const ErrorInfo & crclErrorInfo) added

-------------------------------------------------------------------------- */

#ifndef UIMA_LOG_HPP
#define UIMA_LOG_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> //must be included first to disable warnings
#include <uima/types.h>
#include <uima/strtools.hpp>
#include "unicode/unistr.h"
#include <uima/exceptions.hpp>
#include "apr_pools.h"
#include "apr_proc_mutex.h"
#include <sstream>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class LogFacility;
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {

  /**
   * The class LogStream logs user-related information which would otherwise be
   * hard to concatenate into a single string.
   * Therefor it can be used whereever LogFacility::logMessage() is not
   * easy to use.
   */
  class UIMA_LINK_IMPORTSPEC LogStream : public virtual std::stringstream {
  public:
    friend class LogFacility;
    /// The types of log entries we distinguish
    enum EnEntryType {
      /// Messsage
      EnMessage,
      /// Warning
      EnWarning,
      /// Error
      EnError
    };
    /**
     * This function must be called to tell the stream that the end of
     * a message has been reached. It trigger the actual writing of the
     * acumulated stream content into the log.
     */
    LogStream & flush();
  private:
    /* --- variables --- */
    LogFacility           *  iv_pLogFacility;
    LogStream::EnEntryType   iv_enLogEntryType;
    /* --- functions --- */
    LogStream(LogFacility& logFacility, EnEntryType enLogEntryType):
        iv_pLogFacility(&logFacility),
        iv_enLogEntryType(enLogEntryType) {}
    /* DEFAULT CONSTRUCTOR NOT SUPPORTED */
    LogStream() {}; //lint !e1704
    /* COPY CONSTRUCTOR NOT SUPPORTED */
    LogStream(const LogStream & ); //lint !e1704
    /* ASSIGNMENT OPERATOR NOT SUPPORTED */
    LogStream & operator=(const LogStream & crclObject);
  }
  ; /* LogStream */


  /**
   * This defines the interface for Loggers.  To use a custom logger,
   * implement a class that implements this interface, and register
   * it using the ResourceManager::registerLogger(Logger *) method.
   */

  class UIMA_LINK_IMPORTSPEC Logger {
  public:
    /*
     *  Destructor
     *
     */
    virtual ~Logger() { }

    virtual void log(LogStream::EnEntryType entrytype, 
                  std::string classname,
                  std::string methodname,
                  std::string message,
                  long errorCode)  =0;

  };

  /** This class is the built in Logger that writes log
    * message to a file. The ResourceManager instantiates
    * and registers a FileLogger when the UIMACPP_LOGFILE
    * environment variable is set to the location of a
    * log file. 
    */
  class UIMA_LINK_IMPORTSPEC FileLogger : public  Logger {
    public:
      FileLogger(std::string filename);
      ~FileLogger() {fclose(iv_logfile);}
      
      virtual void log(LogStream::EnEntryType entrytype, 
                  std::string classname,
                  std::string methodname,
                  std::string message,
                  long errorCode) ;
      
    private:
      /** Format the log message */
      std::string format(LogStream::EnEntryType enType,
                        const std::string & cpszMsg, 
                        long lUserCode) ;

      FILE * iv_logfile;
  
  };
    


  /**
   * The class LogFacility logs user-related information
   * for debugging or error recording purposes.  The log messages
   * are written to one or more loggers registered with the
   * ResourceManager.
   * \code
     if(myErrorOccured)
        {
        // my annotator couldn't create a result file for this document:
        // report an error but continue execution
        getAnnotatorContext().getLogger().logError("Could not create result file");
        // continue execution
        }
     \endcode
   * To log more complex messages which might consist of the concatenation of
   * many elements you can use the LogStream provided by LogFacility:
   * \code
     if(myErrorOccured)
        {
        // my annotator couldn't create a result file for this document:
        // report an error but continue execution
        // get a error log stream object
        LogStream & errStream =
           getAnnotatorContext().getLogger().getLogStream(LogStream::EnError);
        // stream out error message
        errStream << "Could not create result file" << strFilename << endl;
        errStream << "(continuing execution)";
        // terminate error message
        errStream.flush();
        // continue execution
        }
     \endcode
   * Instead of the multi-line sequence of statements you can use one of the
   * defines we provide: UIMA_LOG_STREAM, UIMA_LOG_STREAM_MESSAGE,
   * UIMA_LOG_STREAM_WARNING and UIMA_LOG_STREAM_ERROR
   * The example above can be written with UIMA_LOG_STREAM_ERROR as follows
   * \code
     UIMA_LOG_STREAM_ERROR(getAnnotatorContext().getLogger(),
        "Could not create result file" << strFilename << endl
        << "(continuing execution)");
     \endcode
   */
  class UIMA_LINK_IMPORTSPEC LogFacility {
  public:
    /** @name Constructors */
    /*@{*/
    /** Create a new instance of a log facility. <TT>cpszAppKey</TT> must denote the
        key of the application. */
    /** this constructor gets handle to the loggers 
    * from the ResourceManager and sets the mininum log level to INFO.
    *  This is used in the AnnotatorContext to create separate instances
    *   of the  LogFacility for each annotator. */
    LogFacility(icu::UnicodeString const & );
    /** this constructor gets the handle to loggers from the ResourceManager.
        This constructor is used by the framework ResourceManager */
    LogFacility(icu::UnicodeString const &,  LogStream::EnEntryType level );
    ~LogFacility();
    /*@}*/
    /** @name Properties */
    /*@{*/
    const TCHAR *     getLastErrorAsCStr(void) const;
    ErrorInfo const & getLastError() const;
    /*@}*/
    /** @name LogStream Functions */
    LogStream &       getLogStream(LogStream::EnEntryType enLogEntryType);
    void              flushLogStream();
    /*@{*/
    /*@}*/
    /** @name Log Functions */
    /*@{*/

    /** Log the specified message together with a user-defined id. */
    void              logMessage(const TCHAR * cpszMessage, long lUserCode = 0) const;
    /** Log the specified message together with a user-defined id. */
    void              logMessage(const std::string & crclMessage, long lUserCode = 0) const;
    /** Log the specified message together with a user-defined id. */
    void              logMessage(const icu::UnicodeString & crclMessage, long lUserCode = 0) const;
    /** Log the specified warning together with a user-defined id. */
    void              logWarning(const TCHAR * cpszMessage, long lUserCode = 0) const;
    /** Log the specified warning together with a user-defined id. */
    void              logWarning(const std::string & crclMessage, long lUserCode = 0) const;
    /** Log the specified warning together with a user-defined id. */
    void              logWarning(const icu::UnicodeString & crclMessage, long lUserCode = 0) const;
    /** Log the specified warning. */
    void              logWarning(const ErrorInfo & crclErrorInfo) const;
    /** Log the specified error together with a user-defined id. */
    void              logError(const TCHAR * cpszMessage, long lUserCode = 0) const;
    /** Log the specified error together with a user-defined id. */
    void              logError(const std::string & crclMessage, long lUserCode = 0) const;
    /** Log the specified error together with a user-defined id. */
    void              logError(const icu::UnicodeString & crclMessage, long lUserCode = 0) const;
    /** Log the specified error. */
    void              logError(const ErrorInfo & crclErrorInfo) const;
    /**  */
    /*@}*/
  protected:
    /* --- functions --- */
  private:
    /* --- variables --- */
    std::string       iv_strOrigin;
    std::string       iv_strLastError;
    long              iv_lLastUserCode;
    ErrorInfo         iv_errInfo;
    LogStream         iv_logStream;
    LogStream::EnEntryType   iv_logLevel;
    std::vector<Logger*> & vecLoggers;
    
    /* --- functions --- */
    TyMessageId       getTypeAsMessageId(LogStream::EnEntryType enType) const;
    void              doLog(LogStream::EnEntryType enType, const TCHAR * cpszMsg, long lUserCode = 0) const;
    void              log(LogStream::EnEntryType enType, const TCHAR * cpszMsg, long lUserCode = 0) const;
    void              log(LogStream::EnEntryType enType, const icu::UnicodeString & crclMessage, long lUserCode = 0) const;
    void              log(LogStream::EnEntryType enType, ErrorInfo const & errInfo) const;

    void     setLogLevel(LogStream::EnEntryType) ;
    bool           isLoggable(LogStream::EnEntryType) const;


    /* COPY CONSTRUCTOR NOT SUPPORTED */
    LogFacility(const LogFacility & ); //lint !e1704
    /* ASSIGNMENT OPERATOR NOT SUPPORTED */
    LogFacility & operator=(const LogFacility & crclObject);
  }
  ; /* LogFacility */

  /**
   * Use this define to write a single statement log entry using streams
   * @see LogFacility
   */
#define UIMA_LOG_STREAM(logFacility, logEntryType, args)       \
   {                                                          \
     LogStream & _l = logFacility.getLogStream(logEntryType); \
     _l << args;                                              \
     _l.flush();                                              \
   }

  /**
   * Use this define to write a single statement message log entry using streams
   * @see LogFacility
   */
#define UIMA_LOG_STREAM_MESSAGE(logFacility, args) \
   UIMA_LOG_STREAM(logFacility, LogStream::EnMessage, args);

  /**
   * Use this define to write a single statement warning log entry using streams
   * @see LogFacility
   */
#define UIMA_LOG_STREAM_WARNING(logFacility, args) \
   UIMA_LOG_STREAM(logFacility, LogStream::EnWarning, args);

  /**
   * Use this define to write a single statement error entry using streams
   * @see LogFacility
   */
#define UIMA_LOG_STREAM_ERROR(logFacility, args) \
   UIMA_LOG_STREAM(logFacility, LogStream::EnError, args);

} // namespace uima

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {
  inline void LogFacility::logMessage(const TCHAR * cpszMessage, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnMessage, cpszMessage, lUserCode);
  }

  inline void LogFacility::logMessage(const std::string & crclMessage, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnMessage, crclMessage.c_str(), lUserCode);
  }

  inline void LogFacility::logMessage(const icu::UnicodeString & crclMessage, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnMessage, crclMessage, lUserCode);
  }

  inline void LogFacility::logWarning(const TCHAR * cpszMessage, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnWarning, cpszMessage, lUserCode);
  }

  inline void LogFacility::logWarning(const std::string & crclMessage, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnWarning, crclMessage.c_str(), lUserCode);
  }

  inline void LogFacility::logWarning(const icu::UnicodeString & crclMessage, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnWarning, crclMessage, lUserCode);
  }

  inline void LogFacility::logError(const TCHAR * cpszMessage, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnError, cpszMessage, lUserCode);
  }

  inline void LogFacility::logError(const std::string & crclMessage, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnError, crclMessage.c_str(), lUserCode);
  }

  inline void LogFacility::logError(const icu::UnicodeString & crclMessage, long lUserCode) const
  /* ----------------------------------------------------------------------- */
  {
    log(LogStream::EnError, crclMessage, lUserCode);
  }
}

/* ----------------------------------------------------------------------- */
#endif /* UIMA_LOG_HPP */

/* <EOF> */




