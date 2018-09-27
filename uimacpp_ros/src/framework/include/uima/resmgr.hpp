/** \file resmgr.hpp .
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

    \brief  UIMACPP Resource Manager ResourceManager

   Description:

-----------------------------------------------------------------------------


   4/14/1999   co Initial creation
   7/13/1999   co hasInstance() added

-------------------------------------------------------------------------- */

#ifndef UIMA_RESMGR_HPP
#define UIMA_RESMGR_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp> //must be included first to disable warnings
#include <uima/types.h>
#include <uima/err_ids.h>
#include <uima/log.hpp>

#include <vector>
#include <map>

#include "unicode/unistr.h"

#include <uima/assertmsg.h>
#include <uima/location.hpp>
#include <uima/dllfile.hpp>



/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */
namespace uima {
  #define UIMA_XML_NAMESPACE       "http://uima.apache.org/resourceSpecifier"
  #define UIMA_XSD_FILENAME        "resourceSpecifierSchema.xsd"

  const size_t                     UIMA_RESOURCEMANAGER_PREFIX_MAX_LENGTH = 3;

}

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
//   class ResourceManagerImp;   // forward declaration
  class ResourceFactoryABase; // forward declaration
  class ResourceABase;        // forward declaration

  class Language;             // forward declaration
  class ErrorInfo;            // forward declaration

  namespace internal {
    class AnnotatorFile;           // forward declaration
  }
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  /**
   * The class <TT>ResourceManager</TT> is used to manage the UIMACPP resources.
   * The class <TT>ResourceManager</TT> is a singleton. For example,  there can only be
   * one instance of the class per process.
   * The first argument <TT>cpszInstance</TT> defines the instance name of this instance.
   *
   * The second argument <TT>cpszProductPrefix</TT> is optional and defines the product prefix.
   * Both arguments are no longer used.
   *
   * \code
     foo(const TCHAR * cpszInstance)
     {
        // first create an instance of the UIMACPP resource manager
        ResourceManager & rclUimaResourceMgr = ResourceManager::createInstance(cpszInstance);

        if(rclUimaResourceMgr.getLastErrorId() != UIMA_ERR_NONE)
           {
              ErrorMessage clMessage( rclUimaResourceMgr.getLastErrorId() );
              cout << clMessage << endl;
           }
     }
     \endcode
   * The single instance of <TT>ResourceManager</TT> is deleted automatically.
   */
  class UIMA_LINK_IMPORTSPEC ResourceManager {
  public:
    /** Creates a filename for a resource file based on the specified language
        and the specified filename extension. Returns TRUE, if a file with this
        filename exists.

        Example: language is "en-us" and extension is ".tsw" filename will be en-us.tsw

        If <TT>bUseAlternateTerritories</TT> is enabled, another attempt is made, only
        using the language name without the territory.

        Example: language is "en" and the extension is ".tsw". The filename could also
        be en-uk.tsw */
    static bool createFilenameForLanguage(Language & rclLanguage,
                                          const TCHAR * cpszExtension,
                                          bool bUseAlternateTerritories,
                                          const util::Location & crclDirToUse,
                                          util::Filename & rclFilename);
  private:
    ~ResourceManager(void);
  public:
    /** @name Properties */
    /*@{*/
    /** Returns the last error that occurred. */
    TyErrorId                getLastErrorId(void) const                   {
      return(iv_utLastErrorId);
    }
    /** Returns the location used for storing temporary work files. */
    const util::Location &   getLocationWork(void) const;
    /** Returns the location used for storing data files. */
    const util::Location &   getLocationData(void) const;
    /** Returns the location used to find the schema definition file
    used for schema validation when parsing descriptors. */
    bool                     isSchemaAvailable(void);
    TCHAR const *            getSchemaInfo(void);
    void enableSchemaValidation(bool aEnable);
    bool doSchemaValidation(void);


    /** get the current logging level */
    LogStream::EnEntryType getLoggingLevel();
    /** set the current minimum logging level */
    void setLoggingLevel(LogStream::EnEntryType level);
    /** store handle to java env and handle to java log method */
    void setupJavaLogging(void * jniEnv);
    /** true is java logging is setup */
    bool isJavaLoggingEnabled();
    /** returns handle to the framework logger */
    uima::LogFacility &  getLogger();

    /** Returns an AnnotatorFile from the pool of AnnotatorFiles with the specified filename. */
    util::DllProcLoaderFile *   requestAnnotatorFile(const util::Filename & crclFilename);
    /* */

    /**
     * Convenience function.
     * Extracts the string from crLang and calls getResource().
     */
    ResourceABase const * getResource(uima::Language const & crLang,
                                      ResourceFactoryABase const & crFactory,
                                      ErrorInfo & );

    /**
     * Main method to create/access resources.
     * If a resource with the same key and kind as the factory
     * already exists, it is returned, otherwise a new one is
     * created.
     */
    ResourceABase const * getResource(icu::UnicodeString const & crKey,
                                      ResourceFactoryABase const & crFactory,
                                      ErrorInfo &);

    /**
     * Register a factory for a given kind of ressource.
     * Only after registering a factory for a kind a resource can be
     * acquired using the getResource() overload below
     */
    void registerFactory(icu::UnicodeString const & crKind, ResourceFactoryABase & crFactory);

    /**
     * Deregister a factory for a given kind of ressource.
     */
    void deRegisterFactory(icu::UnicodeString const & crKind, ResourceFactoryABase & crFactory);

    /**
     * get or create a ressource of a given kind and key using
     * a factory registered earlier using registerFactory
     */
    ResourceABase const * getResource(icu::UnicodeString const & crKey,
                                      icu::UnicodeString const & crKind,
                                      ErrorInfo &);
    /*@}*/
    /** @name Miscellaneous */
    /*@{*/
    /** Sets a new location to store temporary work files. */
    void                    setNewLocationWork(const util::Location & crclLocation);
    /** Sets a new location to store data files. */
    void                    setNewLocationData(const util::Location & crclLocation);
    /*@}*/
    /** @name Static Functions */
    /*@{*/
    /** Returns a pointer to the singleton instance of ResourceManager for
        this process and passes the instance key to it.
        <B>Note:</B> This function must be called before any calls to
        static method ResourceManager::getInstance()/0. */
    static ResourceManager &createInstance(const TCHAR * cpszInstance, const TCHAR * cpszProductPrefix = 0);
    /** Returns a pointer to the singleton instance of ResourceManager for
        this process. */
    static ResourceManager &getInstance(void);
    /** Return TRUE, if the resource manager has been initialized. */
    static bool             hasInstance(void);
    /** Deletes the resource manager instance. */
    static void             deleteInstance(void);


    /**
      * Look up the streamhandler dll that supports the URI scheme
      * 
      * @param uriScheme - eg file, sdo.
      * @return valid pointer to a dll file or NULL if URI scheme is unknown.
      *
      */
    util::Filename const * getStreamHandlerForURIScheme(std::string uriScheme );

    static icu::UnicodeString resolveFilename(icu::UnicodeString const & filename,
        icu::UnicodeString const & lastFilename);


    /**
      * Register Logger. Caller owns the logger instance.
      */
     void registerLogger (Logger *);
    /**
      * unregister this logger. Caller owns logger instance.
      */
      void unregisterLogger(Logger *);

    std::vector<Logger*> & getLoggers();

    /*@}*/
  protected:
    /* --- functions --- */
  private:
    static ResourceManager *cv_pclSingletonInstance;
    TCHAR                   iv_acPrefix[UIMA_RESOURCEMANAGER_PREFIX_MAX_LENGTH + 1];
    TyErrorId               iv_utLastErrorId;
    ////bool                    iv_bIgnoreAnnotatorPathSpec;


    typedef std::vector<ResourceABase *> TyResourceList;
    typedef std::map<icu::UnicodeString, TyResourceList > TyResources;
    TyResources iv_resources;

    typedef std::map<icu::UnicodeString, ResourceFactoryABase *> TyResourceFactories;
    TyResourceFactories iv_resourceFactories;


    typedef std::map<std::string, util::Filename *> TyURIStreamHandlers;
    std::map<std::string, util::Filename*>   iv_streamhandlers;


    util::Location          iv_locationWork;
    util::Location          iv_locationData;
    bool                    bIsSchemaAvailable;
    bool                    bDoSchemaValidation;
    TCHAR                   schemaInfo[1024];

    //Logging
    std::vector<Logger *>        iv_loggers;    //registered loggers
    FileLogger *            iv_fileLogger; //created if UIMACPP_LOGFILE env variable is set.

    LogStream::EnEntryType  iv_logLevel;  // minimum level for message to be logged
    uima::LogFacility      * iv_frameworkLogger; //instance of log facility used by the
                                                 //framework.
   

    /* --- friends --- */
    friend class ResourceManagerAutomaticInstanceDestructor;
    friend class LogFacility;
    /* --- functions --- */
    ResourceManager(const TCHAR * cpszInstance, const TCHAR * cpszProductPrefix);
    /* COPY CONSTRUCTOR NOT SUPPORTED */
    ResourceManager(const ResourceManager & ); //lint !e1704
    /* ASSIGNMENT OPERATOR NOT SUPPORTED */
    ResourceManager & operator=(const ResourceManager & crclObject);

    void deleteResourceList( TyResourceList & );

    /**
     * Register a stream handler dll filename for a given URI scheme.
     * A URI scheme may be registered only once in an application. 
     * @param uriScheme - UTF-8 string
     *        dllFIlename - UTF-8 filename of dll containing the
     *                      streamhandler implementation for the
     *                      URI scheme.
     * @return - a valid file pointer or NULL if it failed. 
     * NOTE: file URI scheme will use the built in handler.
     */
    util::Filename const * registerStreamHandlerForURIScheme(TCHAR const * uriScheme,
        TCHAR const * dllFilename);


  }
  ;                                                 /* ResourceManager */
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

#endif /* UIMA_RESMGR_HPP */

/* <EOF> */


