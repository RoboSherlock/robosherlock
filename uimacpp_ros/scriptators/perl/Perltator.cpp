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

#include <EXTERN.h>
#include <perl.h>

#define THREAD_PROTECTION
#ifdef THREAD_PROTECTION
#include "ThreadAnnotator.h"
#endif

using namespace uima;
using namespace std;

#define MODULENAME "Perltator"

// copied from SwigGenerated code

// this function requires the SWIG code to be compiled with
// SWIGRUNTIME defined to be "", otherwise this function is static
// and does not scope outside of the library
#ifndef SWIG_GLOBAL 
// SWIG 1.3.25 or better
#include "uimaperlwrap.h" 
#else
// Oh so last week, SWIG 1.3.21 style
struct swig_type_info;
struct swig_module_info;
extern "C" SV *SWIG_Perl_NewPointerObj(void *, swig_type_info *, int);
extern "C" swig_type_info * SWIG_Perl_TypeQuery(const char *);
#define SWIG_Perl_NewPointerObj(a,b,c) SWIG_Perl_NewPointerObj(a,b,c)
#define SWIG_TypeQueryModule(a,b,c) SWIG_Perl_TypeQuery(c)
#define SWIG_Perl_GetModule() ((swig_module_info *) 1)
#endif


EXTERN_C void boot_DynaLoader (pTHX_ CV* cv);
EXTERN_C void boot_perltator (pTHX_ CV* cv);

EXTERN_C void
xs_init(pTHX)
{
  char *file = __FILE__;
  /* DynaLoader is a special case */
  newXS("DynaLoader::boot_DynaLoader", boot_DynaLoader, file);
  /* we will always need UIMA */
  newXS("uimac::boot_perltator", boot_perltator, file);
}

class Perltator : public Annotator {
  int debug;
  swig_type_info *cas_type, *rs_type, *ts_type;
  PerlInterpreter *my_perl;

public:
  Perltator() : my_perl(0) {}

  // We construct a perl interpreter in initialize - it lives for the
  // life of the annotator - even if reconfigure happens.  Reconfigure
  // and intialize both set dirty so the source code in the source file
  // and contained in the type system are evaluated.

  TyErrorId initialize(AnnotatorContext &ac) {
    swig_type_info *ac_type;
    char srcfile[1000 + 256];

    if (ac.isParameterDefined("DebugLevel")) {
      ac.extractValue("DebugLevel", debug);
    }
    if (debug > 1) {
      cerr<< MODULENAME ": Initialize - debug=" << debug <<endl;
    }
    if (!ac.isParameterDefined("SourceFile")) {
      cerr<< MODULENAME ": Missing Perl SourceFile" <<endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    UnicodeString fn;
    ac.extractValue(UnicodeString("SourceFile"), fn);
    if (fn == "") {
      cerr<< MODULENAME ": Empty Perl SourceFile" <<endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    UErrorCode err = U_ZERO_ERROR;
    fn.extract(srcfile,sizeof(srcfile),0,err);
    if (U_FAILURE(err)) {
      cerr << MODULENAME ": Unable to extract parameter, got " << u_errorName(err) << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    if (my_perl == 0) {
      my_perl = perl_alloc();
      perl_construct(my_perl);
      
      char * my_argv[] = { "", "-S", srcfile };
      perl_parse(my_perl, xs_init, 3, my_argv, (char **) NULL);
      PL_exit_flags |= PERL_EXIT_DESTRUCT_END;

      perl_run(my_perl);


      // convert cas and rs to python variables (parameters) 
      swig_module_info *module = SWIG_Perl_GetModule();
      if (!module) {
        cerr << MODULENAME ": could not get Perl swig module" << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }

      ts_type = SWIG_TypeQueryModule(module,module, "TypeSystem *");
      if (!ts_type) {
        cerr << MODULENAME ": could lookup TypeSystem type in swig" << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      ac_type = SWIG_TypeQueryModule(module,module, "AnnotatorContext *");
      if (!ac_type) {
        cerr << MODULENAME ": could lookup AnnotatorContext type in swig" << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      cas_type = SWIG_TypeQueryModule(module,module, "CAS *");
      if (!cas_type) {
        cerr << MODULENAME ": could lookup cas type in swig" << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      rs_type = SWIG_TypeQueryModule(module,module, 
	"ResultSpecification *");
      if (!rs_type) {
        cerr << MODULENAME ": could lookup rs type in swig" << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
    }

    dSP ;

    ENTER ;
    SAVETMPS ;

    PUSHMARK(SP) ;
    XPUSHs(SWIG_Perl_NewPointerObj(
       reinterpret_cast<void *>( const_cast<AnnotatorContext *>(&ac)),
       ac_type, 0));

    PUTBACK ;
    call_pv("initialize", G_DISCARD);

    FREETMPS ;
    LEAVE ;

    if (SvTRUE(ERRSV)) {
      cerr << MODULENAME  " initialize error " 
           << SvPV_nolen(ERRSV) << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId reconfigure() {
    if (my_perl == 0) {
      cerr << MODULENAME ": not initialized in reconfigure" << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }
    if (debug > 1) {
      cerr<< MODULENAME ": reconfigure" <<endl;
    }

    dSP ;
    ENTER ;
    SAVETMPS ;

    PUSHMARK(SP) ;
    PUTBACK ;
    call_pv("reconfigure", G_DISCARD);

    FREETMPS ;
    LEAVE ;

    if (SvTRUE(ERRSV)) {
      cerr << MODULENAME " reconfigure error " 
           << SvPV_nolen(ERRSV) << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    return UIMA_ERR_NONE;
  }
  
  TyErrorId batchProcessComplete() {
    if (my_perl == 0) {
      cerr << MODULENAME 
           ": not initialized in batchProcessComplete" << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }
    if (debug > 1) {
      cerr<< MODULENAME ": batchProcessComplete" <<endl;
    }

    dSP ;
    ENTER ;
    SAVETMPS ;

    PUSHMARK(SP) ;
    PUTBACK ;
    call_pv("batchProcessComplete", G_DISCARD);

    FREETMPS ;
    LEAVE ;

    if (SvTRUE(ERRSV)) {
      cerr << MODULENAME " batchProcessComplete error "
	   << SvPV_nolen(ERRSV) << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId collectionProcessComplete() {
    if (my_perl == 0) {
      cerr << MODULENAME 
           ": not initialized in collectionProcessComplete" << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }
    if (debug > 1) {
      cerr<< MODULENAME ": collectionProcessComplete" <<endl;
    }

    dSP ;
    ENTER ;
    SAVETMPS ;

    PUSHMARK(SP) ;
    PUTBACK ;
    call_pv("collectionProcessComplete", G_DISCARD);

    FREETMPS ;
    LEAVE ;

    if (SvTRUE(ERRSV)) {
      cerr << MODULENAME " collectionProcessComplete error "
	   << SvPV_nolen(ERRSV) << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId typeSystemInit(TypeSystem const &ts) {
    if (my_perl == 0) {
      cerr << MODULENAME ": not initialized in typeSystemInit" << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }
    if (debug > 1) {
      cerr<< MODULENAME ": typeSystemInit" <<endl;
    }

    dSP ;
    ENTER ;
    SAVETMPS ;

    PUSHMARK(SP) ;
    XPUSHs(SWIG_Perl_NewPointerObj(
       reinterpret_cast<void *>( const_cast<TypeSystem *>(&ts)),
       ts_type, 0));

    PUTBACK ;
    call_pv("typeSystemInit", G_DISCARD);

    FREETMPS ;
    LEAVE ;

    if (SvTRUE(ERRSV)) {
      cerr << MODULENAME  " typeSystemInit error " << SvPV_nolen(ERRSV)
           << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    return UIMA_ERR_NONE;
  }

/** 
 * call the UIMA Annotator to deinitialize itself based on a UIMA engine
 * and return a UIMA error code
 */
  TyErrorId destroy()
  { 
    if (debug > 1) {
      cerr<< MODULENAME ": destroy " << endl;
    }

    dSP ;
    ENTER ;
    SAVETMPS ;

    PUSHMARK(SP) ;
    PUTBACK ;
    call_pv("destroy", G_DISCARD);

    FREETMPS ;
    LEAVE ;

    if (SvTRUE(ERRSV)) {
      cerr << MODULENAME  " destory error (ignored) " 
           << SvPV_nolen(ERRSV) << endl;
    }

    PL_perl_destruct_level = 0;
    perl_destruct(my_perl);
    perl_free(my_perl);
    my_perl = 0;
    return (TyErrorId)UIMA_ERR_NONE;
  }

/**
 * call the UIMA Annotator to perform its duty based on a UIMA engine
 * and return a UIMA error code
 */
  TyErrorId process(CAS &_cas, ResultSpecification const & _rs) { 
    if (debug > 1) {
      cerr<< MODULENAME ": process " << endl;
    }

    TyErrorId rc = UIMA_ERR_NONE;
 
    dSP ;
    ENTER ;
    SAVETMPS ;

    PUSHMARK(SP) ;
    XPUSHs(
       SWIG_Perl_NewPointerObj(
       reinterpret_cast<void *>( &_cas), cas_type, 0));
    XPUSHs(
       SWIG_Perl_NewPointerObj(
       reinterpret_cast<void *>( 
		const_cast<ResultSpecification *>(&_rs)),
       rs_type, 0));

    PUTBACK ;
    call_pv("process", G_DISCARD);

    FREETMPS ;
    LEAVE ;

    if (SvTRUE(ERRSV)) {
      cerr << MODULENAME " process error " 
	   << SvPV_nolen(ERRSV) << endl;
      rc = UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }

    return rc;
  }

};


#ifdef THREAD_PROTECTION
MAKE_AE(ThreadAnnotator<Perltator>);
#else
MAKE_AE(Perltator);
#endif


/* <EOF> */

