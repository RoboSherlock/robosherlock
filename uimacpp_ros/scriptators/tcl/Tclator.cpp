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

#include <tcl.h>
#include "uima/api.hpp"

#define THREAD_PROTECTION
#ifdef THREAD_PROTECTION
#include "ThreadAnnotator.h"
#endif

using namespace uima;
using namespace std;

#define MODULENAME "Tclator"
      
extern "C" { int Tclator_Init(Tcl_Interp * interp); }

#ifdef LINUX
// Allow use of single threaded Tcl interpreter in multithreaded environments
#define MUTEX_DEFINE static pthread_mutex_t mutex;
#define MUTEX_INIT   pthread_mutex_init(&mutex,0);
#define MUTEX_LOCK   pthread_mutex_lock(&mutex);
#define MUTEX_UNLOCK pthread_mutex_unlock(&mutex);
#define MUTEX_ALLOCATE pthread_mutex_t Tclator::mutex = PTHREAD_MUTEX_INITIALIZER;
#else
// Not supported yet
#define MUTEX_DEFINE
#define MUTEX_INIT
#define MUTEX_LOCK
#define MUTEX_UNLOCK
#define MUTEX_ALLOCATE
#endif

// copied from SwigGenerated code

// this function requires the SWIG code to be compiled with
// SWIGRUNTIME defined to be "", otherwise this function is static
// and does not scope outside of the library
#if !defined(SWIG_GLOBAL) && !defined(SWIGRUNTIME)
// SWIG 1.3.29 or better
#include "uimatclwrap.h" 
#endif

const char *default_methods = 
 	"proc initialize {ac} {}\n"
	"proc typeSystemInit {ts} {}\n"
	"proc destroy {} {}\n"
	"proc process {cas rs} {}\n"
	"proc reconfigure {} {}\n"
	"proc batchProcessComplete {} {}\n"
	"proc collectionProcessComplete {} {}\n";

class Tclator : public Annotator {
  int debug;
  MUTEX_DEFINE 
  swig_type_info *cas_type, *rs_type, *ts_type;
  Tcl_Interp *interp;
  Tcl_Obj *commands[7];
  enum CMDS { INITIALIZE, TYPESYSTEMINIT, DESTROY, PROCESS, RECONFIGURE, BATCHPROCESSCOMPLETE, COLLECTIONPROCESSCOMPLETE };

public:

  Tclator() : interp(0) {}

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
    if (debug > 0) {
      cerr<< MODULENAME ": Initialize - debug=" << debug <<endl;
    }
    if (!ac.isParameterDefined("SourceFile")) {
      cerr<< MODULENAME ": Missing Tcl SourceFile" <<endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    UnicodeString fn;
    ac.extractValue(UnicodeString("SourceFile"), fn);
    if (fn == "") {
      cerr<< MODULENAME ": Empty Tcl SourceFile" <<endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    //    cerr << fn << endl; 

    UErrorCode err = U_ZERO_ERROR;
    fn.extract(srcfile,sizeof(srcfile),0,err);
    if (U_FAILURE(err)) {
      cerr << MODULENAME ": Unable to extract parameter, got " << u_errorName(err) << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    if (interp == 0) {
      // this seems to make windows work
      // would be nice to know argv[0] somehow
      Tcl_FindExecutable( 0 ); 

      interp = Tcl_CreateInterp();
      if (interp == 0) {
        cerr << MODULENAME ": failed to create interpreter" << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }

      int tclret = Tcl_Init(interp);
      if (tclret != TCL_OK) {
        cerr << MODULENAME ": failed to init interpreter - " <<
		Tcl_GetStringResult(interp) << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }

      if (Tclator_Init(interp) != TCL_OK) {
        cerr << MODULENAME ": failed to init tclator package" << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }

      commands[INITIALIZE] = Tcl_NewStringObj("initialize", -1);
      commands[TYPESYSTEMINIT] = Tcl_NewStringObj("typeSystemInit", -1);
      commands[RECONFIGURE] = Tcl_NewStringObj("reconfigure", -1);
      commands[PROCESS] = Tcl_NewStringObj("process", -1);
      commands[DESTROY] = Tcl_NewStringObj("destroy", -1);
      commands[BATCHPROCESSCOMPLETE] = Tcl_NewStringObj("batchProcessComplete", -1);
      commands[COLLECTIONPROCESSCOMPLETE] = Tcl_NewStringObj("collectionProcessComplete", -1);

      for (unsigned int i=0; i<7; ++i) Tcl_IncrRefCount(commands[i]);

      if (Tcl_Eval(interp, default_methods) != TCL_OK) {
        cerr << MODULENAME << ": Error - " << 
		Tcl_GetStringResult(interp) << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }

      if (Tcl_EvalFile(interp, srcfile) != TCL_OK) {
        cerr << MODULENAME << ": Error - " << 
		Tcl_GetStringResult(interp) << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }

      // this is a static variable because of the way that SWIG's
      // initialization code is written.      
      static swig_module_info *module = 0;
      if (module == 0) {
        MUTEX_INIT
        MUTEX_LOCK
        module = SWIG_Tcl_GetModule(interp);
      } else {
        MUTEX_LOCK
        SWIG_Tcl_SetModule(interp, module);
      }

      if (!module) {
        cerr << MODULENAME ": could not get Tcl swig module" << endl;
        MUTEX_UNLOCK
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }

      ts_type = SWIG_TypeQueryModule(module,module, "TypeSystem *");
      if (!ts_type) {
        cerr << MODULENAME ": could lookup TypeSystem type in swig" << endl;
        MUTEX_UNLOCK
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      ac_type = SWIG_TypeQueryModule(module,module, "AnnotatorContext *");
      if (!ac_type) {
        cerr << MODULENAME ": could lookup AnnotatorContext type in swig" << endl;
        MUTEX_UNLOCK
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      } 
      cas_type = SWIG_TypeQueryModule(module,module, "CAS *");
      if (!cas_type) {
        cerr << MODULENAME ": could lookup cas type in swig" << endl;
        MUTEX_UNLOCK
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      rs_type = SWIG_TypeQueryModule(module,module, 
	"ResultSpecification *");
      if (!rs_type) {
        cerr << MODULENAME ": could lookup rs type in swig" << endl;
        MUTEX_UNLOCK
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
    }

    Tcl_Obj *args[2];
    args[0] = commands[INITIALIZE];
    args[1] = SWIG_Tcl_NewPointerObj(
       reinterpret_cast<void *>( const_cast<AnnotatorContext *>(&ac)),
       ac_type, 0);
    Tcl_IncrRefCount(args[0]);
    Tcl_IncrRefCount(args[1]);
    int rc = Tcl_EvalObjv(interp, 2, args, 0);
    Tcl_DecrRefCount(args[0]);
    Tcl_DecrRefCount(args[1]);
    if (rc != TCL_OK) {
      cerr << MODULENAME  " initialize error " << 
	Tcl_GetStringResult(interp) << endl;
      MUTEX_UNLOCK
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }
    MUTEX_UNLOCK
    return UIMA_ERR_NONE;
  }

  TyErrorId reconfigure() {
    if (interp == 0) {
      cerr << MODULENAME ": not initialized in reconfigure" << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }
    if (debug > 100) {
      cerr<< MODULENAME ": reconfigure" <<endl;
    }

    MUTEX_LOCK
    Tcl_Obj *args[1];
    args[0] = commands[RECONFIGURE];
    Tcl_IncrRefCount(args[0]);
    int rc = Tcl_EvalObjv(interp, 1, args, 0);
    Tcl_DecrRefCount(args[0]);
    if ( rc != TCL_OK) {
      cerr << MODULENAME  " reconfigure error " << 
	Tcl_GetStringResult(interp) << endl;
      MUTEX_UNLOCK
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }
    MUTEX_UNLOCK
    return UIMA_ERR_NONE;
  }

  TyErrorId typeSystemInit(TypeSystem const &ts) {
    if (interp == 0) {
      cerr << MODULENAME ": not initialized in typeSystemInit" << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }
    if (debug > 100) {
      cerr<< MODULENAME ": typeSystemInit" <<endl;
    }

    MUTEX_LOCK
    Tcl_Obj *args[2];
    args[0] = commands[TYPESYSTEMINIT];
    args[1] = 
    	SWIG_Tcl_NewPointerObj(
	  reinterpret_cast<void *>( const_cast<TypeSystem *>(&ts)),
	       ts_type, 0);
    Tcl_IncrRefCount(args[0]);
    Tcl_IncrRefCount(args[1]);
    int rc = Tcl_EvalObjv(interp, 2, args, 0);
    Tcl_DecrRefCount(args[1]);
    Tcl_DecrRefCount(args[0]);
    if (rc != TCL_OK) {
      cerr << MODULENAME  " typeSystemInit error " << 
	Tcl_GetStringResult(interp) << endl;
      MUTEX_UNLOCK
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }
    MUTEX_UNLOCK
    return UIMA_ERR_NONE;
  }

/** 
 * call the UIMA Annotator to deinitialize itself based on a UIMA engine
 * and return a UIMA error code
 */
  TyErrorId destroy()
  { 
    if (debug > 100) {
      cerr<< MODULENAME ": destroy " << endl;
    }
    if (interp != 0) {
      MUTEX_LOCK
      Tcl_Obj *args[1];
      args[0] = commands[DESTROY];
      Tcl_IncrRefCount(args[0]);
      if (Tcl_EvalObjv(interp, 1, args, 0) != TCL_OK) {
        cerr << MODULENAME  " destroy error (ignored) " << 
 	  Tcl_GetStringResult(interp) << endl;
      }
      Tcl_DecrRefCount(args[0]);

      for (unsigned int i=0; i<7; ++i) {
        if (commands[i]) Tcl_DecrRefCount(commands[i]);
        commands[i] = 0;
      }
      Tcl_DeleteInterp(interp);
      interp = 0;
      MUTEX_UNLOCK
    }
    return (TyErrorId)UIMA_ERR_NONE;
  }

/**
 * call the UIMA Annotator to perform its duty based on a UIMA engine
 * and return a UIMA error code
 */
  TyErrorId process(CAS &_cas, ResultSpecification const & _rs) { 
    if (debug > 100) {
      cerr<< MODULENAME ": process " << endl;
    }

    TyErrorId rc = UIMA_ERR_NONE;

    MUTEX_LOCK
    Tcl_Obj *args[3];
    args[0] = commands[PROCESS];
    args[1] = 
       SWIG_Tcl_NewPointerObj(
       reinterpret_cast<void *>( &_cas), cas_type, 0);
    args[2] = 
       SWIG_Tcl_NewPointerObj(
       reinterpret_cast<void *>( 
		const_cast<ResultSpecification *>(&_rs)),
       rs_type, 0);
    Tcl_IncrRefCount(args[0]);
    Tcl_IncrRefCount(args[1]);
    Tcl_IncrRefCount(args[2]);
    int trc = Tcl_EvalObjv(interp, 3, args, 0);
    Tcl_DecrRefCount(args[2]);
    Tcl_DecrRefCount(args[1]);
    Tcl_DecrRefCount(args[0]);

    if (trc != TCL_OK) {
      cerr << MODULENAME " process error (ignored) " << 
	Tcl_GetStringResult(interp) << endl;
      rc = UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }
    MUTEX_UNLOCK
    return rc;
  }

/** 
 * call the UIMA Annotator batchProcessComplete method
 * and return a UIMA error code
 */
  TyErrorId batchProcessComplete()
  { 
    if (debug > 100) {
      cerr<< MODULENAME ": batchProcessComplete " << endl;
    }

    TyErrorId rc = UIMA_ERR_NONE;

    MUTEX_LOCK
    Tcl_Obj *args[1];
    args[0] = commands[BATCHPROCESSCOMPLETE];
    Tcl_IncrRefCount(args[0]);
    int trc = Tcl_EvalObjv(interp, 1, args, 0);
    Tcl_DecrRefCount(args[0]);

    if (trc != TCL_OK) {
      cerr << MODULENAME  " batchProcessComplete error (ignored) " << 
	Tcl_GetStringResult(interp) << endl;
      rc = UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }
    MUTEX_UNLOCK
    return rc;
  }

/** 
 * call the UIMA Annotator collectionProcessComplete method
 * and return a UIMA error code
 */
  TyErrorId collectionProcessComplete()
  { 
    if (debug > 100) {
      cerr<< MODULENAME ": collectionProcessComplete " << endl;
    }

    TyErrorId rc = UIMA_ERR_NONE;
    MUTEX_LOCK

    Tcl_Obj *args[1];
    args[0] = commands[COLLECTIONPROCESSCOMPLETE];
    Tcl_IncrRefCount(args[0]);
    int trc = Tcl_EvalObjv(interp, 1, args, 0);
    Tcl_DecrRefCount(args[0]);

    if (trc != TCL_OK) {
      cerr << MODULENAME  " collectionProcessComplete error (ignored) " << 
	Tcl_GetStringResult(interp) << endl;
      rc = UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }
    MUTEX_UNLOCK
    return rc;
  }

};

MUTEX_ALLOCATE

#ifdef THREAD_PROTECTION
MAKE_AE(ThreadAnnotator<Tclator>);
#else
MAKE_AE(Tclator);
#endif


/* <EOF> */

