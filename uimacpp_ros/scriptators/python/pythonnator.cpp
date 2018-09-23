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

#include <Python.h>
#include "uima/api.hpp"

// for u_init
#include "unicode/uclean.h"

#define MODULENAME "Pythonnator"

#ifdef LINUX
// Beacuse of UIMA not using the RTLD_GLOBAL flag when binding to this
// .so file, we do not get the exports from the dependent libraries in 
// when loading.  This flag causes the python .so file to be re-bound
// hopefully this can be removed pending a future relesase of UIMA
#define REBIND_PYTHON_SO
#include <dlfcn.h>
#endif

#if defined(PYTHON2_3) || defined(PYTHON2_4)
#define _PY_BEGIN_BLOCK_THREADS_ \
  PyGILState_STATE __state = PyGILState_Ensure();
#define _PY_END_BLOCK_THREADS_ \
  PyGILState_Release(__state);
#else
#define _PY_BEGIN_BLOCK_THREADS_
#define _PY_END_BLOCK_THREADS_ 
#endif

using namespace uima;
using namespace std;

// copied from SwigGenerated code

// this function requires the SWIG code to be compiled with
// SWIGRUNTIME defined to be "", otherwise this function is static
// and does not scope outside of the library
#if !defined(SWIG_GLOBAL) && !defined(SWIGRUNTIME)
// SWIG 1.3.29 or better
#include "uimapywrap.h" 
#endif

const unsigned int FUNCTIONCOUNT=7;
enum { FUNCTION_INITIALIZE, FUNCTION_TYPESYSTEMINIT,
  FUNCTION_DESTROY, FUNCTION_RECONFIGURE, FUNCTION_PROCESS,
  FUNCTION_BATCHPROCESSCOMPLETE, FUNCTION_COLLECTIONPROCESSCOMPLETE };
static const char *function_name[FUNCTIONCOUNT] = {
  "initialize", "typeSystemInit", "destroy", "reconfigure", "process",
  "batchProcessComplete", "collectionProcessComplete"
};

class Pythonnator : public Annotator {
  PyThreadState *thread_state;
  PyObject *function[FUNCTIONCOUNT];
  int debug;
  swig_type_info *cas_type, *rs_type, *ts_type;
#ifdef REBIND_PYTHON_SO
      static void *library;
#endif
  static unsigned int refcount;

public:
  // We construct a perl interpreter in initialize - it lives for the
  // life of the annotator - even if reconfigure happens.  Reconfigure
  // and intialize both set dirty so the source code in the source file
  // and contained in the type system are evaluated.

  TyErrorId initialize(AnnotatorContext &ac) {
    PyObject *main_module, *user_module;
    PyObject *dict;
    swig_type_info *ac_type;
#ifdef PATH_MAX
    char srcfile[PATH_MAX + 256];
#else
    char srcfile[1000 + 256];
#endif

    for (unsigned int i=0; i<FUNCTIONCOUNT; i++) {
      function[i] = 0;
    } 

    debug = 0;
    if (ac.isParameterDefined("DebugLevel")) {
      ac.extractValue("DebugLevel", debug);
    }
    if (debug > 100) {
      cerr<< MODULENAME ": Initialize - debug=" << debug <<endl;
    }
    if (!ac.isParameterDefined("SourceFile")) {
      cerr<< MODULENAME ": Missing Python SourceFile" <<endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    UnicodeString fn;
    ac.extractValue(UnicodeString("SourceFile"), fn);
    if (fn == "") {
      cerr<< MODULENAME ": Empty Python SourceFile" <<endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

    UErrorCode err = U_ZERO_ERROR;
    fn.extract(srcfile,sizeof(srcfile),0,err);
    if (U_FAILURE(err)) {
      cerr << MODULENAME ": Unable to extract parameter, got " << u_errorName(err) << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

#ifdef REBIND_PYTHON_SO
      if (refcount == 0) 
        library = dlopen(PYTHONLIBRARYNAME , RTLD_LAZY | RTLD_GLOBAL);
      if (library == 0) {
        cerr<< MODULENAME ": unable to bind to python library " << 
          PYTHONLIBRARYNAME <<endl;
        cerr<< MODULENAME ": dlerror reports " << dlerror() << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      } 
#endif
      if (refcount++ == 0) {
        // This is done here because older versions of UIMA cannot be counted on to properly
	// initialize ICU, and this is necessary for thread safety
        UErrorCode status = U_ZERO_ERROR;
        u_init(&status);
        if (U_FAILURE(status)) {
          cerr<< MODULENAME ": ICU library reports failure to initialize" << endl;
          return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
        }

        Py_Initialize();
        PyEval_InitThreads();
	thread_state = PyThreadState_Get();
        PyEval_ReleaseLock(); // We will reaquire the lock in a few lines
        // after thread_state has been initialized
      }

      PyEval_AcquireLock();
      thread_state = Py_NewInterpreter();
      if (thread_state == 0) {
        cerr<< MODULENAME ": unable to create interpreter" << endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      PyEval_ReleaseLock();

      _PY_BEGIN_BLOCK_THREADS_
      main_module = PyImport_AddModule("__main__");
      if (PyImport_ImportModule("pythonnator") == 0) { 
        cerr << MODULENAME ":" << fn <<  ": failed to import pythonnator module, PYTHONPATH problem? " <<endl;
        _PY_END_BLOCK_THREADS_
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }

      user_module = PyImport_ImportModule(srcfile);
      if (user_module == 0) {
        cerr << MODULENAME ": could not import python module " << srcfile <<endl;
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      dict = PyModule_GetDict(user_module);

      for (unsigned int i=0; i < FUNCTIONCOUNT; i++) {
        function[i] = PyDict_GetItemString(dict, function_name[i] );
        if (function[i]) {
          Py_INCREF(function[i]);
          if (debug>100) {
            cerr << MODULENAME ": Registered function " << 
		function_name[i] << endl;
          }
        } else {
          if (debug>100) {
            cerr << MODULENAME ": no function registered for " << 
		function_name[i] << endl;
          }
        }
      }

      // convert cas and rs to python variables (parameters) 
      swig_module_info *module = SWIG_Python_GetModule();
      if (!module) {
        cerr << MODULENAME ": could not get Python swig module" << endl;
        _PY_END_BLOCK_THREADS_
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }

      ts_type = SWIG_TypeQueryModule(module,module, "_p_TypeSystem");
      if (!ts_type) {
        cerr << MODULENAME ": could lookup TypeSystem type in swig" << endl;
        _PY_END_BLOCK_THREADS_
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      ac_type = SWIG_TypeQueryModule(module,module, "_p_AnnotatorContext");
      if (!ac_type) {
        cerr << MODULENAME ": could lookup AnnotatorContext type in swig" << endl;
        _PY_END_BLOCK_THREADS_
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      cas_type = SWIG_TypeQueryModule(module,module, "_p_CAS");
      if (!cas_type) {
        cerr << MODULENAME ": could lookup cas type in swig" << endl;
        _PY_END_BLOCK_THREADS_
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
//       tcas_type = SWIG_TypeQueryModule(module,module, "_p_TCAS");
//       if (!tcas_type) {
//         cerr << MODULENAME ": could lookup tcas type in swig" << endl;
//         _PY_END_BLOCK_THREADS_
//         return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
//       }
      rs_type = SWIG_TypeQueryModule(module,module, 
	"_p_ResultSpecification");
      if (!rs_type) {
        cerr << MODULENAME ": could lookup rs type in swig" << endl;
        _PY_END_BLOCK_THREADS_
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }

      if (function[FUNCTION_INITIALIZE]) {
        PyObject *arg1 = 
           SWIG_Python_NewPointerObj(
           reinterpret_cast<void *>( &ac),
           ac_type, 0);
        PyObject *rv = PyObject_CallFunctionObjArgs( 
		function[FUNCTION_INITIALIZE], arg1, NULL);
        Py_DECREF(arg1);
        if (rv == 0) {
          cerr << MODULENAME ": python error " << endl;
          PyErr_Print();
          _PY_END_BLOCK_THREADS_
          return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
        } 
        Py_DECREF(rv);
      }

      _PY_END_BLOCK_THREADS_
      return UIMA_ERR_NONE;
  }

  TyErrorId reconfigure() {
    if (refcount==0) {
      cerr << MODULENAME ": not initialized in reconfigure" << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }
    _PY_BEGIN_BLOCK_THREADS_
    if (debug > 100) {
      cerr<< MODULENAME ": reconfigure" <<endl;
    }
    if (function[FUNCTION_RECONFIGURE]) {
      PyObject *rv = PyObject_CallFunctionObjArgs( 
        function[FUNCTION_RECONFIGURE], NULL);
      if (rv == 0) {
        cerr << MODULENAME ": python error " << endl;
        PyErr_Print();
        _PY_END_BLOCK_THREADS_
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      Py_DECREF(rv);
    }
    _PY_END_BLOCK_THREADS_
    return UIMA_ERR_NONE;
  }

  TyErrorId typeSystemInit(TypeSystem const &ts) {
    if (refcount==0) {
      cerr << MODULENAME ": not initialized in typeSystemInit" << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }
    _PY_BEGIN_BLOCK_THREADS_
    if (debug > 100) {
      cerr<< MODULENAME ": typeSystemInit" <<endl;
    }
    if (function[FUNCTION_TYPESYSTEMINIT] == 0) return UIMA_ERR_NONE;
    PyObject *arg1 = 
       SWIG_Python_NewPointerObj(
       reinterpret_cast<void *>( const_cast<TypeSystem *>(&ts)),
       ts_type, 0);
    PyObject *rv = PyObject_CallFunctionObjArgs( 
		function[FUNCTION_TYPESYSTEMINIT], arg1, NULL);
    Py_DECREF(arg1);

    if (rv == 0) {
      cerr << MODULENAME ": python error " << endl;
      PyErr_Print();
      _PY_END_BLOCK_THREADS_
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    } else {
      Py_DECREF(rv);
    }
    _PY_END_BLOCK_THREADS_
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
    _PY_BEGIN_BLOCK_THREADS_
    if (function[FUNCTION_DESTROY] != 0) {
      PyObject *rv = PyObject_CallFunctionObjArgs( 
        function[FUNCTION_DESTROY], NULL);
      if (rv == 0) {
        cerr << MODULENAME ": python error on destroy - ignoring" << endl;
        PyErr_Print();
      } else {
        Py_DECREF(rv);
      }
    }
    for (unsigned int i=0; i < FUNCTIONCOUNT; i++) {
      if (function[i]) Py_DECREF(function[i]);
      function[i] = 0;
    }

    if (--refcount == 0) {
      Py_Finalize();
#ifdef REBIND_PYTHON_SO
      dlclose(library); 
#endif
    } else {
      _PY_END_BLOCK_THREADS_
      // Py_EndInterpreter is crashing with multiple pythonnators running in parallel
      // Py_EndInterpreter(thread_state);
      thread_state = 0;
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
    if (function[FUNCTION_PROCESS]==0) {
      return UIMA_ERR_NONE;
    }

    _PY_BEGIN_BLOCK_THREADS_
    PyObject *arg1 = 
       SWIG_Python_NewPointerObj(
       reinterpret_cast<void *>( &_cas), cas_type, 0);
    if (!arg1) {
      cerr << "process: could not allocate Python object for cas" << endl;
      _PY_END_BLOCK_THREADS_
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }
    PyObject *arg2 = 
       SWIG_Python_NewPointerObj(
       reinterpret_cast<void *>( 
		const_cast<ResultSpecification *>(&_rs)),
       rs_type, 0);
    if (!arg2) {
      Py_DECREF(arg1);
      cerr << "process: could not allocate Python object for resultspec"<<endl;
      _PY_END_BLOCK_THREADS_
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    }

    PyObject *rv = PyObject_CallFunctionObjArgs( 
		function[FUNCTION_PROCESS], arg1, arg2, NULL);

    Py_DECREF(arg2);
    Py_DECREF(arg1);

    if (rv == 0) {
      cerr << MODULENAME ": python error " << endl;
      PyErr_Print();
      rc = UIMA_ERR_USER_ANNOTATOR_COULD_NOT_PROCESS;
    } else {
      Py_DECREF(rv);
    }

    _PY_END_BLOCK_THREADS_
    return rc;
  }

  TyErrorId batchProcessComplete()
  {
    if (debug > 100) {
      cerr<< MODULENAME ": batchProcessComplete " << endl;
    }
    _PY_BEGIN_BLOCK_THREADS_
    if (function[FUNCTION_BATCHPROCESSCOMPLETE]) {
      PyObject *rv = PyObject_CallFunctionObjArgs( 
        function[FUNCTION_BATCHPROCESSCOMPLETE], NULL);
      if (rv == 0) {
        cerr << MODULENAME ": python error " << endl;
        PyErr_Print();
        _PY_END_BLOCK_THREADS_
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      Py_DECREF(rv);
    }
    _PY_END_BLOCK_THREADS_
    return (TyErrorId)UIMA_ERR_NONE;
  }

  TyErrorId collectionProcessComplete()
  {
    if (debug > 100) {
      cerr<< MODULENAME ": collectionProcessComplete " << endl;
    }
    _PY_BEGIN_BLOCK_THREADS_
    if (function[FUNCTION_COLLECTIONPROCESSCOMPLETE]) {
      PyObject *rv = PyObject_CallFunctionObjArgs( 
        function[FUNCTION_COLLECTIONPROCESSCOMPLETE], NULL);
      if (rv == 0) {
        cerr << MODULENAME ": python error " << endl;
        PyErr_Print();
        _PY_END_BLOCK_THREADS_
        return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
      }
      Py_DECREF(rv);
    }
    _PY_END_BLOCK_THREADS_
    return (TyErrorId)UIMA_ERR_NONE;
  }

};

#ifdef REBIND_PYTHON_SO
void *Pythonnator::library = 0;
#endif
unsigned int Pythonnator::refcount = 0;

MAKE_AE(Pythonnator);


// /* ----------------------------------------------------------------------- */
// /*   Mapping for generic C API wrapper                                     */
// /* ----------------------------------------------------------------------- */

// typedef Pythonnator UserDefinedAnnotator;
// // define for error/exception info in uima_annotator_generic.inl
// #define UIMA_ANNOTATOR_NAME MODULENAME

// /* ----------------------------------------------------------------------- */
// /*   Include generic C API wrapper                                         */
// /* ----------------------------------------------------------------------- */

// // use casconsumer version so that both annotators and casconsumers are supported
// #include "uima_casconsumer_generic.inl"


/* <EOF> */


