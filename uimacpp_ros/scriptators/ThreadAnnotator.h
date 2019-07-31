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

#include "apr_thread_proc.h"
#include "apr_thread_mutex.h"
#include "apr_thread_cond.h"
template <class _Annotator>
class ThreadAnnotator : public uima::Annotator {
  _Annotator *child;
  apr_pool_t * threadPool;
  apr_thread_t * my_thread;
  apr_thread_cond_t *batton4P;
  apr_thread_cond_t *batton4C;
  apr_thread_mutex_t *mutex;

  /* all of these are not thread safe - parent must sleep when child runs
     and vice versa */
  uima::AnnotatorContext *_ac;
  uima::CAS *_cas;
  uima::ResultSpecification const *_rs;
  uima::TypeSystem const *_ts;
  uima::TyErrorId ret;
  bool threadReady;

  enum method {
    NOP, INITIALIZE, RECONFIGURE, DESTROY, TYPESYSTEMINIT, 
    PROCESS, DESTRUCTOR, BATCHPROCESSCOMPLETE, COLLECTIONPROCESSCOMPLETE
  };
  method signal;

  /* note that the parent object must be suspended when this wakes up */
  static void *APR_THREAD_FUNC threadfunc(apr_thread_t* this_thread, void *p) {
    ThreadAnnotator<_Annotator> *parent = 
	static_cast<ThreadAnnotator<_Annotator> *>(p); 
	  parent->threadReady = true;
	while (true) {
 	  apr_thread_cond_wait(parent->batton4C, parent->mutex);
	  apr_thread_mutex_unlock(parent->mutex);

	  switch (parent->signal) {
	  case INITIALIZE:
		parent->ret = 
		  parent->child->initialize(*parent->_ac);
		break;
	  case RECONFIGURE:
		parent->ret = 
		  parent->child->reconfigure();
		break;
	  case DESTROY:
		parent->ret = 
		  parent->child->destroy();
 		apr_thread_cond_signal(parent->batton4P);
 		apr_thread_exit(this_thread, APR_SUCCESS);
	  case BATCHPROCESSCOMPLETE:
		parent->ret = 
		  parent->child->batchProcessComplete();
		break;
	  case COLLECTIONPROCESSCOMPLETE:
		parent->ret = 
		  parent->child->collectionProcessComplete();
		break;
	  case TYPESYSTEMINIT:
		parent->ret = 
		  parent->child->typeSystemInit(*parent->_ts);
		break;
	  case PROCESS:
		parent->ret = 
		  parent->child->process(*parent->_cas, *parent->_rs); 
		break;
	  case NOP:
	  case DESTRUCTOR:
		// worker thread should never be here!
		break;
	  }
	  apr_thread_mutex_lock(parent->mutex);
 	  apr_thread_cond_signal(parent->batton4P);
	}
   	return p;
  } 

  void execThread() {
	apr_thread_mutex_lock(mutex);
     apr_thread_cond_signal(batton4C);
     apr_thread_cond_wait(batton4P, mutex);
	 apr_thread_mutex_unlock(mutex);
  }

public:
  ThreadAnnotator() : child( new _Annotator() ), signal(NOP) {
    apr_status_t rv = apr_initialize();
    rv = apr_initialize();
    rv += atexit(apr_terminate);
    rv += apr_pool_create(&threadPool, NULL);
    rv += apr_thread_mutex_create(&mutex, APR_THREAD_MUTEX_UNNESTED, threadPool);
    rv += apr_thread_cond_create(&batton4P, threadPool);
    rv += apr_thread_cond_create(&batton4C, threadPool);

 	threadReady = false;
 	rv = apr_thread_create(&my_thread, NULL, threadfunc, 
						   static_cast<void *>( this), threadPool);

	// wait for worker thread to be ready
    apr_sleep(1000);
	apr_thread_mutex_lock(mutex);
 	if (!threadReady) {
	  std::cerr << "WARNING: ThreadAnnotator worker thread not ready! " << std::endl;
 	}
	apr_thread_mutex_unlock(mutex);
  }
  ~ThreadAnnotator() {
    delete child;
  }
  uima::TyErrorId initialize(uima::AnnotatorContext &ac) {
    _ac = &ac;
    signal = INITIALIZE;
    execThread();
    return ret;
  }
  uima::TyErrorId typeSystemInit(uima::TypeSystem const &ts) {
    _ts = &ts;
    signal = TYPESYSTEMINIT;
    execThread();
    return ret;
  }
  uima::TyErrorId process(uima::CAS &cas, uima::ResultSpecification const &rs) {
    _cas = &cas;
    _rs = &rs;
    signal = PROCESS;
    execThread();
    return ret;
  }
  uima::TyErrorId reconfigure() {
    signal = RECONFIGURE;
    execThread();
    return ret;
  }
  uima::TyErrorId batchProcessComplete() {
    signal = BATCHPROCESSCOMPLETE;
    execThread();
    return ret;
  }
  uima::TyErrorId collectionProcessComplete() {
    signal = COLLECTIONPROCESSCOMPLETE;
    execThread();
    return ret;
  }
  uima::TyErrorId destroy() {
    signal = DESTROY;
    execThread();
    apr_status_t status;
	apr_thread_join(&status, my_thread);
	apr_thread_cond_destroy(batton4P);
	apr_thread_cond_destroy(batton4C);
	apr_thread_mutex_destroy(mutex);
    return ret;
  }
};

