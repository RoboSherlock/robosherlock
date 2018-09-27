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

* Analysis Engine service wrapper implementation based on 
* Active MQ C++ client.
*/

#include "ActiveMQAnalysisEngineService.hpp"
#include "deployCppService.hpp"
#include <activemq/core/ActiveMQConsumer.h>
#include <activemq/library/ActiveMQCPP.h>
using namespace std;
using namespace activemq::exceptions;

int shutdown_service;
//===================================================
//Main
//---------------------------------------------------

void printUsage() {
  cout << "Usage: deployCppService aeDescriptor queueName " << endl <<
    "       <-b brokerURL [defaults to tcp://localhost:61616]> " << endl <<   
    "       <-n numInstance [defaults to 1]> " << endl << 
    "       <-p prefetchSize [defaults to 1]> " << endl <<
    "       <-l loglevel [defaults to 0 INFO]> " << endl <<
    "       <-t tracelevel [defaults to 0 and valid to 4]> " << endl << 
    "       <-d datapath [defaults to not set] " << endl ;
  cout << "Example: deployCppService DaveDetector.xml DAVEDETECTORQ -b tcp://localhost:61616 -n 1 -p 1 -l 0 -t 0 -d c:/mydata" << endl;        
}

int main(int argc, char* argv[]) {
 // cout << "=====================================================\n";    
  cout << __FILE__ << " Starting the UIMA C++ Remote Service using ActiveMQ broker." << std::endl;
 // cout << "-----------------------------------------------------\n";
  // _CrtSetBreakAlloc(1988);
  activemq::library::ActiveMQCPP::initializeLibrary();
  std::cout << "Running with ActiveMQ CPP failover enabled. " << std::endl;
  try {
    if (argc < 3) {
      printUsage();
      return(-1);
    }

    /* create ResourceManager */
    if (!uima::ResourceManager::hasInstance()) {
      uima::ResourceManager::createInstance("ActiveMQAnalysisEngineService");
    }

    /* APR pool */
    apr_status_t rv;
    apr_pool_t *pool;
    rv = apr_pool_create(&pool, NULL);
    if (rv != APR_SUCCESS) {
      cerr << __FILE__ << " ERROR: apr_pool_create() failed. " << endl;
      return -1;
    }


    /*parse descriptor */
    ServiceParameters serviceDesc;
    serviceDesc.parseArgs(argc, argv);
    serviceDesc.print();

    /*set up logging and monitoring */
    initialize(serviceDesc, pool);   

    /*create service*/
    AMQAnalysisEngineService aeService(serviceDesc,singleton_pMonitor, pool);
    aeService.setTraceLevel(serviceDesc.getTraceLevel());

    /* create processing threads */
    aeService.startProcessingThreads();

    /*start receiving messages*/ 
    cerr << __FILE__ << __LINE__  << " Start receiving messages " << endl;
    aeService.start();

    //check if java process still there
   

    cerr << __FILE__ << " UIMA C++ Service " << serviceDesc.getQueueName() << " at " <<
    serviceDesc.getBrokerURL() << " Ready to process..." << endl;
    
    /* connect to java proxy if called from java */  
    apr_thread_t *thread=0;
    apr_threadattr_t *thd_attr=0;

    if (cs) {
      apr_threadattr_create(&thd_attr, pool);
      rv = apr_thread_create(&thread, thd_attr, handleCommands, 0, pool);
      assert(rv == APR_SUCCESS);
      //rv = apr_thread_join(&rv, thread);
      //assert(rv == APR_SUCCESS);
      apr_thread_mutex_lock(singleton_pMonitor->cond_mutex);
      apr_thread_cond_wait(singleton_pMonitor->cond, singleton_pMonitor->cond_mutex);
      apr_thread_mutex_unlock(singleton_pMonitor->cond_mutex);

    }  else {
      uima::ResourceManager::getInstance().getLogger().logError(" not from java running ");
      apr_thread_t *stdinthread=0;
      rv = apr_thread_create(&stdinthread, thd_attr, readstdin, 0, pool);
      rv = apr_thread_join(&rv, stdinthread);
    }  

    //wait
/**    if (cs) { 
      apr_thread_mutex_lock(singleton_pMonitor->cond_mutex);
      apr_thread_cond_wait(singleton_pMonitor->cond, singleton_pMonitor->cond_mutex);
      apr_thread_mutex_unlock(singleton_pMonitor->cond_mutex);    
    }
**/
    if (singleton_pMonitor->getQuiesceAndStop()) {
      cerr << __FILE__ << " " << serviceDesc.getServiceName() << " Quiesce started. " << endl;
      //quiesce 
      aeService.quiesceAndStop();
      cerr << __FILE__ << " " << serviceDesc.getServiceName() << " quiesced. " << endl;
    } else {
      cerr << __FILE__ << " Shutdown started. " << endl;
      aeService.shutdown();
      cerr << __FILE__ << " Shutdown done. " << endl;
    }

    //notify java controller
    if (cs) {
      apr_size_t len = 4;
      apr_status_t rv = apr_socket_send(cs, "DONE" , &len);
      len = 1;
      apr_socket_send(cs,"\n", &len);
      if (rv != APR_SUCCESS) {
        cerr << " apr_socket_send() failed sending shutdown notification." << endl;
      }
    }  

    /* cleanup */
    terminateService();
    
    if (pool) {
      apr_pool_destroy(pool);
      pool=0;
    }
  }  catch (CMSException& e) {
    stringstream str; 
    str << e.getMessage() << endl;
    cerr << __FILE__ << __LINE__ << " " << str.str() << endl;
    /***
    if (cs) {
      apr_size_t len = str.str().length();
      rv = apr_socket_send(cs, str.str().c_str(), &len);
      len = 1;
      apr_socket_send(cs,"\n", &len);
    }
    ***/
  } catch (XMLException& e) {
    stringstream str; 
    str << e.getMessage() << endl;
    cerr << __FILE__ << __LINE__ << " " << str.str() << endl;
    /***
    if (cs) {
      apr_size_t len = str.str().length();
      rv = apr_socket_send(cs, str.str().c_str(), &len);
      len = 1;
      apr_socket_send(cs,"\n", &len);
    }
    ***/
  } catch (uima::Exception e) {
    stringstream str;
    str << e.getErrorInfo().getMessage().asString() << endl;
    cerr << __FILE__ << __LINE__ << " " << str.str() << endl;
    /***
    if (cs) {
      apr_size_t len = str.str().length();
      rv = apr_socket_send(cs, str.str().c_str(), &len);
      len = 1;
      apr_socket_send(cs,"\n", &len);
    }
    ***/
  } catch (...) {
    stringstream str;
    str <<  "Unknown Exception" << endl;
    cerr << __FILE__ << __LINE__ << " " << str.str() << endl;
    /***
    if (cs) {
      apr_size_t len = str.str().length();
      rv = apr_socket_send(cs, str.str().c_str(), &len);
      len = 1;
      apr_socket_send(cs,"\n", &len);
    }
    ***/
  }
  //cout << "-----------------------------------------------------\n";    
  cout << __FILE__ " UIMA C++ Remote Service terminated:\n";
  //cout << "=====================================================\n";    

  activemq::library::ActiveMQCPP::shutdownLibrary();
   exit(0);

}  //main






