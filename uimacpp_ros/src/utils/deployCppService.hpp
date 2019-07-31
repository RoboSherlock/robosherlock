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

* This file contains code common to the ActiveMQ and WebSphere MQ
* implementations of the UIMA C++ service wrapper.
*/

#ifndef __DEPLOY_SERVICE__
#define __DEPLOY_SERVICE__
#include <uima/api.hpp>
#include <apr_network_io.h>
#include <apr_proc_mutex.h>
#include <apr_errno.h>
#include <apr_general.h>
#include <apr_thread_proc.h>
#include <apr_thread_cond.h>
#include <apr_signal.h>
#include <apr_portable.h>
#include <string>
using namespace std;
class SocketLogger;
class Monitor;
class AMQAnalysisEngineService;
class ServiceParameters;


/*
* Constants
*/
#define PROCESS_CAS_COMMAND 2000
#define GET_META_COMMAND    2001
#define CPC_COMMAND         2002
#define REQUEST             3000
#define RESPONSE            3001
#define XMI_PAYLOAD         1000
#define XCAS_PAYLOAD        1004
#define META_PAYLOAD        1002
#define EXC_PAYLOAD         1003
#define NO_PAYLOAD          1005

static char * getmeta_selector = "Command = 2001";
static char * annotator_selector = "Command = 2000 or Command = 2002";

static int initialize(ServiceParameters &, apr_pool_t*);
static void* APR_THREAD_FUNC handleCommands(apr_thread_t *thd, void *data);
static int terminateService();
static void signal_handler(int signum);
static void* APR_THREAD_FUNC readstdin(apr_thread_t *thd, void *data);



/**
* This class holds command line parameters that 
* configure this service.
*/

class ServiceParameters {
  public:
  
    ServiceParameters() :
      iv_aeDescriptor(), iv_brokerURL("tcp://localhost:61616"),
        iv_queueName(), iv_numInstances(1),
        iv_prefetchSize(1), iv_javaport(0), 
        iv_datapath(), iv_loglevel(0), iv_tracelevel(-1),
        iv_errThreshhold(0), iv_errWindow(0), iv_terminateOnCPCError(false),
        iv_user(), iv_password(), iv_initialfsheapsize(0) {}

    ~ServiceParameters() {}

    void print() {
        cout << asString() << endl;
    }	

    string asString() {
      stringstream str;
      str << "AE descriptor " <<  iv_aeDescriptor 
        << " Initial FSHeap size " << this->iv_initialfsheapsize
        << " Input Queue " << iv_queueName
        << " Num Instances " << iv_numInstances 
        << " Prefetch size " << iv_prefetchSize 
        << " ActiveMQ broker URL " << iv_brokerURL 
        << " Java port " << iv_javaport 
        << " logging level " << iv_loglevel 
        << " trace level " << iv_tracelevel 
        << " datapath " << iv_datapath << endl;
      return str.str();
    }

    const string & getBrokerURL() const {
      return iv_brokerURL;
    }

    const string & getQueueName() const {
      return iv_queueName;
    }

    const string & getAEDescriptor() const {
      return iv_aeDescriptor;
    }

    const int getTraceLevel() const {
      return iv_tracelevel;
    }
    const int getLoggingLevel() const {
      return iv_loglevel;
    }
    const int getNumberOfInstances() const {
      return iv_numInstances;
    }
    const int getPrefetchSize() const {
      return iv_prefetchSize;
    }
    const int getJavaPort() const {
      return iv_javaport;
    }

    const string & getDataPath() {
      return iv_datapath;
    }

    const int getErrorThreshhold() {
      return iv_errThreshhold;
    }

    const int getErrorWindow() {
      return iv_errWindow;
    }
    const bool terminatOnCPCError() {
      return iv_terminateOnCPCError;
    }

    const string & getUserName() {
      return iv_user;
    }

    const string & getPassword() {
      return iv_password;
    }

    const size_t getInitialFSHeapSize() {
      return iv_initialfsheapsize;
    }

    void setBrokerURL(const string url) {
      iv_brokerURL = url;
    }

    void setQueueName(const string qname) {
      iv_queueName = qname;
    }

    void setAEDescriptor(const string loc) {
      iv_aeDescriptor = loc;
    }

    void setNumberOfInstances(int num) {
      iv_numInstances=num;
    }

    void setTraceLevel(int num) {
      iv_tracelevel=num;
    }

    void setLoggingLevel(int num) {
      iv_loglevel=num;
    }
    void setJavaPort(int val) {
      iv_javaport = val;
    }

    void setDataPath(const string path) {
      iv_datapath=path;
    }

    void parseArgs(int argc, char* argv[]) {
      int index =0;
      while (++index < argc) {
        char * arg = argv[index];
        if ( 0 == strcmp(arg, "-b") ) {
          if (++index < argc) {
            this->iv_brokerURL = argv[index];
          }
        } else if (0 == strcmp(arg, "-n")) {
          if (++index < argc) {
            this->iv_numInstances = atoi(argv[index]);
          }
        } else if (0 == strcmp(arg, "-fsheapsz")) {
          if (++index < argc) {
            this->iv_initialfsheapsize = atoi(argv[index]);
            this->iv_initialfsheapsize = this->iv_initialfsheapsize/4;
          }
        } else if (0 == strcmp(arg, "-p")) {
          if (++index < argc) {
            this->iv_prefetchSize = atoi(argv[index]);
          }
        } else if (0 == strcmp(arg, "-t")) {
          if (++index < argc) {
            this->iv_tracelevel = atoi(argv[index]);
          }
        } else if (0 == strcmp(arg, "-l")) {
          if (++index < argc) {
            this->iv_loglevel = atoi(argv[index]);
          }
        } else if (0 == strcmp(arg, "-jport")) {
          if (++index < argc) {
            this->iv_javaport = atoi(argv[index]);
          }
        } else if (0 == strcmp(arg, "-d")) {
          if (++index < argc) {
            this->iv_datapath = argv[index];
          }
        } else if (0 == strcmp(arg, "-e")) {
          if (++index < argc) {
            this->iv_errThreshhold = atoi(argv[index]);
          }
        }  else if (0 == strcmp(arg, "-w")) {
          if (++index < argc) {
            this->iv_errWindow = atoi(argv[index]);
          }
        } else if (0 == strcmp(arg, "-user")) {
          if (++index < argc) {
            this->iv_user = argv[index];
          }
        } else if (0 == strcmp(arg, "-pw")) {
          if (++index < argc) {
            this->iv_password = argv[index];
          }
        } else if (0 == strcmp(arg, "-a")) {
          if (++index < argc) {
			if (strcasecmp(argv[index],"true")==0) {
              this->iv_terminateOnCPCError = true;
            } else {
              this->iv_terminateOnCPCError=false;
            }
          }
        } else {
          if (this->iv_aeDescriptor.length() == 0) {
            this->iv_aeDescriptor = argv[index];
          } else {
            if (this->iv_queueName.length() == 0) {
              this->iv_queueName = argv[index];
            } else {
              cerr << "unexpected argument " << argv[index] << endl;
            }
          }
        }
      }
    }

  string getServiceName() {
    return getQueueName() + "@" + getBrokerURL();  
  }

  private:
    string iv_aeDescriptor;
    string iv_brokerURL;
    string iv_queueName;
    int iv_prefetchSize;
    int iv_numInstances;
    int iv_javaport;
    string iv_datapath;
    int iv_loglevel;
    int iv_tracelevel;
    int iv_errThreshhold;
    int iv_errWindow;
    bool iv_terminateOnCPCError;
    string iv_user;
    string iv_password;
    size_t iv_initialfsheapsize;
};

/* SocketLogger */
class SocketLogger : public uima::Logger {
  private:
    apr_socket_t * socket;
    apr_thread_mutex_t *mutex;

  public:
    SocketLogger(apr_socket_t * sock, apr_pool_t * pool) : socket(sock) {
      apr_thread_mutex_create(&mutex,APR_THREAD_MUTEX_UNNESTED, pool);
    }

    virtual void log(uima::LogStream::EnEntryType entrytype,
      string classname,
      string methodname,
      string message,
      long lUserCode) {
      //cout << "SocketLogger::log() " << message << endl;
      apr_thread_mutex_lock(mutex);

      apr_status_t rv;
      stringstream str;
      str << entrytype << " " << classname << " " << methodname;
      if (entrytype == uima::LogStream::EnMessage) {
        if (lUserCode != 0) {
          str << " RC=" << lUserCode;
        }
      } else {
        str << " RC=" << lUserCode;
      } 
      str << " " << message << endl;

      apr_size_t len = str.str().length();
      rv = apr_socket_send(socket, str.str().c_str(), &len);

      if (rv != APR_SUCCESS) {
        cerr << __FILE__ << __LINE__ <<   " apr_socket_send() failed " <<  str.str() << endl;
      }
      apr_thread_mutex_unlock(mutex);
    }

    //used by Monitor to send status messages
    void log(std::string message) {
      apr_thread_mutex_lock(mutex);
      stringstream str;
      str << message << endl;
      apr_size_t len = str.str().length();
      apr_status_t rv = apr_socket_send(socket, str.str().c_str(), &len);

      if (rv != APR_SUCCESS) {
        cerr << "apr_socket_send() failed " <<  str.str() << endl;
      }
      apr_thread_mutex_unlock(mutex);
    }
  };


class Monitor {
  public:
  //Constructor

    Monitor (apr_pool_t * pool, string brokerURL,
      string queueName, string aeDesc,
      int numInstances, int prefetchSize, int processCasErrorThreshhold,
      int processCasErrorWindow, bool terminateOnCPCError, 
      SocketLogger * logger)         
    {
      iv_status = "Initializing";
      iv_quiesceandstop=false;
      iv_brokerURL = brokerURL;
      iv_queueName = queueName;
      iv_aeDescriptor = aeDesc;
      iv_numInstances = numInstances;
      iv_prefetchSize = 0;
      iv_cpcErrors = 0;
      iv_getmetaErrors = 0;
      iv_processcasErrors = 0;
      for (int i=0; i < numInstances;i++) {
        iv_idleTimes[i] = 0;
      }
      iv_deserTime = 0;
      iv_serTime = 0;
      iv_annotatorTime = 0;
      iv_sendTime = 0;
      iv_startTime = apr_time_now();
      iv_numCasProcessed = 0;
      iv_errorThreshhold = processCasErrorThreshhold;
      iv_errorWindow = processCasErrorWindow;
      iv_terminateOnCPCError = terminateOnCPCError;
      iv_getmetaId = -1;
      iv_numRunning = 0;
      iv_pLogger = logger;
      mutex = 0;
      lmutex = 0;
      cond = 0;
      cond_mutex = 0;
      apr_status_t rv = apr_thread_mutex_create(&mutex,APR_THREAD_MUTEX_UNNESTED, pool);
      apr_thread_mutex_create(&cond_mutex,APR_THREAD_MUTEX_UNNESTED, pool);
      apr_thread_mutex_create(&lmutex,APR_THREAD_MUTEX_UNNESTED, pool);
      apr_thread_cond_create(&cond, pool);
    }

    void print();

    void shutdown() {
      //apr_thread_mutex_unlock(mutex);
      //  apr_thread_mutex_unlock(lmutex);
      apr_thread_cond_signal(this->cond);
      return ;
    }

    void setQuiesceAndStop() {
      //apr_thread_mutex_unlock(mutex);
      //  apr_thread_mutex_unlock(lmutex);
      this->iv_quiesceandstop = true;
      apr_thread_cond_signal(this->cond);
      return ;
    }

    bool getQuiesceAndStop() {
      return iv_quiesceandstop;
    }

    const string & getBrokerURL() const {
      return iv_brokerURL;
    }

    const string & getQueueName() const {
      return iv_queueName;
    }

    const string & getAEDescriptor() const {
      return iv_aeDescriptor;
    }

    const int getNumberOfInstances() const {
      return iv_numInstances;
    }

    void setBrokerURL(const string url) {
      iv_brokerURL = url;
    }

    void setQueueName(const string qname) {
      iv_queueName = qname;
    }

    void setAEDescriptor(const string loc) {
      iv_aeDescriptor = loc;
    }

    void setNumberOfInstances(int num) {
      iv_numInstances=num;
      iv_numRunning = num;
    }

    void listenerStopped(int id) {
      cout << "listener stopped " << id << endl;
      if (id == iv_getmetaId || iv_numRunning == 1) {
        shutdown();
      } else {
        apr_thread_mutex_lock(mutex);
        iv_numRunning--;
        apr_thread_mutex_unlock(mutex);
      }
    }

    void  setGetMetaListenerId(int id) {
      iv_getmetaId = id;
    }

    void setStartTime() {
      apr_thread_mutex_lock(mutex);
      iv_startTime = apr_time_now();
      apr_thread_mutex_unlock(mutex);
    }

    apr_time_t getServiceStartTime() {
      return iv_startTime;
    }

    void logMessage(string message) {
      apr_thread_mutex_lock(lmutex);
      uima::ResourceManager::getInstance().getLogger().logMessage(message);
      apr_thread_mutex_unlock(lmutex);
    }

    void logWarning(string message) {
      apr_thread_mutex_lock(lmutex);
      uima::ResourceManager::getInstance().getLogger().logWarning(message);
      apr_thread_mutex_unlock(lmutex);
    }
    void logError(string message) {
      apr_thread_mutex_lock(lmutex);
      uima::ResourceManager::getInstance().getLogger().logError(message,-99);
      apr_thread_mutex_unlock(lmutex);
    }

    void reconnecting(int id) {
      apr_thread_mutex_lock(mutex);
      if (id == this->iv_getmetaId) {
        iv_status = "Reconnecting";
        if (iv_pLogger != 0) 
          iv_pLogger->log("StatusReconnecting");
      }
      apr_thread_mutex_unlock(mutex);
    }

    void reconnectionSuccess(int id) {
      apr_thread_mutex_lock(mutex);
      if (id == this->iv_getmetaId ) {
        iv_status = "Running";
        if (iv_pLogger != 0) 
          iv_pLogger->log("StatusReconnectionSuccess");
      }
      apr_thread_mutex_unlock(mutex);
    }

    void running(int id){
      apr_thread_mutex_lock(mutex);
      if (id == this->iv_getmetaId) {
        iv_status = "Running";
      }
      apr_thread_mutex_unlock(mutex);
    }

    //Called when a processing of a PROCESSCAS or CPC request is started.
    //It records the processing start time and accumulates the idle time 
    //for this message processing instance.  
    void processingStarted(int id, apr_time_t startTime, apr_time_t idleTime) {
      apr_thread_mutex_lock(mutex);
      if (id != iv_getmetaId) {
        //cout << "processingStarted idleTime=" << idleTime << endl;
        iv_processingStarted[id] = startTime; //record start time
        if (idleTime > 0) {                   //convert to millis 
          iv_idleTimes[id] += (idleTime/1000);
        } else {
          iv_idleTimes[id] = 0;
        }
      }
      apr_thread_mutex_unlock(mutex);
    }

    //Called when handling of a PROCESSCAS or CPC request is finished.
    //Records the timing data and error counts. 
    //Resets the processingStarted and processingCompleted timestamps
    //for this instance.
    //All times are in microseconds.
    void processingComplete(int id, int command, bool success, 
      apr_time_t totalTime,
      apr_time_t deserTime,
      apr_time_t analyticTime,
      apr_time_t serTime,
      apr_time_t sendTime) {		  
        apr_thread_mutex_lock(mutex);
        iv_messageProcessTime += totalTime;
        iv_sendTime += sendTime;
        if (command == PROCESS_CAS_COMMAND) {
          iv_numCasProcessed++;
          iv_deserTime += deserTime;
          iv_annotatorTime += analyticTime;
          iv_serTime += serTime;
        } else if (command == GET_META_COMMAND) {
          iv_getmetaTime += totalTime;
        } else if (command == CPC_COMMAND) {
          iv_cpcTime += totalTime;
        } 

        if (!success) {
          incrementErrorCount(command); 
        }
        if (id != iv_getmetaId) {
          iv_processingStarted[id] = 0;
          iv_processingComplete[id] = apr_time_now();
        }
        apr_thread_mutex_unlock(mutex);
    }

    //Takes a snapshot of the statistics and writes it to the 
    //socket.  This is invoked when the UimacppServiceControllerMBean
    //requests statistics for this service. 
    void writeStatistics(apr_socket_t * cs) {
      apr_thread_mutex_lock(mutex);
      stringstream str; 
      getStatistics(str);
      apr_size_t len = str.str().length();
      apr_status_t rv = apr_socket_send(cs, str.str().c_str(), &len);
      len = 1;
      apr_socket_send(cs,"\n", &len);
      if (rv != APR_SUCCESS) {
        //TODO throw exception
        cout << "apr_socket_send() failed " << endl;
      }
      cout << "ThreadId: "  << apr_os_thread_current() << " writeStatistics() "             << str.str() << endl; 
      apr_thread_mutex_unlock(mutex);
    }


    //Collects and formats statistics data.
    void getStatistics(stringstream & str) {
      apr_time_t snapshotTime = apr_time_now();
      apr_time_t totalProcessTime = (snapshotTime - iv_startTime)*iv_numInstances;
      //accumulate idle times recorded by instances as they processed 
      //requests. If an instance has never recorded idle time, 
      //then it has not processed any requests and idle time for that
      //instance is computed from time the service was started.
      INT64 idleTime = 0;
      for (int i=0; i < iv_numInstances; i++) { 
        if (iv_idleTimes[i] == 0 ) { 
          INT64 idleT = (snapshotTime - iv_startTime)/1000; //is this right ??
          if (idleT > 0) {
            idleTime += idleT;
          }
        }  else { 
          if (iv_idleTimes[i] > 0) {
            idleTime += iv_idleTimes[i];
          }
        } 
      }
      //cerr << "IDLETIME recorded by instances " << idleTime << endl;

      //account for instances that have processed requests but are 
      //currently idle
      map<int, apr_time_t>::iterator ite;
      for (ite=iv_processingStarted.begin();ite != iv_processingStarted.end();ite++) {
        if (ite->second == 0) { 
          INT64 idleT = 0;
          if (iv_processingComplete[ite->first]  >  0) {
            idleT = (snapshotTime - iv_processingComplete[ite->first])/1000;
            if (idleT > 0) {
              //cerr << "currently idle " << ite->first << " for " << idleT << endl;
              idleTime += idleT;
            }
          } 
        }
      }

      //cerr << "IDLETIME after accounting for current idle instances " << idleTime << endl;
      //Format string. Each statistics represented as name=value pairs.
      //Each pair separated by a space.  i
      //Convert all times to millis. 
      str << "NUMINSTANCES=" << iv_numInstances
        << " CPCERRORS=" << iv_cpcErrors
        << " GETMETAERRORS=" << iv_getmetaErrors
        << " PROCESSCASERRORS=" << iv_processcasErrors
        << " CPCTIME=" << iv_cpcTime/1000
        << " GETMETATIME=" << iv_getmetaTime/1000
        << " NUMCASPROCESSED=" << iv_numCasProcessed
        << " SERIALIZETIME=" <<  iv_serTime/iv_numInstances/1000
        << " DESERIALIZETIME=" << iv_deserTime/iv_numInstances/1000
        << " ANNOTATORTIME="  << iv_annotatorTime/iv_numInstances/1000
        << " SENDTIME=" << iv_sendTime/iv_numInstances/1000
        << " MESSAGEPROCESSTIME=" << iv_messageProcessTime/iv_numInstances/1000
        << " IDLETIME=" << idleTime/iv_numInstances << " SERVICEUPTIME=" << totalProcessTime/iv_numInstances/1000  
        << " STATUS=" << iv_status <<  " SERVICESTARTTIME=" << iv_startTime 
        << " SNAPSHOTTIME=" << snapshotTime << endl;

      //cout << apr_os_thread_current() << " write statistics " << str.str() << endl;
    }

    void reset() {
      apr_thread_mutex_lock(mutex);

      this->iv_numCasProcessed=0;
      this->iv_annotatorTime=0;
      this->iv_deserTime=0;
      this->iv_serTime=0;     
      this->iv_cpcTime=0;
      this->iv_getmetaTime=0;
      this->iv_sendTime=0;
      this->iv_messageProcessTime=0;

      this->iv_startTime=apr_time_now();

      this->iv_cpcErrors=0;
      this->iv_getmetaErrors=0;
      this->iv_processcasErrors=0;
      this->iv_processingStarted.clear();
      this->iv_processingComplete.clear();
      for (int i=0; i < iv_numInstances; i++) {
        iv_idleTimes[i] =0;
      } 
      apr_thread_mutex_unlock(mutex);
    }

    void  printStatistics();

  public:
    apr_thread_mutex_t *cond_mutex;
    apr_thread_cond_t  *cond;
  private:
    apr_thread_mutex_t *mutex; 
    apr_thread_mutex_t *lmutex; 
    SocketLogger * iv_pLogger;
    string iv_brokerURL;
    string iv_queueName;
    string iv_aeDescriptor;
    string iv_status;
    int iv_numInstances;
    int iv_prefetchSize;
    bool iv_quiesceandstop;

    INT64 iv_cpcErrors;
    INT64 iv_getmetaErrors;
    INT64 iv_processcasErrors;

    int iv_getmetaId;
    /////apr_os_thread_t iv_getmetaThreadId;
    map<int,apr_time_t> iv_processingStarted;
    map<int,apr_time_t> iv_processingComplete;
    map<int,INT64> iv_idleTimes;
    int iv_numRunning;

    apr_time_t iv_deserTime;
    apr_time_t iv_serTime;
    apr_time_t iv_annotatorTime;
    apr_time_t iv_sendTime;
    apr_time_t iv_messageProcessTime;
    apr_time_t iv_cpcTime;
    apr_time_t iv_getmetaTime;
    apr_time_t iv_startTime;

    INT64 iv_numCasProcessed; 
    int iv_errorThreshhold;
    int iv_errorWindow;
    bool iv_terminateOnCPCError;


    void incrementErrorCount(int command) {
      if (command == PROCESS_CAS_COMMAND) {
        iv_processcasErrors++;
        if (this->iv_errorThreshhold > 0 ) {
          if ( this->iv_errorWindow == 0 && iv_processcasErrors >=
            this->iv_errorThreshhold) {
              cerr << " number of PROCESSCAS errors exceeded the threshhold. Terminating the service." << endl;
              shutdown();
          } else {
            //TODO sliding window
          }
        }
      } else if (command == GET_META_COMMAND) {
        iv_getmetaErrors++;
        cerr << "getMeta Error terminating the service." << endl;
        shutdown();
      } else if (command == CPC_COMMAND) {
        iv_cpcErrors++;
        if (this->iv_terminateOnCPCError) {
          cerr << "CPC Error terminating the service." << endl;
          shutdown();
        }
      }    
    }
  };

static SocketLogger * singleton_pLogger =0;
static Monitor * singleton_pMonitor=0;
static apr_socket_t *s=0;  //logger socket
static apr_socket_t *cs=0; //receive commands socket
static apr_sockaddr_t *sa=0;
static apr_status_t rv=0;

static void signal_handler(int signum) {
  stringstream str;
  str << __FILE__ << __LINE__ << " Received Signal: " << signum;
  cerr << str.str() << endl;
  singleton_pMonitor->shutdown();
  //terminateService();
  //activemq::library::ActiveMQCPP::shutdownLibrary();
  // exit(0);

  //singleton_pMonitor->setQuiesceAndStop();
}


static int terminateService() {

  cout << __FILE__ << __LINE__ << "::terminateService" << endl;

  if (cs) {
    apr_socket_close(cs);
    cs=0;
  }

  if (singleton_pMonitor) {
    delete singleton_pMonitor;
    singleton_pMonitor=0;
  }

  if (singleton_pLogger) {
    uima::ResourceManager::getInstance().unregisterLogger(singleton_pLogger);
    delete singleton_pLogger;
    singleton_pLogger =0;
  }

  if (s) {
    apr_socket_close(s);
    s=0;
  }
  return 0;
}


static int initialize(ServiceParameters & serviceDesc,  apr_pool_t * pool) {

  if (serviceDesc.getLoggingLevel() == 0) {
    uima::ResourceManager::getInstance().setLoggingLevel(uima::LogStream::EnMessage);
  } else if (serviceDesc.getLoggingLevel() == 1) {
    uima::ResourceManager::getInstance().setLoggingLevel(uima::LogStream::EnWarning);
  } else if (serviceDesc.getLoggingLevel() == 2) {
    uima::ResourceManager::getInstance().setLoggingLevel(uima::LogStream::EnError);
  }

  /*use only first path that exists */
  if (serviceDesc.getDataPath().length() > 0) {
    uima::util::Filename dataPath("");
    dataPath.determinePath(serviceDesc.getDataPath().c_str());
    uima::util::Location dataLocation(dataPath.getAsCString());
    uima::ResourceManager::getInstance().setNewLocationData(dataLocation);
  }

  //register signal handler
  apr_signal(SIGINT, signal_handler);
  apr_signal(SIGTERM, signal_handler);

  /* set up connection to Java controller bean if a port is specified */
  int javaport=serviceDesc.getJavaPort();
  if (javaport > 0) {
    //cout << "connecting to java controller port " << javaport << endl;
    rv = apr_sockaddr_info_get(&sa, "localhost", APR_INET, javaport, 0, pool);   
    if (rv != APR_SUCCESS) {
      cerr << "ERROR: apr_sockaddr_info_get localhost at port " << javaport << endl;
      return -2;
    }

    rv = apr_socket_create(&s, sa->family, SOCK_STREAM, APR_PROTO_TCP, pool);
    if (rv != APR_SUCCESS) {
      cerr << "ERROR: apr_socket_create() logger connection at port " << javaport << endl;
      return -3;
    }

    rv = apr_socket_connect(s, sa);
    if (rv != APR_SUCCESS) {
      cerr << "ERROR: apr_socket_connect() logger connection at  port " << javaport << endl;
      return -4;
    }

    //commands connection
    rv = apr_socket_create(&cs, sa->family, SOCK_STREAM, APR_PROTO_TCP, pool);
    if (rv != APR_SUCCESS) {
      cerr << "ERROR: apr_socket_create() commands connection at port " <<  javaport << endl;
      return -5;
    }

    rv = apr_socket_connect(cs, sa);
    if (rv != APR_SUCCESS) {
      cerr << "ERROR: apr_socket_connect() commands connection at port " <<  javaport << endl;
      return -6;
    }

    //register SocketLogger
    singleton_pLogger = new SocketLogger(s,pool);
    if (singleton_pLogger == NULL) {
      cerr << "ERROR: SocketLogger() failed. " << endl;
      return -7;
    }
    uima::ResourceManager::getInstance().registerLogger(singleton_pLogger);
  } 
  /* create object to collect JMX stats */
  singleton_pMonitor = new Monitor(pool,
    serviceDesc.getBrokerURL(),serviceDesc.getQueueName(),
    serviceDesc.getAEDescriptor(), serviceDesc.getNumberOfInstances(),
    serviceDesc.getPrefetchSize(),
    serviceDesc.getErrorThreshhold(),
    serviceDesc.getErrorWindow(),
    serviceDesc.terminatOnCPCError(),
    singleton_pLogger);
  return 0;
} //initialize


static void* APR_THREAD_FUNC handleCommands(apr_thread_t *thd, void *data) {

  //if we are here service initialization was successful, send message
  //to controller.
  string msg = "0";
  apr_size_t len = msg.length();
  apr_socket_timeout_set(cs, 5000000);
  rv = apr_socket_send(cs, msg.c_str(), &len);
  if (rv != APR_SUCCESS || len != msg.length() ) {
    singleton_pMonitor->shutdown();
    return 0;
  }
  len = 1;
  apr_socket_send(cs,"\n", &len);
  //cout << "sent 0 to controller " << endl;
  //receive JMX, admin requests from controller 
  char buf[16];
  memset(buf,0,16);
  len = 16;
  apr_socket_timeout_set(cs,-1);
  bool doloop = true;
  //while ( (rv = apr_socket_recv(cs, buf, &len)) != APR_EOF) {
  while (doloop) {
    rv = apr_socket_recv(cs,buf,&len);
    if (APR_STATUS_IS_TIMEUP(rv) ) continue;    
    if (APR_STATUS_IS_EOF(rv)  || APR_STATUS_IS_ECONNRESET(rv) ) {
      cerr << __FILE__ << __LINE__ << "::handleCommands Connection broken... apr_status=" << rv << endl;
      doloop = false;
      continue;
    }
    string command = buf;
    memset(buf,0,16);
    cerr << rv << " " << len << " apr_socket_recv command=" << command << endl;
    len=16;
    if (command.compare("GETSTATS")==0) {
      //singleton_pLogger->log(LogStream::EnMessage,"deployCppService","getStats","retrieving stats",0);
      singleton_pMonitor->writeStatistics(cs);
      //singleton_pLogger->log(LogStream::EnMessage,"deployCppService","getStats","sent statistics",0); 
    } else if (command.compare("RESET")==0) {
      singleton_pLogger->log(uima::LogStream::EnMessage,"deployCppService", "RESET",
        "reset JMX statistics",0);
      singleton_pMonitor->reset();
    } else if (command.compare("QUIESCEANDSTOP")==0) {
      singleton_pLogger->log(uima::LogStream::EnMessage,"deployCppService", "QUIESCEANDSTOP",
        "quiesce and shutdown",0);
      singleton_pMonitor->setQuiesceAndStop();
      break;
    } else if (command.compare("SHUTDOWN")==0) {
      singleton_pMonitor->shutdown();
      break;
    } else {
      if (rv != APR_SUCCESS) {
        singleton_pMonitor->shutdown();
        doloop = false;
        break;
      }   
      else {
        char * c = new char[256];
        apr_strerror(rv, c, 255);
        stringstream str;
        str << c;
        str << "deployCppService::handleCommand() aprerror=" << rv << " invalid command=" << command;
        cerr << str.str() << endl;
        delete c;
      }
    }
  }
  cout << __FILE__ << __LINE__ << "::handleCommand() calling shutdown. " << endl;
  singleton_pMonitor->shutdown();
  apr_thread_exit(thd,APR_SUCCESS);
  return NULL;
}


static void* APR_THREAD_FUNC readstdin(apr_thread_t *thd, void *data) {
  
  printf ("Enter 'q' to quiesce and stop or 's' to stop:\n ");
  char str[2];
  str[0] = '\n';

  
  while( str[0] == '\n' ) {
    scanf("%s", str);

    if (str[0] == 's') {
      singleton_pMonitor->shutdown();
    } else if (str[0] == 'q') {
      singleton_pMonitor->setQuiesceAndStop();
    } else {
       apr_sleep(1000000);
       str[0] = '\n';
       printf ("Enter 'q' to quiesce and stop or 's' to stop:\n ");
    }
  }
  return 0;
}

#endif



