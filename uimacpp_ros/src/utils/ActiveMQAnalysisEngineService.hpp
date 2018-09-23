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

* Analysis Engine service wrapper based on 
* Active MQ C++ client.
*/

#ifndef __ACTIVEMQ_AE_SERVICE__
#define __ACTIVEMQ_AE_SERVICE__
#include <uima/api.hpp>

#include <cms/ConnectionFactory.h>
#include <cms/Connection.h>
#include <cms/Session.h>
#include <cms/TextMessage.h>
#include <cms/ExceptionListener.h>
#include "time.h"
#include <apr_thread_proc.h>

#include <activemq/transport/DefaultTransportListener.h>
using namespace activemq::transport;

using namespace std;
using namespace cms;
using namespace uima;

//Forward declarations
class Monitor;
class ServiceParameters;

/** The function that the request processing thread will run. 
  * It receives and processes each message from the input queue
  * as it arrives.
  */
static void* APR_THREAD_FUNC handleMessages(apr_thread_t *thd, void *data);


/** common base class */
class CommonUtils {
protected:
  Monitor * iv_pMonitor;
  
  void logError(string msg);
 
  void logWarning(string msg);
 
  void logMessage(string msg);

};
//==========================================================
// This class wraps a ActiveMQ JMS connection 
// and provides utility methods to create a MessageProducer
// and MessageConsumer and to send and receive messages.
//----------------------------------------------------------
class AMQConnection : public ExceptionListener, 
	public DefaultTransportListener,
                      public CommonUtils {
private:
  string iv_brokerURL;
 
  Connection* iv_pConnection;
  bool iv_valid;
  bool iv_reconnecting;
  bool iv_started;
  int iv_id;

  //consumer session
  Session* iv_pConsumerSession;
  MessageConsumer * iv_pConsumer;
  string iv_inputQueueName;
  cms::Queue * iv_pInputQueue;      
  MessageListener * iv_pListener;
  TextMessage * iv_pReceivedMessage;
  string iv_selector;
  int iv_prefetchSize;
	
  //producer session
  Session * iv_pProducerSession;
  MessageProducer * iv_pProducer;  
	TextMessage * iv_pReplyMessage;
  map<string, cms::Destination*> iv_replyDestinations; //queuename-destination
  
  void initialize();
public:
  virtual void transportInterrupted();
  virtual void transportResumed();
  static ConnectionFactory * createConnectionFactory(ServiceParameters & params);
  
	/** Establish connection to the broker and create a Message Producer session. 
   */
  AMQConnection ( ConnectionFactory * connFact, string brokerURL, Monitor * pStatistics, int id);

  /** Creates a MessageConsumer session and registers a listener. 
      Caller owns the listener. */
	//void createMessageConsumer(string aQueueName); 
  void createMessageConsumer(string aQueueName, string selector, int prefetchSize); 
 	
  /** destructor */
	~AMQConnection();

	/** caller owns the ExceptionListener */
	void setExceptionListener(ExceptionListener  * el);

	/** This also is a default Exception handler */
	void onException( const CMSException& ex);

	/** If this is consumer, must be called to start receiving messages */ 
	void start();

	/** If this is a consumer, stops receiving messages. */
	void stop();

	/** returns a TextMessage owned by this class. */
	TextMessage * getTextMessage();

	/** sends the reply message and clears it. */
	void sendMessage(string queueName);
	void sendMessage(const Destination * cmsReplyTo);
  void sendMessage (TextMessage * request);

  
  bool isValid() {
    return iv_valid;
  }

  bool isStarted() {
    return iv_started;
  }

  bool isReconnecting() {
    return iv_reconnecting;
  }		    

  /** get the brokerURL */
  string getBrokerURL() {
    return this->iv_brokerURL;
  }
  
  /** get the input queue name */
  string getInputQueueName() {
    return this->iv_inputQueueName;
  }

  /** reset the connection handles */
  void reset();

  /** reset before reconnecting */
  void resetBeforeReconnect();


  /** reestablish broken connection.
   * Attempts to reconnect 30 seconds after each failed attempt.
   */
  void reconnect();

  /** receives the next message for this consumer.
    * Delay is in millis. 
    */
  Message * receive(const int delay);

  ConnectionFactory * iv_pConnFact;
};


//==================================================
// This class is used to cache and reuse connections
// used to send reply messages.
//--------------------------------------------------
class AMQConnectionsCache : public CommonUtils  {
private:
  ConnectionFactory * iv_pConnFact;
	map<string, AMQConnection *> iv_connections; //key is brokerurl
	
public:
	AMQConnectionsCache(ConnectionFactory * pConnFact, Monitor * pStatistics); 

	~AMQConnectionsCache(); 

  /**
	 * Retrieves a Connection from the cache if it 
	 * exists or establishes a new connection to the
	 * the specified broker and adds it to the cache.
   */
	AMQConnection * getConnection(string brokerURL); 
	
};

//======================================================
// This class handles getMeta, processCAS
// and Collection Processing Complete requests.
//
// Records timing and error JMX statistics.
//
//------------------------------------------------------
class AMQListener : public CommonUtils            {
private:
  apr_thread_t *thd;
  bool iv_stopProcessing;

	int iv_id;									    //Listener id
	string iv_inputQueueName;				//queue this listener gets messages from
	string iv_brokerURL;				    //broker this listener is connected to 
  AMQConnection * iv_pConnection; //connection this Listener is registered with.
                                  //Used to send replies to queues on the same broker.
	AnalysisEngine * iv_pEngine;	  //AnalysisEngine
	CAS * iv_pCas;								  //CAS
	string iv_aeDescriptor;			    //AE descriptor XML 
	int iv_count;                   //num messages processed
	bool iv_busy;								    //is processing a message
  apr_time_t iv_timeLastRequestCompleted; //used to calculate idle time between requests
	AMQConnectionsCache    iv_replyToConnections; //maintain connections cache for
                                                //sending reply to other brokers.
	
  void getMetaData(AnalysisEngine * pEngine); 

  void handleRequest(const Message * request);

  bool validateRequest(const TextMessage *, string &);

  void sendResponse(const TextMessage * request, apr_time_t timeToDeserialize,
                    apr_time_t timeToSerialize, apr_time_t timeInAnalytic,
                    apr_time_t idleTime, apr_time_t elapsedTime, 
		                string msgContent, bool isExceptionMsg); 
public:
  
	/** constructor */
  AMQListener(int id,
              AMQConnection * pConnection,
              AnalysisEngine * pEngine,
              Monitor * pStatistics); 

   AMQListener(int id,
              AMQConnection * pConnection,
              AnalysisEngine * pEngine,
              CAS * pCas,
              Monitor * pStatistics); 
  /** destructor */
	~AMQListener(); 

	bool isBusy() {
		return this->iv_busy;
	}

   /* Flag to break out of receive loop */
    void stopProcessing() {
      iv_stopProcessing = true;
    }

   bool isReconnecting() {
     return iv_pConnection->isReconnecting();
   }
    /* 
     * Synchronously receives and processes messages 
     */
    void receiveAndProcessMessages(apr_thread_t *thd);

	
};

//===================================================
// This class creates and configures a C++ service
// according to parameters passed in as arguments.
//
// This class creates connections to an ActiveMQ broker 
// and registers one or more MessageConsumers to receive 
// messages from a specified queue. 
//
// A thread is started for each instance. Each thread maintains a
// connection to the broker, and an instance of the UIMA AnalysisEngine.
// To support fast handling of GETMETA requests, an additional separate
// MessageConsumer and thread is started that processes only GETMETA requests.
// 
// 
// The service wrapper sets acknowledgment mode to AUTO_ACKNOWLEDGE mode 
// by default and lets the underlying middleware handle the message
// acknowledgements.
//
// The lifecycle of the service may be managed by the UIMA AS Java 
// controller bean org.apache.uima.controller.UimacppServiceController.
// In this case, the C++ service is started by the controller bean.
// A socket connection is established between the controller bean and
// the C++ process and used to route logging messages and JMX statistics.
//
// See the UIMA-EE documentation for how to start and manage a C++
// servcice from Java using the UimacppServiceController bean.
//------------------------------------------------------------------------
class AMQAnalysisEngineService : public  CommonUtils {
private:

  apr_pool_t * iv_pool;
  apr_threadattr_t *thd_attr;
  ConnectionFactory * iv_pConnFact;
  vector<apr_thread_t *> iv_listenerThreads;


  string iv_brokerURL;  
	string iv_inputQueueName;  
  string iv_aeDescriptor;
	int iv_numInstances;					
	int iv_prefetchSize;
  size_t iv_initialFSHeapSize;

  AMQConnection * iv_pgetMetaConnection;
	vector <AMQConnection*> iv_vecpConnections;
  vector <AnalysisEngine *> iv_vecpAnalysisEngines;
  vector <CAS *> iv_vecpCas;
  map<int, AMQListener *> iv_listeners;    //id - listener

  bool iv_started;
  bool iv_closed;

  void initialize(ServiceParameters & params);
public:
  void cleanup();
  
  ~AMQAnalysisEngineService(); 

	AMQAnalysisEngineService(ServiceParameters & desc, Monitor * pStatistics, apr_pool_t * pool);

	void setTraceLevel(int level);

  void startProcessingThreads();
	
	void start();
		
	int stop(); 	

  void quiesce();

  void quiesceAndStop();

  void shutdown();
};

#endif




