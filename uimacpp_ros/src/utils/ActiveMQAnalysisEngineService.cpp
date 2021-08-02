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

#include <activemq/core/ActiveMQConnectionFactory.h>
#include <activemq/core/ActiveMQProducer.h>
#include <activemq/core/ActiveMQConstants.h>
#include <activemq/core/ActiveMQConnection.h>

#include <uima/xmlwriter.hpp>
#include <uima/xcasdeserializer.hpp>
#include <uima/xmiwriter.hpp>
#include <uima/xmideserializer.hpp>
#include <uima/xmlerror_handler.hpp>

using namespace activemq::core;
using namespace uima;

enum traceLevels {NONE, INFO, FINE, FINER, FINEST };
traceLevels uimacpp_ee_tracelevel=INFO;
#define MSGHEADER   apr_time_now() << " ThreadId: " << apr_os_thread_current() << __FILE__ << ":" << __LINE__ 
#define FORMATMSG(x)  stringstream lstr; lstr << MSGHEADER << " " << x;

#define LOGINFO(n,x) { if (n > uimacpp_ee_tracelevel) {} else { FORMATMSG(x); logMessage(lstr.str().c_str()); } }
#define LOGERROR(x){ FORMATMSG(x); logError(lstr.str().c_str()); }
#define LOGWARN(x) { FORMATMSG(x); logWarning(lstr.str().c_str());}

static void listener_signal_handler(int signum) {
    stringstream str;
    str << __FILE__ << __LINE__ << " Received Signal: " << signum;
    cerr << str.str() << endl;    
}


//=================================================================
//
//Message processing function executed by message handling threads.
//-----------------------------------------------------------------
static void* APR_THREAD_FUNC handleMessages(apr_thread_t *thd, void *data) {  
   //cout << __FILE__ << __LINE__ << Thread::getId() << " handleMessages start " << endl;
   AMQListener * handler = (AMQListener*) data;
   handler->receiveAndProcessMessages(thd);
   apr_thread_exit(thd, APR_SUCCESS);
   return NULL;
}


//===================================================
//AMQConnection
//---------------------------------------------------
 ConnectionFactory * AMQConnection::createConnectionFactory(ServiceParameters & params) {

   //encode prefetch size in the broker url as there is no api to set it.
   stringstream str;
   str << params.getBrokerURL();
   if (str.str().find("?") == std::string::npos)
     str << "?jms.prefetchPolicy.queuePrefetch=" << params.getPrefetchSize() ;
   else 
      str << "&jms.prefetchPolicy.queuePrefetch=" << params.getPrefetchSize() ;
  
   //str << params.getBrokerURL() << "?consumer.prefetchSize=" << params.getPrefetchSize() << endl;
   //str << "tcp://127.0.0.1:61616";
   ConnectionFactory * pConnFactory = ConnectionFactory::createCMSConnectionFactory(params.getBrokerURL());
   cout << "AMQConnection()::createConnectionFactory " <<  str.str() << endl;
   return pConnFactory;
 };

 AMQConnection::AMQConnection(  ConnectionFactory * connFact, 
                                      string brokerURL, Monitor * pMonitor, int id) :
                                      iv_id(id),
                                      iv_pConnFact(connFact),
                                      //iv_pMonitor(0),
                                      //iv_brokerURL(((ActiveMQConnectionFactory*)connFact)->getBrokerURL()), 
									  iv_brokerURL(brokerURL),
                                      iv_pConnection(0), 
                                      iv_pConsumerSession(0),
                                      iv_pConsumer(0),
                                      iv_inputQueueName(),
                                      iv_pInputQueue(0),
                                      iv_pListener(0),
                                      iv_pProducerSession(0),
                                      iv_pProducer(0),
                                      iv_pReplyMessage(0),
                                      iv_replyDestinations(),
                                      iv_valid(false),
                                      iv_started(false),
                                      iv_reconnecting(false)  {
    int pos  = iv_brokerURL.find_first_of("?");
    if (pos != string::npos) {
      iv_brokerURL = iv_brokerURL.substr(0,pos);
    }
    iv_pMonitor = pMonitor;
    initialize();
 }
 void AMQConnection::initialize( ) {

    try {
  
      LOGINFO(INFO, "AMQConnection() connecting to " + iv_brokerURL);
      
      // Create a Connection
      if (iv_pConnFact == NULL) {	
        LOGERROR("AMQConnection() invalid connection factory");
        ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
        msg.addParam("AMQConnection() invalid create connection factory");
        ErrorInfo errInfo;
        errInfo.setMessage(msg);
        UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
          UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
          errInfo.getMessage(),
          errInfo.getMessage().getMessageID(),
          ErrorInfo::unrecoverable);
      }

      bool retrying = false;
      while (iv_pConnection == NULL) {
        try {
          iv_pConnection = iv_pConnFact->createConnection();
          if (iv_pConnection == NULL ) {
            if (!retrying) {
              stringstream str;
              str << " AMQConnection::initialize() Connection object is null. Failed to connect to " << iv_brokerURL
                         << ". Retrying..." << endl;
              LOGWARN(str.str());
              retrying = true;
              apr_sleep(30000000); //wait 30 seconds to reconnect
            }
          }
        } catch (cms::CMSException& e) {
          if (!retrying) {
            stringstream str;
            str << "AMQConnection::initialize() Failed to connect to " << iv_brokerURL
                         << e.getMessage() << ". Retrying..." << endl;
			cout << e.getMessage() << endl;
            LOGWARN(str.str());
            retrying = true;
            apr_sleep(30000000); //wait 30 seconds to reconnect
          }
        } 
      }
     
      if (retrying) {
        LOGWARN("AMQConnection::initialize() Connected to " + iv_brokerURL);
      }

      //default exception listener
      this->iv_pConnection->setExceptionListener(this);	
      ActiveMQConnection* amqConnection = dynamic_cast<ActiveMQConnection*>( this->iv_pConnection );
      if( amqConnection != NULL ) {
        amqConnection->addTransportListener( this );
      }
      // Create a Producer Session
      LOGINFO(FINEST,"AMQConnection() create Producer Session " + iv_brokerURL);
      this->iv_pProducerSession = this->iv_pConnection->createSession( Session::AUTO_ACKNOWLEDGE );

      if  (this->iv_pProducerSession == NULL) {
        LOGERROR("AMQConnection() createSession() failed."); 
        ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
        msg.addParam("AMQConnection() createSession failed." );
        ErrorInfo errInfo;
        errInfo.setMessage(msg);
        UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
          UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
          errInfo.getMessage(),
          errInfo.getMessage().getMessageID(),
          ErrorInfo::unrecoverable);
      } 
      this->iv_pProducer = this->iv_pProducerSession->createProducer(NULL);
      if (this->iv_pProducer == NULL) {
        LOGERROR("AMQConnection() could not create MessageProducer ");
        ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
        msg.addParam("AMQConnection() create MessageProducer failed.");
        ErrorInfo errInfo;
        errInfo.setMessage(msg);
        UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
          UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
          errInfo.getMessage(),
          errInfo.getMessage().getMessageID(),
          ErrorInfo::unrecoverable);
      }
      this->iv_pProducer->setDeliveryMode( DeliveryMode::NON_PERSISTENT );
	  
	  //TODO set sendTimeout ?
      ////((ActiveMQProducer*)this->iv_pProducer)->setSendTimeout(3000);
      
	  //create TextMessage
      this->iv_pReplyMessage = this->iv_pProducerSession->createTextMessage();
      if (this->iv_pReplyMessage == NULL) {
        LOGERROR("AMQConnection() create textMessage failed. ");
        ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
        msg.addParam("AMQConnection() failed to create message.");
        ErrorInfo errInfo;
        errInfo.setMessage(msg);
        UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
          UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
          errInfo.getMessage(),
          errInfo.getMessage().getMessageID(),
          ErrorInfo::unrecoverable);
      }
      this->iv_valid = true;
      LOGINFO(0, "AMQConnection() connected successfully to " + iv_brokerURL);
    } catch (cms::CMSException& e) {
      LOGERROR("AMQConnection(): " + e.getMessage());  
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam(e.getMessage());
      ErrorInfo errInfo;
      errInfo.setErrorId(UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE);
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    } catch (...) {
      cout << "... " << endl;
      LOGERROR("AMQConnection() failed to create a connection");  
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam("AMQConnection() create connection failed");
      ErrorInfo errInfo;
      errInfo.setErrorId(UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE);
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }
  }

/* destructor */
  AMQConnection::~AMQConnection() {
    if (this->iv_pConsumer != NULL) {
      delete this->iv_pConsumer;
      this->iv_pConsumer = NULL;
    }
    if (this->iv_pProducer != NULL) {
      delete this->iv_pProducer;
      this->iv_pProducer = NULL;
    }
    if (this->iv_pConsumerSession != NULL) {
      delete this->iv_pConsumerSession;
      this->iv_pConsumerSession = NULL;
    }
    if (this->iv_pProducerSession != NULL) {
      delete this->iv_pProducerSession;
      this->iv_pProducerSession = NULL;
    }
    if (this->iv_pInputQueue != NULL) {
      delete this->iv_pInputQueue;
      this->iv_pInputQueue=NULL;
    }
    if (this->iv_pConnection != NULL) {
      this->iv_pConnection->close();
      delete this->iv_pConnection;
      this->iv_pConnection = NULL;
    }
    if (this->iv_pReplyMessage != NULL) {
      delete this->iv_pReplyMessage;
      this->iv_pReplyMessage=NULL;
    } 
    
    //destinations
    map<string, cms::Destination*>::iterator ite; 
    for (ite= iv_replyDestinations.begin();ite !=  iv_replyDestinations.end();ite++) {
      delete ite->second;
    }
  }

/* create a MessageConsumer session and register a MessageListener
   to receive messages from the input queue. */
  void AMQConnection::createMessageConsumer(string queueName, string selector, int prefetchSize) {
    LOGINFO(FINEST, "AMQConnection::createMessageConsumer() consumer start " + queueName);
    this->iv_inputQueueName = queueName;
    stringstream str;
    str << queueName;
   
    if (this->iv_pConsumerSession != NULL || iv_pConsumer != NULL) {
      LOGERROR("AMQConnection::createMessageConsumer() A session already exists. ");
      ErrorInfo errInfo;
      errInfo.setMessage(ErrorMessage(UIMA_MSG_ID_LOG_ERROR, "A session already exists."));
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }

    this->iv_pConsumerSession = this->iv_pConnection->createSession( Session::AUTO_ACKNOWLEDGE );		
    if  (this->iv_pConsumerSession == NULL) {
      LOGERROR("AMQConnection() createSession failed."); 
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam("AMQConneciton() createSession failed." );
      ErrorInfo errInfo;
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    } 

    this->iv_pInputQueue = this->iv_pConsumerSession->createQueue(this->iv_inputQueueName);

    if (this->iv_pInputQueue == NULL) {
      LOGERROR("AMQConnection::createMessageConsumer() createQueue failed. " + queueName);
      stringstream str;
      str << "AMQConnection::createMessageConsumer() createQueue failed. " << queueName << endl;
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam(str.str());
      ErrorInfo errInfo;
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }

    iv_selector = selector;
	this->iv_prefetchSize = prefetchSize;
    if (selector.length() > 0) {
      this->iv_pConsumer = this->iv_pConsumerSession->createConsumer(this->iv_pInputQueue, iv_selector);
    } else {
      this->iv_pConsumer = this->iv_pConsumerSession->createConsumer(this->iv_pInputQueue);
    }

    if (this->iv_pConsumer == NULL) {
      LOGERROR("AMQConnection::createMessageConsumer() createConsumer failed. " + queueName);
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam("AMQConnection::createMessageConsumer() createConsumer() failed.");
      ErrorInfo errInfo;
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }
    //set prefetch size
	//no api to set prefetchsize so encode in broker url;

    LOGINFO(FINEST, "AMQConnection::createMessageConsumer() " + queueName + " successful.");
  }


  void AMQConnection::onException(const CMSException & ex) {
    //mark endpoint as broken.
    this->iv_valid = false;
    //log that connection is invalid.
    stringstream str;
    str << "AMQConnection()::onException() Connection to " 
                       << iv_brokerURL << " is broken. Reconnecting ... " << ex.getMessage() << endl;
    LOGWARN(str.str());
  }
  void AMQConnection::transportInterrupted() {
        std::cout << "The Connection's Transport has been Interrupted." << std::endl;
		stringstream str;
        str << "AMQConnection()::transportInterrupted() Connection to " 
                       << iv_brokerURL << " has been interrupted. Reconnecting ... " << endl;
        LOGWARN(str.str());
    }

  void AMQConnection::transportResumed() {
        std::cout << "The Connection's Transport has been Restored." << std::endl;
		stringstream str;
        str << "AMQConnection()::transportResumed() Reconnected to " 
                       << iv_brokerURL  << endl;
        LOGWARN(str.str());
    }

  TextMessage * AMQConnection::getTextMessage() {
    if (this->iv_pReplyMessage == NULL) {
      LOGERROR("AMQConnection::getTextMessage() failed. ");
      ErrorInfo errInfo;
      errInfo.setMessage(ErrorMessage(UIMA_MSG_ID_LOG_ERROR, "TextMessage could not be created."));
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }
    iv_pReplyMessage->clearProperties();
    iv_pReplyMessage->clearBody();
    return this->iv_pReplyMessage;
  }

  void AMQConnection::sendMessage(string queuename) {
    LOGINFO(FINEST, "AMQConnection::sendMessage() to " + queuename);
    if (this->iv_pProducer == NULL) {
      LOGERROR("AMQConnection::sendMessage() Invalid Message producer. ");
      ErrorInfo errInfo;
      errInfo.setMessage(ErrorMessage(UIMA_MSG_ID_LOG_ERROR, "No message producer."));
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }
    //look up destination queue
    cms::Destination * pDest;
    map<string,Destination*>::iterator ite = this->iv_replyDestinations.find(queuename);
    if (ite != this->iv_replyDestinations.end()) {
      pDest = ite->second;
    } else {
      pDest = this->iv_pProducerSession->createQueue(queuename);
      if (pDest != NULL) {
        this->iv_replyDestinations[queuename] = pDest;
      }
    }

    if (pDest == NULL) {
      LOGERROR("AMQConnection::sendMessage() invalid destination " + queuename);
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam("AMQConnection::sendMessage() invalid destination.");
      ErrorInfo errInfo;
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }

    this->iv_pProducer->send(pDest,this->iv_pReplyMessage);

    //cout << "producer->send elapsed time " << (apr_time_now() - stime) << endl;
    this->iv_pReplyMessage->clearBody();
    this->iv_pReplyMessage->clearProperties();
    LOGINFO(FINEST, "AMQConnection::sendMessage() successful to " + queuename);
  }

  void AMQConnection::sendMessage(const Destination * cmsReplyTo) {
    LOGINFO(FINEST, "AMQConnection::sendMessage() to " +
 ((Queue*)cmsReplyTo)->getQueueName() );
    //LOGINFO(FINEST, "AMQConnection::sendMessage() to " + cmsReplyTo->toProviderString());
    if (this->iv_pProducer == NULL) {
      LOGERROR("AMQConnection::getTextMessage Invalid message producer. ");
      ErrorInfo errInfo;
      errInfo.setMessage(ErrorMessage(UIMA_MSG_ID_LOG_ERROR, "A MessageProducer does not exist."));
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }
    this->iv_pProducer->send(cmsReplyTo,this->iv_pReplyMessage);
    //cout << "producer->send elapsed time " << (apr_time_now() - stime) << endl;
    this->iv_pReplyMessage->clearBody();
    this->iv_pReplyMessage->clearProperties();
    LOGINFO(4, "AMQConnection::sendMessage() successful to " + ((Queue*)cmsReplyTo)->getQueueName());
	//LOGINFO(4, "AMQConnection::sendMessage() successful to " + cmsReplyTo->toProviderString());
  }

// must be called to start receiving messages 
  void AMQConnection::start() {
    if (this->iv_pConnection != NULL) {
      this->iv_pConnection->start();
      LOGINFO(0,"AMQConnection::start() Start receiving messages.");
    } else {
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam("AMQConnection::start() failed. A connection does not exist.");
      ErrorInfo errInfo;
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }	
  }

//stops receiving messages
  void AMQConnection::stop() {
    if (this->iv_pConnection != NULL) {
      this->iv_pConnection->stop();
    } else {
      LOGERROR("AMQConnection::stop() invalid connection. ");
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam("stop() invalid connection.");
      ErrorInfo errInfo;
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }
  }

  void AMQConnection::resetBeforeReconnect() {
    this->iv_reconnecting = true;

    if (this->iv_pConsumer != NULL) {
      delete this->iv_pConsumer;
      this->iv_pConsumer = NULL;
    }
    if (this->iv_pProducer != NULL) {
      delete this->iv_pProducer;
      this->iv_pProducer = NULL;
    }
    if (this->iv_pConsumerSession != NULL) {
      delete this->iv_pConsumerSession;
      this->iv_pConsumerSession = NULL;
    }
    if (this->iv_pProducerSession != NULL) {
      delete this->iv_pProducerSession;
      this->iv_pProducerSession = NULL;
    }
    if (this->iv_pConnection != NULL) {
      delete this->iv_pConnection;
      this->iv_pConnection = NULL;
    }

    this->iv_started = false;
    this->iv_valid = false;

    if (this->iv_pReplyMessage != NULL) {
      delete this->iv_pReplyMessage;
      this->iv_pReplyMessage=NULL;
    } 
    
    //destinations
    map<string, cms::Destination*>::iterator ite; 
    for (ite= iv_replyDestinations.begin();ite !=  iv_replyDestinations.end();ite++) {
      delete ite->second;
    }
  }

  void AMQConnection::reconnect() {
    try {
        this->iv_reconnecting = true;
        this->iv_pMonitor->reconnecting(iv_id);
        apr_sleep(30000000); //wait 30 seconds to reconnect
        //cout << "AMQConnection::reconnect() calling initialize >>>" << endl;
        this->initialize();
        this->createMessageConsumer(this->iv_inputQueueName,this->iv_selector, this->iv_prefetchSize);
        this->start();
        stringstream str;
        str << "AMQConnection::reconnect() Successfully reconnected to >>> " <<  iv_brokerURL << endl;
        LOGWARN(str.str());
        this->iv_pMonitor->reconnectionSuccess(iv_id);
        this->iv_reconnecting = false;
     } catch (uima::Exception e) {
       cerr << "AMQConnection::reconnect() " << e << endl;	
       resetBeforeReconnect();
     } catch (...) {
       cerr << "AMQConnection::reconnect() catch ... " << endl;	
       resetBeforeReconnect();
     }

  }
  

  Message * AMQConnection::receive(const int delay) {
      if (iv_valid) {
        return iv_pConsumer->receive(delay);
      } else {
        this->resetBeforeReconnect();
        this->reconnect();
        this->iv_valid = true;
        return NULL;
      }
  }

//===================================================
//AMQConnectionCache
//---------------------------------------------------
  AMQConnectionsCache::AMQConnectionsCache(ConnectionFactory * pConnFact,Monitor * stats) {
    iv_pConnFact = pConnFact;
    iv_pMonitor = stats;
  }

  AMQConnectionsCache::~AMQConnectionsCache() {
    map<string,AMQConnection*>::iterator ite;
    for (ite = this->iv_connections.begin(); ite != this->iv_connections.end(); ite++) {
      if (ite->second != NULL) {
        delete ite->second;
      }	
    }	
  }

//Retrieves a Connection from the cache if it 
//exists or establishes a connection to the
//the specified broker and adds to the cache 
//and returns the new connection.
  AMQConnection * AMQConnectionsCache::getConnection(string brokerURL) {
    LOGINFO(FINE,"AMQConnectionCache::getConnection() looking up connection to " 
      + brokerURL );	 
    AMQConnection * connection = NULL;

    try {	    	
      map<string,AMQConnection*>::iterator ite;
      ite = this->iv_connections.find(brokerURL);
      if (ite == iv_connections.end()) {
        LOGINFO(FINE,"AMQConnectionsCache::getConnection() create new connection to " +
          brokerURL);
        connection = new AMQConnection(iv_pConnFact, brokerURL, iv_pMonitor, iv_connections.size());
        if (connection == NULL) {
          LOGERROR("AMQConnectionCache::getConnection Could not create a endpoint connection to " +
            brokerURL);
        } else {								    
          this->iv_connections[brokerURL] = connection;
        }
      } else {
        connection = ite->second;
        //if not a valid connection, reconnect
        if (connection == NULL) {
          LOGERROR("AMQConnectionCache::getConnection() could not connect to "
            + brokerURL);
        } else {
          if (!connection->isValid()) {
            LOGWARN("AMQConnectionCache::getEndPoint() Existing connection invalid. Reconnecting to " + brokerURL );
            delete connection;
            this->iv_connections.erase(brokerURL);
            connection = new AMQConnection(iv_pConnFact, brokerURL, iv_pMonitor, iv_connections.size());
            if (connection == NULL) {
              LOGERROR("AMQConnectionCache::getConnection() could not connect to "
                + brokerURL );
            } else {
              LOGWARN("AMQConnectionCache::getConnection() reconnected to " + 
                brokerURL);
              this->iv_connections[brokerURL] = connection; 
            }
          }
        }
      }
      return connection;		
    } catch (cms::CMSException & e) {
      LOGERROR("AMQConnectionCache::getConnection() " + e.getMessage());
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam("AMQConnectionCache::getConnection() " + e.getMessage());
      ErrorInfo errInfo;
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);

    } catch (...) {
      LOGERROR("AMQConnectionCache::getConnection() Unknown Exception. ");
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam("AMQConnectionCache::getConnection Unknown Exception");
      ErrorInfo errInfo;
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }
  }


//===================================================
//Listener that process UIMA requests. 
//---------------------------------------------------

  AMQListener::AMQListener(int id,
                           AMQConnection * connection,
                           AnalysisEngine * ae,
                           CAS * cas,
                           Monitor * stats) : 
                            iv_id(id),
                            iv_pConnection(connection),
                            iv_pEngine(ae),
                            iv_pCas(cas),
                            //iv_pMonitor(stats),
                            iv_replyToConnections(connection->iv_pConnFact,stats),
                            iv_timeLastRequestCompleted(0),
                            iv_busy(false),
                            iv_stopProcessing(false),
                            iv_count(0),
                            iv_aeDescriptor(),
                            iv_brokerURL(connection->getBrokerURL()),
                            iv_inputQueueName(connection->getInputQueueName()) {

    iv_pMonitor = stats;
    getMetaData(iv_pEngine); 
 
  }

  AMQListener::AMQListener(int id,
                           AMQConnection * connection,
                           AnalysisEngine * ae,
                           Monitor * stats) : 
                            iv_id(id),
                            iv_pConnection(connection),
                            iv_pEngine(ae),
                            //iv_pMonitor(stats),
                            iv_replyToConnections(connection->iv_pConnFact, stats),
                            iv_timeLastRequestCompleted(0),
                            iv_busy(false),
                            iv_count(0),
                            iv_aeDescriptor(),
                            iv_brokerURL(connection->getBrokerURL()),
                            iv_inputQueueName(connection->getInputQueueName()) {

    iv_pMonitor = stats;
    getMetaData(iv_pEngine); 
 
  }

  AMQListener::~AMQListener() {

  }

  //extract AE descriptor as XML to use when processing GETMETA requests.
 void AMQListener::getMetaData(AnalysisEngine * pEngine) {
    if (pEngine == NULL) {
      LOGERROR("AMQListener::getMetaData() Invalid handle to AnalysisEngine.");
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam("AMQListener::getMetaData() Invalid handle to AnalysisEngine.");
      ErrorInfo errInfo;
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(Uima_runtime_error, 
            UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
            errInfo.getMessage(),
            errInfo.getMessage().getMessageID(),
            ErrorInfo::unrecoverable);
    }

    CAS * mcas = pEngine->newCAS();
    delete mcas;
    const AnalysisEngineMetaData & aeMetaData = pEngine->getAnalysisEngineMetaData();			
    icu::UnicodeString xmlBuffer;
    xmlBuffer.insert(0, "<?xml version=\"1.0\"?>");
    pEngine->getAnnotatorContext().getTaeSpecifier().toXMLBuffer(aeMetaData,
      false, xmlBuffer);
    UnicodeStringRef uref(xmlBuffer.getBuffer(), xmlBuffer.length());

    this->iv_aeDescriptor = uref.asUTF8();
    //cout << this->iv_aeDescriptor << endl;
  }



bool AMQListener::validateRequest(const TextMessage * textMessage, string & errmsg) {
  bool valid = true; 
  
  if ( textMessage->getCMSReplyTo() == NULL) 
    LOGWARN("AMQListener::validateRequest() JMSReplyTo is not set. " );    
  
  if (textMessage->getCMSReplyTo() == NULL &&
    !textMessage->propertyExists("MessageFrom") ) {
     errmsg = "Reply to destination not set.";
     return false;
  }  

  if (!textMessage->propertyExists("Command") ) {
    errmsg = "Required property 'Command' is not set.";
    LOGERROR("AMQListener::validateRequest " + errmsg);
    valid = false;  
  } else {
    int command = textMessage->getIntProperty("Command");   
    if (command != PROCESS_CAS_COMMAND &&
        command != GET_META_COMMAND &&
        command != CPC_COMMAND) {
        stringstream str;
        str << "Unexpected value for 'Command' " << command;
        errmsg = str.str(); 
        LOGERROR("AMQListener::validateRequest " + errmsg);
        valid=false;
    } else if (command == CPC_COMMAND) {
      if (iv_pEngine == NULL) {
        errmsg = "CPC request received but AnalysisEngine not available.";
        LOGERROR("AMQListener::validateRequest() " + errmsg);
        valid = false;
      }
    } else if (command == PROCESS_CAS_COMMAND) {
      if (iv_pCas == NULL || iv_pEngine == NULL) {
        errmsg = "Process Cas request but an AnalysisEngine and CAS not available.";
        LOGERROR("AMQListener::validateRequest() " + errmsg);
        valid = false;
      }
      if (!textMessage->propertyExists("Payload") ) {
        errmsg = "Required property 'Payload' is not set.";
        LOGERROR("AMQListener::validateRequest " + errmsg);
        valid = false;  
      } else {
        int payload = textMessage->getIntProperty("Payload");
        if (payload != XCAS_PAYLOAD && payload != XMI_PAYLOAD) {
           stringstream str;
           str << "Unexpected value for 'Payload' " << payload;
           errmsg = str.str();
           LOGERROR("AMQListener::validateRequest " + errmsg);
           valid=false;
        } 
		
		try {
          string text = textMessage->getText().c_str();
          if (text.length() == 0) {
            errmsg = "There is no payload data. Nothing to process.";
            LOGERROR("AMQListener::validateRequest " + errmsg);
            valid = false;  
          }
		} catch (CMSException& e) {
			errmsg = e.getMessage();
			LOGERROR("AMQListener::validateRequest " + e.getMessage());
			valid = false;
		}
      }
    }
  }
  return valid;
}


void AMQListener::receiveAndProcessMessages(apr_thread_t * thd) {
 
  try {
    this->thd = thd;
    cout << "Instance: " << iv_id << " ThreadId: " << apr_os_thread_current() <<  " started." << endl;
    //start receiving messages
    //this->iv_pConnection->start();

    this->iv_stopProcessing = false;
    apr_time_t lastStatsTime = apr_time_now(); 
    while (!iv_stopProcessing) {

      stringstream astr;
     
      //cout << "receiveAndProcess going to call recieve" << endl; 
      Message * msg = this->iv_pConnection->receive(2000);

      if (msg != NULL) {
         iv_busy = true;
         astr << this->iv_id;
         astr << " *****Message#: "<< ++iv_count << "*****";
         //cerr << astr.str() << endl;         
         //LOGINFO(FINE,astr.str());
         this->handleRequest(msg);
         ///this->testHandleRequest(msg);
         delete msg;
         msg = 0;
         //cerr << iv_id << " ****Message#: " << iv_count << " deleted " << endl;
         iv_busy = false;
         this->iv_timeLastRequestCompleted = apr_time_now();
         //cout << iv_id << " processMessages done: getnext " << iv_count  << endl;
      }
    
    }
    LOGWARN("AMQListener::receiveAndProcessMessage() stopped receiving messages.");
  } catch (uima::Exception ex ) {
     LOGERROR("AMQListener::receiveAndProcessMessages() " + 
       ex.getErrorInfo().getMessage().asString());
     this->iv_stopProcessing = true;
     //tell the monitor that the thread has stopped processing
     iv_pMonitor->listenerStopped(this->iv_id);
  } catch (...) {
     LOGERROR("AMQListener::receiveAndProcessMessages UnExpected error.");
     iv_stopProcessing = true;
     //tell the monitor that the thread has stopped processing
     iv_pMonitor->listenerStopped(this->iv_id);
  }
}


/*
* Receive a TextMessage and examine the header properties
* to determine the type of request and payload.  Processes
* one request at a time and is blocked till the requestis 
* handled and the response sent.
*/
  void AMQListener::handleRequest( const Message* message ) {


    apr_time_t startTime = apr_time_now();
    apr_time_t timeIdle = 0;
    // Idle time is computed as the interval from time last request 
    // was processed. If this is the first request, idle time is 
    // computed from the time the service was started.  
    if (iv_timeLastRequestCompleted != 0)
      timeIdle =  startTime - iv_timeLastRequestCompleted;
    else 
      timeIdle = startTime - iv_pMonitor->getServiceStartTime(); 

    this->iv_pMonitor->processingStarted(this->iv_id, startTime, timeIdle );
    stringstream astr;
    astr << this->iv_id;
    astr << " ****handleRequest(): "<< iv_count << " start"<< endl;
    LOGINFO(FINE,astr.str());
    ///cout << astr.str() << endl;
    ///LOGWARN(astr.str());
    apr_time_t endTime = 0;
    apr_time_t startSerialize = 0;
    apr_time_t startDeserialize = 0;
    apr_time_t startAnnotatorProcess = 0; 
    apr_time_t startSendResponse = 0;
    apr_time_t timeToDeserializeCAS = 0;
    apr_time_t timeToSerializeCAS = 0;
    apr_time_t timeInAnalytic = 0;

    const TextMessage* textMessage=0;
    int command = 0;

    try {	
    
      textMessage = dynamic_cast< const TextMessage* >( message );
      if (textMessage==0) {
        LOGERROR("AMQListener::handleRequest() invalid pointer to TextMessage");
        endTime = apr_time_now();
        iv_pMonitor->processingComplete(iv_id, 0,false,endTime-startTime,0,0,0,endTime-startTime);
        this->iv_timeLastRequestCompleted = apr_time_now();
        return;
      }
      if (textMessage->propertyExists("MessageFrom")) {	
        LOGINFO(FINER,"Received from " + textMessage->getStringProperty("MessageFrom"));
      } 

      //validate request properties
      string errormessage;
      if (!validateRequest(textMessage, errormessage)) {
        LOGERROR("Listener::handleRequest() " + errormessage);
        endTime = apr_time_now();
        sendResponse(textMessage, 0,0,0,timeIdle, endTime-startTime,
          errormessage ,true);
        endTime = apr_time_now();
        iv_pMonitor->processingComplete(iv_id, 0,false,endTime-startTime,0,0,0,endTime-startTime);
        return;
      }

      command = textMessage->getIntProperty("Command");
      astr.str("");
      astr << "Received request Command: " << command ;
      if (textMessage->propertyExists("CasReference")) {
        astr << " CasReference " << textMessage->getStringProperty("CasReference");
      }
   
      astr << "Received request Command: " << command ;
      if (textMessage->propertyExists("CasReference")) {
        astr << " CasReference " << textMessage->getStringProperty("CasReference");
      }
      LOGINFO(INFO,astr.str());

      if (command == PROCESS_CAS_COMMAND) { //process CAS
        LOGINFO(FINE,"Process CAS request start.");
        int payload = textMessage->getIntProperty("Payload");
        //get the text in the payload 
        string text = textMessage->getText().c_str();
        icu::UnicodeString utext(text.c_str());
		text = UnicodeStringRef(utext).asUTF8();
        astr.str("");
        astr << "Payload: " << payload << " Content: " << text ;
        LOGINFO(FINER, astr.str());

        //InputSource
        MemBufInputSource memIS((XMLByte const *)text.c_str(),
          text.length(),
          "sysID");

        //reset the CAS
        iv_pCas->reset();
        ios::openmode mode = ios::binary;
        stringstream xmlstr;  
        //deserialize payload data into the CAS,
        //call AE process method and serialize
        //the CAS which will be sent with the
        //response.
        if (payload == XCAS_PAYLOAD) {
          LOGINFO(FINEST, "AMQListener::handleRequest() XCAS serialization.");
          startDeserialize = apr_time_now();
          XCASDeserializer deserializer;
          deserializer.deserialize(memIS, *iv_pCas); 
          startAnnotatorProcess=apr_time_now();
          timeToDeserializeCAS = startAnnotatorProcess-startDeserialize;

          iv_pEngine->process(*iv_pCas);
          startSerialize=apr_time_now();
          timeInAnalytic = startSerialize - startAnnotatorProcess;

          XCASWriter xcaswriter(*iv_pCas, true);
          xcaswriter.write(xmlstr);
          timeToSerializeCAS = apr_time_now() - startSerialize;
        } else if (payload == XMI_PAYLOAD) {
          //deserialize incoming xmi CAS data.
          LOGINFO(FINEST, "AMQListener::handleRequest() XMI serialization.");
          
          startDeserialize = apr_time_now();
          XmiSerializationSharedData sharedData;
          XmiDeserializer deserializer;
          deserializer.deserialize(memIS,*iv_pCas,sharedData);
          startAnnotatorProcess=apr_time_now();
          timeToDeserializeCAS = startAnnotatorProcess-startDeserialize;
          LOGINFO(FINEST, "AMQListener::handleRequest() calling process.");
          iv_pEngine->process(*iv_pCas);
          startSerialize=apr_time_now();
          timeInAnalytic = startSerialize - startAnnotatorProcess;

          //serialize CAS 
          LOGINFO(FINEST, "AMQListener::handleRequest() calling serialize.");
          XmiWriter xmiwriter(*iv_pCas, true, &sharedData);
          xmiwriter.write(xmlstr);
          //cout << "SERIALIZED CAS " << xmlstr.str() << endl;
          timeToSerializeCAS = apr_time_now() - startSerialize;
          LOGINFO(FINEST, "AMQListener::handleRequest() done processing CAS.");
        }
        //done with this CAS.
        iv_pCas->reset(); 
        //record end time
        endTime = apr_time_now();
        //send reply
        LOGINFO(FINER,"AnalysisEngine::process() completed successfully. Sending reply.");
        sendResponse(textMessage, timeToDeserializeCAS,
                      timeToSerializeCAS, timeInAnalytic,
                      timeIdle, endTime-startTime,
                      xmlstr.str(),false);
        endTime=apr_time_now();
        iv_pMonitor->processingComplete(iv_id, command,true,endTime-startTime,
          timeToDeserializeCAS, timeInAnalytic, timeToSerializeCAS,
          endTime-startSendResponse);
        LOGINFO(FINE,"Process CAS finished.");		
      } else if (command ==  GET_META_COMMAND ) { //get Meta 	 
        LOGINFO(FINE, "Process getMeta request start.");
        endTime = apr_time_now();
        startSendResponse = apr_time_now();
        sendResponse(textMessage, timeToDeserializeCAS,
                  timeToSerializeCAS, timeInAnalytic,
                  timeIdle, endTime-startTime,this->iv_aeDescriptor,false);			
        endTime=apr_time_now();
        //record timing
        iv_pMonitor->processingComplete(iv_id, command,true,endTime-startTime,0,0,0,endTime-startSendResponse);
        LOGINFO(FINE,"Process getMeta request finished.");			
      } else if (command ==  CPC_COMMAND ) { //CPC       			
        LOGINFO(FINE, "Processing CollectionProcessComplete request start");				
        iv_pEngine->collectionProcessComplete();
        endTime = apr_time_now();
        startSendResponse = apr_time_now();
        sendResponse(textMessage, 0,
                    0, timeInAnalytic,
                    timeIdle,endTime-startTime,"CPC completed.",false);
        endTime=apr_time_now();
        iv_pMonitor->processingComplete(iv_id, command,true,endTime-startTime,0,0,0,endTime-startSendResponse);
        LOGINFO(FINE, "Processing CollectionProcessComplete request finished.");	
      } 
    } catch (XMLException& e) {
      stringstream str;
      str << "AMQListener::handleRequest XMLException." << e.getMessage();
      LOGERROR(str.str());
      endTime = apr_time_now();
      startSendResponse = apr_time_now();
      sendResponse(textMessage, timeToDeserializeCAS,
                    timeToSerializeCAS, timeInAnalytic,
                    timeIdle, endTime-startTime,str.str(),true);
      endTime = apr_time_now();
      iv_pMonitor->processingComplete(iv_id, command,false,endTime-startTime,0,0,0,endTime-startSendResponse);
    } catch (uima::Exception e) {
      LOGERROR("AMQListener::handleRequest UIMA Exception " + e.asString());
      endTime = apr_time_now();
      startSendResponse = apr_time_now();
      sendResponse(textMessage, timeToDeserializeCAS,
                      timeToSerializeCAS, timeInAnalytic,
                      timeIdle,endTime-startTime,e.asString(),true);
      endTime = apr_time_now();
      iv_pMonitor->processingComplete(iv_id, command,false,endTime-startTime,0,0,0,endTime-startSendResponse);
    }	catch(...) {
      LOGERROR("AMQListener::handleRequest Unknown exception ");
      //TODO: log / shurdown ?}   
      endTime = apr_time_now(); 
      iv_pMonitor->processingComplete(iv_id, command,false,endTime-startTime,0,0,0,endTime-startTime);

    }
  }

  void AMQListener::sendResponse(const TextMessage * request, 
                               apr_time_t timeToDeserialize,
                               apr_time_t timeToSerialize, 
                               apr_time_t timeInAnalytic, 
                               apr_time_t idleTime,
                               apr_time_t elapsedTime, 
                               string msgContent, bool isExceptionMsg) {

     //TODO retry
     string serverURI;
     AMQConnection * replyTo=NULL;
     TextMessage * reply=NULL;

     try {
       const Destination * cmsReplyTo = request->getCMSReplyTo();
       if (cmsReplyTo !=  NULL)  {
         reply = this->iv_pConnection->getTextMessage();
       } else {
         LOGINFO(2, "AMQListener::sendResponse() start");
         if (!request->propertyExists("ServerURI")  ) {
           LOGERROR("AMQListener::sendResponse() ServerURI header property does not exist.");
           return;
         }
         if (!request->propertyExists("MessageFrom")  ) {
           LOGERROR("AMQListener::sendResponse() MessageFrom header property does not exist.");
           return;
         }

         //get handle to a connection
         string tmp = request->getStringProperty("ServerURI");
         LOGINFO(FINEST,"replyTo ServerURI: " +  tmp);

         //special handling when protocol is http.
         //HTTP protocol not supported by ActiveMQ C++ client.
         //replace the http broker URL with the broker URL of
         //the queue this listener is attached to.
         if (tmp.find("http:") != string::npos) { 
           serverURI=this->iv_brokerURL;
           LOGINFO(FINER,"HTTP reply address: " + tmp);
         } else {  //send reply via tcp
           size_t begpos = tmp.find("tcp:",0);
           tmp = tmp.substr(begpos);
           size_t endpos = tmp.find_first_of(",");

           if (begpos == string::npos) { 
             LOGERROR("AMQListener::sendResponse() Could not find tcp URL in ServerURI header property.");	 
             return;
           }
           if (endpos == string::npos) {
             serverURI = tmp;
           } else {
             serverURI = tmp.substr(0, endpos);
           }
         }
         LOGINFO(FINER,"ReplyTo BrokerURL " + serverURI);

         //look up the endpoint connection
         replyTo = this->iv_replyToConnections.getConnection(serverURI);
         if (replyTo == NULL) {
           LOGERROR("Could not get a connection to " + serverURI);
           return;
         }
         //get message object
         reply = replyTo->getTextMessage();
       }

       if (reply == NULL) {
         LOGERROR("AMQListener::sendResponse() invalid textMessage object " );
         return;
       }

       //construct the reply message
       reply->setStringProperty("MessageFrom", this->iv_pConnection->getInputQueueName() );
       reply->setStringProperty("ServerURI", this->iv_brokerURL);

       reply->setIntProperty("MessageType",RESPONSE);
       reply->setLongProperty("TimeInService", elapsedTime*1000);
       reply->setLongProperty("IdleTime", idleTime*1000);
       if (request->propertyExists("CasReference")  ) {
         reply->setStringProperty("CasReference", request->getStringProperty("CasReference"));
       }
       if (request->propertyExists("Command")  ) {
         reply->setIntProperty("Command", request->getIntProperty("Command"));
         if (request->getIntProperty("Command") == PROCESS_CAS_COMMAND) {
           reply->setLongProperty("TimeToSerializeCAS", timeToSerialize*1000);
           reply->setLongProperty("TimeToDeserializeCAS", timeToDeserialize*1000);
           reply->setLongProperty("TimeInAnalytic", timeInAnalytic*1000);
           reply->setLongProperty("TimeInProcessCAS", timeInAnalytic*1000);
         }
       }
       if (isExceptionMsg) {
         reply->setIntProperty("Payload",EXC_PAYLOAD);
       } else {
         if (request->propertyExists("Payload") ) {
           reply->setIntProperty("Payload",request->getIntProperty("Payload"));
         } else {
           reply->setIntProperty("Payload",NO_PAYLOAD);
         }
       }
       //cargo
       reply->setText(msgContent);	
       //log the reply message content
       stringstream str;
       str << "Sending Reply Command: " << reply->getIntProperty("Command") << " MessageType: " << reply->getIntProperty("MessageType") << " ";
       if (reply->propertyExists("CasReference") ) {
         str << "CasReference: " << reply->getStringProperty("CasReference");
       }
       LOGINFO(INFO,str.str());

       if (cmsReplyTo != NULL) {
         str << " to " << ((Queue*)cmsReplyTo)->getQueueName();
       } else {
         str << " to " << request->getStringProperty("MessageFrom") 
           << " at " << serverURI;
       }
       str << " Message text: " << msgContent;
       //std::cout << "****Reply" << msgContent << std::endl;
       LOGINFO(FINEST,"PRINT Reply message:\n" + str.str());	

       //send
       if (cmsReplyTo != NULL) {
         //cout << "cmsReplyTo=" << cmsReplyTo->toProviderString() << endl;
         iv_pConnection->sendMessage(cmsReplyTo);
       } else {
         replyTo->sendMessage(request->getStringProperty("MessageFrom"));
       }
       LOGINFO(FINER,"AMQListener::sendResponse DONE");

     } catch (CMSException& ex ) {
       LOGERROR("AMQListener::handleMessage()" +  ex.getMessage());
     } catch (...)  { 
       LOGERROR("AMQListener::handleRequest() UnExpected error sending reply.");
     }
}

//===================================================
//AMQAnalysisEngineService
//---------------------------------------------------
  AMQAnalysisEngineService::~AMQAnalysisEngineService() {
    //stop();
    cleanup();
  }

  AMQAnalysisEngineService::AMQAnalysisEngineService(ServiceParameters & desc,
                                                     Monitor * stats,  apr_pool_t * pool)  : 
                                        //iv_pMonitor(stats),
                                        iv_pool(pool),
                                        iv_numInstances(desc.getNumberOfInstances()),
                                        iv_brokerURL(desc.getBrokerURL()),
                                        iv_inputQueueName(desc.getQueueName()),
                                        iv_aeDescriptor(desc.getAEDescriptor()),
                                        iv_prefetchSize(desc.getPrefetchSize()),
                                        iv_initialFSHeapSize(desc.getInitialFSHeapSize()),
                                        iv_vecpConnections(),
                                        iv_vecpAnalysisEngines(),
                                        iv_vecpCas(),
                                        iv_closed(false),
                                        iv_listeners() {
    try {
      iv_pMonitor = stats;
      initialize(desc);
    } catch (CMSException & e) {
      ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
      msg.addParam(e.getMessage());
      ErrorInfo errInfo;
      errInfo.setMessage(msg);
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }
  }

  void AMQAnalysisEngineService::initialize(ServiceParameters & params) {

    try {
      //create connection factory
      LOGINFO(FINEST,"AMQAnalysisEngineService::initialize() Create connection factory");
      this->iv_pConnFact = AMQConnection::createConnectionFactory(params);
	  
      if (iv_pConnFact == NULL) {
        ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
        msg.addParam("AMQAnalysisEngineService::initialize() Failed to create connection factory.");
        ErrorInfo errInfo;
        errInfo.setMessage(msg);
        UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
          UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
          errInfo.getMessage(),
          errInfo.getMessage().getMessageID(),
          ErrorInfo::unrecoverable);
      }
      if (!uima::ResourceManager::hasInstance()) {
        uima::ResourceManager::createInstance("ActiveMQAnalysisEngineService");
      }
      ErrorInfo errInfo;
      icu::UnicodeString ustr(this->iv_aeDescriptor.c_str());
      icu::UnicodeString ufn = ResourceManager::resolveFilename(ustr,ustr);

      //create a AnalysisEngine and CAS for each instance
      for (int i=0; i < iv_numInstances; i++) {
        //create the connection
        AMQConnection * newConnection = new AMQConnection(this->iv_pConnFact, params.getBrokerURL(), this->iv_pMonitor, i);
        if (newConnection == NULL) {
          LOGERROR("AMQAnalysisEngineService::initialize() Could not create ActiveMQ endpoint connection.");
          ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
          msg.addParam("AMQAnalysisEngineService::initialize() Failed to connect to broker.");
          ErrorInfo errInfo;
          errInfo.setMessage(msg);
          UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
            UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
            errInfo.getMessage(),
            errInfo.getMessage().getMessageID(),
            ErrorInfo::unrecoverable);
        }
        /////newConnection->setExceptionListener(this);
        this->iv_vecpConnections.push_back(newConnection);
        //create AE
        AnalysisEngine * pEngine = uima::TextAnalysisEngine::createTextAnalysisEngine((UnicodeStringRef(ufn).asUTF8().c_str()),
          errInfo);
        if (pEngine) {
          LOGINFO(0,"AMQAnalysisEngineService::initialize() AnalysisEngine initialization successful.");
        } else {
          LOGERROR("AMQAnalysisEngineService::initializer() could not create AE" + errInfo.asString());
          ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
          msg.addParam("AMQAnalysisEngineService::initialize() create AE failed. " + errInfo.getMessage().asString() );
          ErrorInfo errInfo;
          errInfo.setErrorId(UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE),
            errInfo.setMessage(msg);
          UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
            UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
            errInfo.getMessage(),
            errInfo.getMessage().getMessageID(),
            ErrorInfo::unrecoverable);
        }

        this->iv_vecpAnalysisEngines.push_back(pEngine);

        //initial FSHeap size
        if (this->iv_initialFSHeapSize > 0) {
          pEngine->getAnnotatorContext().
            assignValue(UIMA_ENGINE_CONFIG_OPTION_FSHEAP_PAGESIZE,this->iv_initialFSHeapSize);
        }

        //create CAS
        CAS * cas = pEngine->newCAS();
        if (cas == NULL) {
          LOGERROR("AMQAnalysisEngineService::initialize() Could not create CAS");
          ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
          msg.addParam("AMQListener::initialize() create CAS failed.");
          ErrorInfo errInfo;
          errInfo.setErrorId(UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE),
            errInfo.setMessage(msg);
          UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
            UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
            errInfo.getMessage(),
            errInfo.getMessage().getMessageID(),
            ErrorInfo::unrecoverable);
        }
        this->iv_vecpCas.push_back(cas);
      
        //cout << __FILE__ << __LINE__ << "AMQAnalysisEngineService::initialize() create listener " << endl;
        //create listeners and register these
        AMQListener * newListener = new AMQListener(i,iv_vecpConnections.at(i),
          iv_vecpAnalysisEngines.at(i), iv_vecpCas.at(i),
          iv_pMonitor);
        if (newListener == NULL) {
          LOGERROR("AMQAnalysisEngineService::initialize() Could not create listener.");
          ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
          msg.addParam("AnalysisEngineService::initialize() Could not create listener.");
          ErrorInfo errInfo;
          errInfo.setMessage(msg);
          UIMA_EXC_THROW_NEW(Uima_runtime_error, 
            UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
            errInfo.getMessage(),
            errInfo.getMessage().getMessageID(),
            ErrorInfo::unrecoverable);
        }
        this->iv_listeners[i] = newListener;
        //cout << __FILE__ << __LINE__ << "AMQAnalysisEngineService::initialize() create message consumer " << endl;
        //create MessageConsumer session and register Listener       
        this->iv_vecpConnections.at(i)->createMessageConsumer(iv_inputQueueName,annotator_selector, params.getPrefetchSize());  
     } 
     
     iv_pMonitor->setNumberOfInstances(iv_numInstances);
     //cout <<  __FILE__ << __LINE__ << "AMQAnalysisEngineService::initialize() setup getmeta " << endl;
      //Fast GetMeta
      //create connection 
     LOGINFO(FINEST, "AMQAnalysisEngineService::initialize() Setup GETMETA instance.");
     iv_pgetMetaConnection = new AMQConnection(this->iv_pConnFact, params.getBrokerURL(), this->iv_pMonitor, iv_numInstances);

     if (iv_pgetMetaConnection == NULL) {
          LOGERROR("AMQAnalysisEngineService::initialize() Could not create fast getmeta ActiveMQ endpoint connection.");
          ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
          msg.addParam("AMQAnalysisEngineService::initialize() Failed to connect to broker.");
          ErrorInfo errInfo;
          errInfo.setMessage(msg);
          UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
            UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
            errInfo.getMessage(),
            errInfo.getMessage().getMessageID(),
            ErrorInfo::unrecoverable);
     }

     //cout <<  __FILE__ << __LINE__ << "AMQAnalysisEngineService::initialize() setup getmeta listener" << endl;
     //create a MessageListener MessageConsumer to handle getMeta requests only.
     AMQListener * newListener = new AMQListener(iv_numInstances,iv_pgetMetaConnection,
          iv_vecpAnalysisEngines.at(0), 
          this->iv_pMonitor);
     if (newListener == NULL) {
          LOGERROR("AMQAnalysisEngineService::initialize() Could not create Fast getMeta listener.");
          ErrorMessage msg(UIMA_MSG_ID_LOG_ERROR);
          msg.addParam("AMQAnalysisEngineService::initialize() Could not create listener.");
          ErrorInfo errInfo;
          errInfo.setMessage(msg);
          UIMA_EXC_THROW_NEW(Uima_runtime_error, 
            UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
            errInfo.getMessage(),
            errInfo.getMessage().getMessageID(),
            ErrorInfo::unrecoverable);
      }
      this->iv_listeners[this->iv_numInstances] = newListener; 
      iv_pgetMetaConnection->createMessageConsumer(iv_inputQueueName, 
                      getmeta_selector,0); 
      this->iv_pMonitor->setGetMetaListenerId(iv_numInstances);
      //cout <<  __FILE__ << __LINE__ << "AMQAnalysisEngineService::initialize() done" << endl;
    } catch (uima::Exception e) {
      cout << __FILE__ << __LINE__ <<  "AMQAnalysisEngineService::initialize() failed " << e.getErrorInfo().asString() << endl;
      LOGERROR("AMQAnalysisEngineService::initialize() " + e.asString());
      throw e;
    }
  }

  void AMQAnalysisEngineService::setTraceLevel(int level) {
    if (level < 0) {
    uimacpp_ee_tracelevel=NONE;
    } else if (level == 0) {
      uimacpp_ee_tracelevel = INFO;
    } else if (level == 1) {
      uimacpp_ee_tracelevel = FINE;
    } else if (level == 2) {
      uimacpp_ee_tracelevel = FINER;
    } else if (level > 2) {
      uimacpp_ee_tracelevel = FINEST;
    } else {
      uimacpp_ee_tracelevel = INFO;
    }
    cout << "tracelevel=" << uimacpp_ee_tracelevel << endl;
  }

  void AMQAnalysisEngineService::startProcessingThreads() {
    LOGINFO(FINER,"AMQAnalysisEngineService::start() create listener threads.");
    //create the listener threads
    thd_attr=0;
    apr_status_t rv;
    rv = apr_threadattr_create(&thd_attr, iv_pool);
    assert(rv == APR_SUCCESS);
    map<int, AMQListener*>::iterator ite; 
    int i=0;
    for (ite= iv_listeners.begin();ite !=  iv_listeners.end();ite++) {
      apr_thread_t *thread=0;
      rv = apr_thread_create(&thread, thd_attr, handleMessages, ite->second, iv_pool);
      assert(rv == APR_SUCCESS);
      iv_listenerThreads.push_back(thread);
    }
    this->iv_pMonitor->setStartTime();
}


void AMQAnalysisEngineService::start() {
  
  if (iv_pgetMetaConnection != NULL) {
    cerr << "Startinging GetMetaData instance" << endl;
    this->iv_pgetMetaConnection->start();
  }
  for (size_t i=0; i < iv_vecpConnections.size(); i++) {
    cerr << "Starting Annotator instance " << i << endl;
    AMQConnection * connection = iv_vecpConnections.at(i);
    if (connection != NULL) {
      connection->start();
    } else {
      LOGERROR("AMQAnalysisEngineService::start() Connection object is NULL.");
      ErrorInfo errInfo;
      errInfo.setMessage(ErrorMessage(UIMA_MSG_ID_LOG_ERROR, "connection object is NULL."));
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }
  } 
}	

void AMQAnalysisEngineService::shutdown() {
  //LOGWARN("AMQAnalysisEngineService::shutdown()");

  stop();

  cout << "AMQAnalysisEngineService::shutdown() going to terminate threads " << endl;;
  //terminate the threads
  apr_status_t rv;
  for (size_t i=0; i < this->iv_listenerThreads.size(); i++) {
    //cout << "wait for thread " << i << " to end " << endl;
    this->iv_listeners[i]->stopProcessing();
    if (!iv_listeners[i]->isReconnecting()) {
      apr_thread_join(&rv, this->iv_listenerThreads.at(i));
    }
  }
  
  cout << "AMQAnalysisEngineService::shutdown stopped all connection" << endl;
  cleanup();
  cout << "AMQAnalysisEngineService::shutdown shutdown done" << endl;
}

int AMQAnalysisEngineService::stop() {
  
  //TODO: let listeners finish processing first
  //stop messages notification

  if (iv_pgetMetaConnection != NULL) {
    cerr << "Stopping GetMetaData instance" << endl;
    this->iv_pgetMetaConnection->stop();
  }
  for (size_t i=0; i < iv_vecpConnections.size(); i++) {
    cerr << "Stopping Annotator instance " << i << endl;
    AMQConnection * connection = iv_vecpConnections.at(i);
    if (connection != NULL) {
      connection->stop();
    } else {
      LOGERROR("AMQAnalysisEngineService::stop() Connection object is NULL.");
      ErrorInfo errInfo;
      errInfo.setMessage(ErrorMessage(UIMA_MSG_ID_LOG_ERROR, "connection object is NULL."));
      UIMA_EXC_THROW_NEW(uima::Uima_runtime_error, 
        UIMA_ERR_RESMGR_COULD_NOT_INITIALIZE_RESOURCE,
        errInfo.getMessage(),
        errInfo.getMessage().getMessageID(),
        ErrorInfo::unrecoverable);
    }
  } 
  return 0;
}	

void  AMQAnalysisEngineService::quiesceAndStop() {
  
  quiesce();
  
  //shutdown worker threads.
   cout << "AMQAnalysisEngineService::quiesceAndStop() going to terminate threads " << endl;;
  if (this->iv_pMonitor->getQuiesceAndStop() ) {
    for (size_t i=0; i < this->iv_listenerThreads.size(); i++) {
      //cout << "wait for thread " << i << " to end " << endl;
      this->iv_listeners[i]->stopProcessing();
      apr_thread_join(&rv, this->iv_listenerThreads.at(i));
    }
  }

  cleanup();
}

void  AMQAnalysisEngineService::quiesce() {
  //stop connections - does not work
  stop();

  //check whether processing threads are finished.
  bool allfinished = false;
  while (!allfinished) {
    allfinished = true;
    map<int, AMQListener*>::iterator ite; 
    for (ite= iv_listeners.begin();ite !=  iv_listeners.end();ite++) {
      if (ite->second->isBusy()) { 
        allfinished  = false;
        break;
      }
    }
    //Thread::sleep(1000);
	apr_sleep(1000000);
  }
}

  void AMQAnalysisEngineService::cleanup() {
    // Destroy resources.
    try { 
      if (iv_closed) {
         return;
      }
      if (iv_pgetMetaConnection != NULL) {
        delete this->iv_pgetMetaConnection;
        this->iv_pgetMetaConnection = 0;
      }

      for (size_t i=0; i < iv_vecpConnections.size(); i++) {
        //cout << "deleting consumerConnection " << i << endl;	
        if (iv_vecpConnections.at(i) != NULL) {
          iv_vecpConnections.at(i)->stop();
          delete iv_vecpConnections.at(i);
        }
      }  
      
      for (size_t i=0; i < iv_vecpAnalysisEngines.size(); i++) {
         delete iv_vecpAnalysisEngines.at(i);
      }

      for (size_t i=0; i < iv_vecpCas.size(); i++) {
         delete iv_vecpCas.at(i);
      }

      iv_vecpAnalysisEngines.clear();
      iv_vecpCas.clear();
      iv_vecpConnections.clear();

      map<int, AMQListener*>::iterator ite; 
      for (ite= iv_listeners.begin();ite !=  iv_listeners.end();ite++) {
        delete ite->second;
      }

      iv_listeners.clear();

      if (iv_pConnFact != NULL) {
        delete iv_pConnFact;
        iv_pConnFact = NULL;
      }
      iv_closed = true;

    } catch (CMSException& e) {
      cerr << "AMQAnalysisEngineService::cleanup() "  << e.getMessage() << endl;;
    }  
  }           


//===================================================
//CommonUtils
//---------------------------------------------------
void CommonUtils::logError(string msg) {
    this->iv_pMonitor->logError(msg);
    //cerr << "ERROR: " << msg << endl;
  }
void CommonUtils::logWarning(string msg) {
    this->iv_pMonitor->logWarning(msg);
    //cout << "WARN: " << msg << endl;
  }
void CommonUtils::logMessage(string msg) {
  cout << "INFO: " << msg << endl;
    this->iv_pMonitor->logMessage(msg);
    //cout << "INFO: " << msg << endl;
  }

