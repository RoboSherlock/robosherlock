/** \file sofastream.hpp .
-----------------------------------------------------------------------------



\brief  Contains implementation of uima::CASPool

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

-------------------------------------------------------------------------- */


// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------


#include <uima/caspool.hpp>
#include <uima/err_ids.h>
#include <uima/msg.h>

using namespace std;
namespace uima {


  UIMA_EXC_CLASSIMPLEMENT(CASPoolException, uima::Exception);
  //------------------------------------------------------------
  //
  // CASPool implementation
  //
  //------------------------------------------------------------

  CASPool::CASPool(const AnalysisEngineDescription & taeSpec,
                   size_t numInstances)
      :iv_vecAllInstances(),
      iv_vecFreeInstances(),
      iv_pCasDef(NULL),
      iv_numInstances(numInstances) {

    iv_pCasDef = uima::internal::CASDefinition::createCASDefinition(taeSpec);

    if (iv_pCasDef == NULL) {
      UIMA_EXC_THROW_NEW(CASPoolException,
                         UIMA_ERR_CASPOOL_CREATE_CASDEFINITION,
                         UIMA_MSG_ID_EXC_CREATE_CASPOOL,
                         UIMA_MSG_ID_EXC_CREATE_CASPOOL,
                         ErrorInfo::unrecoverable);
    }

    for (size_t i=0; i < numInstances; i++) {
      CAS  * pCas = uima::internal::CASImpl::createCASImpl(*iv_pCasDef,false);
      if (pCas == NULL) {
        UIMA_EXC_THROW_NEW(CASPoolException,
                           UIMA_ERR_CASPOOL_CREATE_CAS,
                           UIMA_MSG_ID_EXC_CREATE_CASPOOL,
                           UIMA_MSG_ID_EXC_CREATE_CASPOOL,
                           ErrorInfo::unrecoverable);
      }
      iv_vecAllInstances.push_back((CAS *)pCas->getInitialView());
      iv_vecFreeInstances.push_back((CAS *)pCas->getInitialView());
    }
  }


  CASPool::~CASPool() {
    for (size_t i=0; i < iv_vecAllInstances.size(); i++) {
      delete iv_vecAllInstances[i];
    }
    if (iv_pCasDef)
      delete iv_pCasDef;
  }


  CAS &  CASPool::getCAS() {

    CAS * pCas = NULL;
    vector<CAS *>::iterator iter;

    for (iter=iv_vecFreeInstances.begin(); iter != iv_vecFreeInstances.end() ;iter++) {
      pCas = *iter;
      if (pCas) {
        iv_vecFreeInstances.erase(iter);
        break;
      }
    }
    if (pCas) {    //free cas
      return *pCas;
    } else {              //for now throw exception if there
      //are no available CASs.
      UIMA_EXC_THROW_NEW(CASPoolException,
                         UIMA_ERR_CASPOOL_GET_CAS,
                         UIMA_MSG_ID_EXC_NO_FREE_CAS,
                         UIMA_MSG_ID_EXC_NO_FREE_CAS,
                         ErrorInfo::unrecoverable);
    }

  }

  void CASPool::releaseCAS(CAS & aCas) {

    aCas.reset();
    iv_vecFreeInstances.push_back(&aCas);
    return;
  }

} //namespace

