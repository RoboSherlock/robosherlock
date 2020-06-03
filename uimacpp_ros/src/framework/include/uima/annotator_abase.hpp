/** \file annotator_abase.hpp .
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

    \brief  Contains Annotator a abstract base class four the Annotator object

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */

#ifndef UIMA_ANNOTATOR_ABASE_HPP
#define UIMA_ANNOTATOR_ABASE_HPP

/* ----------------------------------------------------------------------- */
/*       Interface dependencies                                            */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
////#include "uima/annotator_api.h"
#include <uima/cas.hpp>
#include <iostream>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

namespace uima {
  class AnnotatorContext;              // forward declaration
  class ResultSpecification;
  class TCAS;
}

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  /**
   * The class <TT>Annotator</TT> defines the API methods an
   * annotator must implement.
   */
  class  Annotator { //lint -esym(1512, Annotator ) destructor for base class Annotator is not virtual
  public:
    Annotator(void);
    virtual                 ~Annotator()                               {
      ;
    }

    /**
     * set the AnnotatorContext.
     *
     * This function need not be called by the annotator code.
     */
    void                    setAnnotatorContext(AnnotatorContext & rclAnnotatorContext);

    /** get the annotator context of this annotator. */
    AnnotatorContext &      getAnnotatorContext(void);

    /** @name Annotator Processing Functions */
    /*@{*/

    /** Call the annotator to initialize itself based on an AnnotatorContext. */
    virtual
    TyErrorId               initialize(AnnotatorContext & rclAnnotatorContext); // = 0;

    /**
     * Call the annotator to cache type/feature objects used in subsequent process() calls.
     */
    virtual
    TyErrorId               typeSystemInit(TypeSystem const &); // = 0;

    /** Call the annotator to deinitialize itself.
     */
    virtual
    TyErrorId               destroy(void); // = 0;

    /** Call the annotator to reconfigure itself.(optional method)
     */
    virtual TyErrorId               reconfigure();

    /** Call the annotator to perform its task with
     * a ResultSpecification indicating what (sub)set of the capabilities
     * of the annotator are actually needed. This method should only be 
     * called with an instance of Annotator.  It should not be called with
     * an instance of a TextAnnotator as the TextAnnotator works with a TCAS
     * view of the CAS.
     */
    virtual
    TyErrorId               process(CAS & cas, ResultSpecification const & crResultSpecification); // = 0;


    /** Call the annotator to perform a batchProcessComplete operation. */
    virtual TyErrorId               batchProcessComplete();

    /** Call the annotator to perform a collectionProcessComplete operation. */
    virtual TyErrorId               collectionProcessComplete();

    /** Call the annotator to perform a hasNext operation to determine whether
     *  this analysis component will be returning new CASs. */
    virtual bool     hasNext();

    /** Call the annotator to retrieve the next CAS if hasNext() has returned true. */
    virtual CAS &     next();

    /** Call the annotator to get the number of CAS instances it will use
     * concurrently in order to determine the size of the CASPool for this component. */
    virtual int     getCasInstancesRequired();

    /*@}*/
  private:
    AnnotatorContext *      iv_pclAnnotatorContext;
    TypeSystem const * iv_typeSystem;
  }
  ; /* Annotator */


  /** @deprecated
   * A TextAnnotator is an Annotator which can process text documents. 
   * In other words, it operates on a TCAS.
   */
  class TextAnnotator : public Annotator {
  public:
    TextAnnotator();
    virtual ~TextAnnotator() { }

    virtual TyErrorId process(CAS & cas, ResultSpecification const & crResultSpecification);
  };




  /* ----------------------------------------------------------------------- */
  /*       Implementation                                                    */
  /* ----------------------------------------------------------------------- */
  inline Annotator::Annotator(void) :
      iv_pclAnnotatorContext(NULL),
      iv_typeSystem(NULL) {
    ;
  }

  inline TyErrorId Annotator::initialize(AnnotatorContext & rclAnnotatorContext) {
    return (TyErrorId)UIMA_ERR_NONE;
  }

  inline TyErrorId Annotator::typeSystemInit(TypeSystem const &) {
    return (TyErrorId)UIMA_ERR_NONE;
  }

  inline TyErrorId Annotator::destroy() {
    return (TyErrorId)UIMA_ERR_NONE;
  }

  inline TyErrorId Annotator::reconfigure() {
    std::cout << "\n\nCalling base reconfigure()\n\n";
    return (TyErrorId)UIMA_ERR_NONE;
  }

  inline TyErrorId Annotator::process(CAS & cas, ResultSpecification const & crResultSpecification) {
    return (TyErrorId) UIMA_ERR_NONE;
  }

  inline TyErrorId Annotator::batchProcessComplete() {
    return (TyErrorId)UIMA_ERR_NONE;
  }

  inline TyErrorId Annotator::collectionProcessComplete() {
    return (TyErrorId)UIMA_ERR_NONE;
  }

  inline bool Annotator::hasNext() {
    return false;
  }

  inline CAS & Annotator::next() {

    UIMA_EXC_THROW_NEW(ExcInvalidRequest,
                       UIMA_ERR_NOT_YET_IMPLEMENTED,
                       UIMA_MSG_ID_EXC_INVALID_CALL_TO_NEXT,
                       UIMA_MSG_ID_EXC_INVALID_CALL_TO_NEXT,
                       ErrorInfo::unrecoverable);
  }

  inline int Annotator::getCasInstancesRequired() {
    return 0;
  }

  inline void Annotator::setAnnotatorContext(AnnotatorContext & rclAnnotatorContext) {
    iv_pclAnnotatorContext = &rclAnnotatorContext;
  }

  inline AnnotatorContext & Annotator::getAnnotatorContext(void) {
    assertWithMsg(EXISTS(iv_pclAnnotatorContext), "Annotator::init() Forgot to call 'setAnnotatorContext()'");
    return(*iv_pclAnnotatorContext);
  }


/////////////////////////////////////////

  inline TextAnnotator::TextAnnotator()
      : Annotator() {}

  inline TyErrorId TextAnnotator::process(CAS & cas, ResultSpecification const & crResultSpecification) {
//    CAS & tcas = CAS::promoteCAS(cas);
    return process(cas, crResultSpecification);
  }

  /** MAKE_AE macro must be used by UIMA component writers
      to export an entry point to the component */
#define MAKE_AE(classAE) extern "C" UIMA_ANNOTATOR_LINK_IMPORTSPEC Annotator * makeAE() { return new classAE; }

  extern "C" UIMA_ANNOTATOR_LINK_IMPORTSPEC Annotator * makeAE();


}

#endif /* UIMA_ANNOTATOR_ABASE_HPP */

/* <EOF> */

