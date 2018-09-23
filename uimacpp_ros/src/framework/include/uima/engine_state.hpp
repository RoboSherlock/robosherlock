/** \file engine_state.hpp .
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

    \brief  Contains EngineState a class to maintain the state of the egine, i.e. what is next

   Description:

-----------------------------------------------------------------------------


   6/18/1999  Initial creation

-------------------------------------------------------------------------- */

#ifndef UIMA_ENGINE_STATE_HPP
#define UIMA_ENGINE_STATE_HPP

/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/assertmsg.h>

/* ----------------------------------------------------------------------- */
/*       Constants                                                         */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */

namespace uima {
  /**
   * The class <TT>EngineState</TT> maintains the state of the
   * engine. For example, its location and what the user may do next.
   * \code
     \endcode
   * @see
   */
  class UIMA_LINK_IMPORTSPEC EngineState {
  public:
    /** @name Types */
    /*@{*/
    enum EnEngineState {
      enEngineState_readyForInit      = 0,  /* Note: all states must be ordered */
      enEngineState_readyForProcessOrReconfigOrDeinit = 1,
      enEngineState_readyForDeletion = 2
    } ;
    /*@}*/
    /** @name Constructors */
    /*@{*/
    /** Create a new state object with initial value: enEngineState_ctor */
    EngineState(void);
    /*@}*/
    /** @name Properties */
    /*@{*/
    /** Return the current state */
    EnEngineState           getState(void) const                         {
      return(iv_enState);
    }
    /*@}*/
    /** @name Match operations */
    /*@{*/
    /** Return TRUE, if this state matches the specified state. */
    bool                       operator==(EnEngineState enState) const   {
      return(iv_enState == enState);
    }
    /** Return TRUE, if this state does not match the specified state. */
    bool                       operator!=(EnEngineState enState) const   {
      return(iv_enState != enState);
    }
    /** Return TRUE, if this state is higher than the specified state. */
    bool                       operator>(EnEngineState enState) const    {
      return(iv_enState > enState);
    }
    /** Return TRUE, if this state is higher or equal to the specified state. */
    bool                       operator>=(EnEngineState enState) const   {
      return(iv_enState >= enState);
    }
    /** Return TRUE, if this state is lower than the specified state. */
    bool                       operator<(EnEngineState enState) const    {
      return(iv_enState < enState);
    }
    /** Return TRUE, if this state is lower or equal to the specified state. */
    bool                       operator<=(EnEngineState enState) const   {
      return(iv_enState <= enState);
    }
    /** Return TRUE, if this state indicates that no documents have been added so far. For example,
        the document buffer is empty. */
    bool                       isReadyToAddDocParts(void) const;
    /** Return TRUE, if this state indicates that documents may be added at this time. */
    bool                       isAllowedToAddDocParts(void) const;
    /** Check that this state matches the specified state -
        this function is for debugging purposes only. */
    void                       assertMatch(EnEngineState enState) const;
    /*@}*/
    /** @name Miscellaneous */
    /*@{*/
    /** Set this state to the specified state. */
    void                       setToState(EnEngineState enState);
    /*@}*/
  protected:
    /* --- functions --- */
  private:
    EnEngineState           iv_enState;
    /* --- functions --- */
    /* COPY CONSTRUCTOR NOT SUPPORTED */
    EngineState(const EngineState & ); //lint !e1704
    /* ASSIGNMENT OPERATOR NOT SUPPORTED */
    EngineState & operator=(const EngineState & crclObject);
  }
  ; /* EngineState */
}

/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */

namespace uima {
  inline EngineState::EngineState(void) :
      iv_enState(enEngineState_readyForInit)
      /* ----------------------------------------------------------------------- */
  {
    ;
  }

  inline void EngineState::assertMatch(EnEngineState enState) const
  /* ----------------------------------------------------------------------- */
  {
    assertWithMsg(iv_enState == enState, _TEXT("Engine state is not as expected, Engine and EngineState do not match"));
  }


  inline void EngineState::setToState(EnEngineState enState)
  /* ----------------------------------------------------------------------- */
  {
    iv_enState = enState;
  }

}

/* ----------------------------------------------------------------------- */
#endif /* UIMA_ENGINE_STATE_HPP */

/* <EOF> */

