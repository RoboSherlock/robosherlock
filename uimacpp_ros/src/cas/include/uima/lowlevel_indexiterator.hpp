#ifndef UIMA_LOWLEVEL_INDEXITERATOR_HPP
#define UIMA_LOWLEVEL_INDEXITERATOR_HPP
/** \file lowlevel_indexiterator.hpp .
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

   Description:

-----------------------------------------------------------------------------


-------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <uima/lowlevel_typedefs.hpp>
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
  namespace lowlevel {

    /**
     * Abstract base class for all lowlevel iterators over indexes.
     */
    class UIMA_LINK_IMPORTSPEC IndexIterator {
    public:

      virtual ~IndexIterator() { }
      /**
       * sets the iterator to point to the first element.
       */
      virtual void moveToFirst() = 0;

      /**
       * advance the iterator.
       * If <code>isValid()</code> is false, behaviour is undefined.
       */
      virtual void moveToNext() = 0;

      virtual void moveToPrevious() = 0;

      virtual void moveToLast() = 0;

      /**
       * dereference the iterator.
       * If <code>isValid()</code> is false, behaviour is undefined.
       */
      virtual TyFS get() const = 0;

      /**
       * get TyFSType of the iterator.
       * If <code>isValid()</code> is false, behaviour is undefined.
       */
      virtual TyFS getTyFSType() const = 0;

      /**
       * Check if the iterator points to a valid element.
       */
      virtual bool isValid() const = 0;

      /**
       * create a copy of this iterator.
       */
      virtual IndexIterator* clone() const = 0;

      /**
       * sets the iterator to the position of <code>fs</code>.
       * @return true if <code>fs</code> is a feature structure contained in the index this
       *         iterator iterates over and resetting was successful, false  otherwise.
       */
      virtual bool moveTo(TyFS fs) = 0;
    };

  }
}
/* ----------------------------------------------------------------------- */
/*       Implementation                                                    */
/* ----------------------------------------------------------------------- */


/* ----------------------------------------------------------------------- */


#endif

