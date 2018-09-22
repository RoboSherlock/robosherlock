/** \file sofastreamhandler.hpp .
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

   \brief Functions that a stream handler must implement in order
   to support remote Sofa access

-----------------------------------------------------------------------------

   Description:
   This specifies the functions that a stream handler
   must implement to support remote Sofa access in UIMACPP.
*/

#ifndef __UIMA_SOFASTREAMHANDLER_HPP__
#define __UIMA_SOFASTREAMHANDLER_HPP__

#include <uima/types.h>

namespace uima {
  /**
   * The class <TT>SofaStreamHandler</TT> defines the API methods that must
   * be implemented to support reading custom URI schemes for reading
   */
  class  SofaStreamHandler {
  public:
    /** Constructor */
    SofaStreamHandler(void);
    /** Destructor */
    virtual     ~SofaStreamHandler() {
      ;
    }
    /**
     * Open a stream to this URL.
     *  
     */
    virtual void openStream(const char * uriString)=0;

    /**
     * Open a stream to this URL and specify the minimum size of 
     * the internal buffer.
     *
     */
    virtual void openStream(const char * uriString, size_t minimumBufferSize)=0;

    /**
     * Gets the total size of the data in bytes.
     */
    virtual INT64 getTotalStreamSize()=0;

    /**
     * Gets the size of the internal buffer.
     */
    virtual size_t getBufferSize()=0;

    /**
     * Gets number of bytes available to read.
     */
    virtual INT64 howManyAvailable()=0;

    /**
     * Read the specified number of bytes from the current position.
     * Advance the current position by the number of bytes read.
     * This call blocks till read request is satisfied or EOF is 
     * reached.  
     * @param numBytes the number of bytes to read
     * @param pBuffer buffer into which the bytes are to be copied.
     * @returns number of bytes read of EOF (-1). This may be less than
     * the number of bytes requested.
     */
    virtual INT64 getNext(size_t numBytes, void * pBuffer)=0;

    /**
     *
     * Sets the position within the current stream.
     *
     * @param offset - number of bytes from origin
     * @param origin is one of the following (taken from lseek spec):
     *          If SEEK_SET, the position is set to offset bytes.
     *          If SEEK_CUR, the position is  set  to  its
     *              current location plus offset bytes.
     *          If SEEK_END,  the position is set to the total length
     *              plus offset bytes.
     *          These constants are defined in stdio.h 
     * @return 0 indicates success
     *         -1 EOF
     */
    virtual int seek (INT64 offset,
                      int origin)=0;

    /**
    * closeStream
    * close the stream. Delete the internal buffer. 
    */
    virtual void closeStream()=0;

    /**
     * getDataPointer
     * This returns a pointer to the data in memory.
     * A valid pointer is returned only if the entire 
     * stream data is available in memory, that is getTotalLength()
     * is equal to howManyAvailable().  Otherwise return NULL.
     *
           * @return pointer to data or NULL.
     */
    virtual void * getDataPointer();

  }
  ; /* SofaStreamHandler */


  inline SofaStreamHandler::SofaStreamHandler(void) {
    return;
  }

  inline void * SofaStreamHandler::getDataPointer() {
    return NULL;
  }


  /** MAKE_HANDLER macro must be used by the custom handler writers
      to export an entry point to the handler  */
#define MAKE_HANDLER(classHandler) extern "C" UIMA_ANNOTATOR_LINK_IMPORTSPEC SofaStreamHandler * makeHandler() { return new classHandler; }

  extern "C" UIMA_ANNOTATOR_LINK_IMPORTSPEC SofaStreamHandler * makeHandler();

  typedef SofaStreamHandler*  (* TyMakeStreamHandler) (void);
}
#endif
