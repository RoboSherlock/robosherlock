#ifndef UIMA_SOFASTREAM_HPP
#define UIMA_SOFASTREAM_HPP

/** \file sofastream.hpp .
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

    \brief  Contains class uima::SofaDataStream

   Description: The SofaDataStream class provides stream access
                to Sofa data in a Sofa feature strucure in the CAS.
                The Sofa data may be local in the CAS or remote from
                the CAS and reference by a URI conformant string
                set in the SofaURI feature.

                Handlers for the various URI schemes both standard and user-defined
                schemes may be registered by setting the environment variable
                UIMACPP_STREAMHANDLERS as follows:

                 set UIMACPP_STREAMHANDLERS=file:sofastreamhandler_file;aURI:aDLLfilename

                The APIs deliver data in native byte order and expect the
                data read from the source to be in the network byte order.

-----------------------------------------------------------------------------


   10/01/2004  Initial creation

-------------------------------------------------------------------------- */


// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include <uima/pragmas.hpp> //must be first to surpress warnings
#include <uima/err_ids.h>
#include <uima/taemetadata.hpp>
#include <uima/cas.hpp>
#include <uima/dllfile.hpp>
#include <vector>

/* ----------------------------------------------------------------------- */
/*       Forward declarations                                              */
/* ----------------------------------------------------------------------- */
namespace uima {
  class SofaFS;
  class SofaStreamHandler;
}


/* ----------------------------------------------------------------------- */
/*       Types / Classes                                                   */
/* ----------------------------------------------------------------------- */
namespace uima {

#define UIMA_SOFASTREAM_MAKE_HANDLER           _TEXT("makeHandler")

  typedef void * TySofaDataPointer;

  /**           The SofaDataStream class provides stream access
                to Sofa data in a Sofa feature strucure in the CAS.
                The Sofa data may be local in the CAS or remote from 
                the CAS and reference by a URI conformant string
                set in the SofaURI feature. 
                <br><br>
                Handlers for the various URI schemes both standard and user-defined
                schemes may be registered by setting the environment variable
                UIMACPP_STREAMHANDLERS as follows:
                <br>
                <br>
                <code>
                   UIMACPP_STREAMHANDLERS=file:SofaStreamHandlerFile aScheme:aLibrary ...
                </code>  
                <br>
                The APIs deliver data in native byte order and expect the
                data read from the source to be in the network byte order.
  **/
  class UIMA_LINK_IMPORTSPEC SofaDataStream {
  public:
    /**
     * open the stream for reading
     * @param minbufsize optional specifies the minimum
     *                   size of the internal buffer the
     *                   stream handler should use.
     *        defaults to the value of BUFSIZE
     */
    virtual int open(size_t minbufsize=0) =0;

    /**
     * Gets the total size of the stream in number of bytes if known.
     *  
     * @return - size in bytes
     *           -1 if size cannot be determined as with an openended stream,
     *
     */
    virtual INT64 getTotalStreamSizeInBytes()=0;

    /**
    * Gets the number of bytes available.
    * 
    *  
    * @return - size in bytes
    *           -1 if size cannot be determined, 
    *
    */
    virtual INT64 howManyBytesAvailable()=0;


    /**
     * This call reads at most the number of elements into specified 
     * buffer.  The call blocks until the number of required
     * element are read or EOF.  This will return elements in
     * the native byte order for the current platform
     * 
     * The buffer is allocated and owned by the caller and must be
     * at least elementSize*numElements in size.
     *
     * @param pbuffer
     * @param elementSize e.g.,  1, 2, 4, 8
     * @param numElements
     * @return number of elements read
     *         -1 indicates EOF and no elements read 
     *         -2 indicates elementSize is not compatible with elementSize
     *            of the data source.  This would be the case for a 
     *            LocalSofA where the Sofa data is in a typed array FS.
     */
    virtual int read( void * pbuffer, int elementSize, size_t numElements)=0;


    /*
     * This call is used to reposition the current position in the stream to 
     * the specified offset from the specified origin.
     * The origin is one of: 
     *     
     * @param  offset in number of bytes
     * @param  origin as defined in stdio.h is one of 
     *         SEEK_SET,  from the start 
     *         SEEK_CUR,  from current location. 
     *         SEEK_END,  from end.
     * @return  0 for success,  
     *         -1 for EOF  
     *         anything else indicates an error.
     *
     */
    virtual int seek(INT64 offset, int origin)=0;


    /*
     * Close the stream.
     *
     */
    virtual void close()=0;
    /*
     * Get the pointer to the data.
     * This is valid only for a RemoteSofA 
     * where the entire SofA data is available in
     * memory.
     * @return a valid pointer or NULL.
     */
    virtual const TySofaDataPointer getDataPointer()=0;


    /*
     *  Destructor
     *
     */
    virtual ~SofaDataStream() { }
  };


  /**
   * This class implements stream access to Sofa data for
   * a Local Sofa.
   */
  class UIMA_LINK_IMPORTSPEC LocalSofaDataStream : public SofaDataStream {
  public:
    LocalSofaDataStream(SofaFS & sofaFS);
    ~LocalSofaDataStream();
    int open(size_t minbufsize=0);
    INT64 getTotalStreamSizeInBytes();
    INT64 howManyBytesAvailable();
    int read( void * pbuffer, int elementSize, size_t numElements);
    int seek(INT64 offset, int origin);
    void close();
    const TySofaDataPointer getDataPointer();

  private:
    SofaFS * iv_psofafs;           //the SofaFS
    const char * iv_psofadata;    //ptr to the sofa data
    char * iv_pstringsofadata;    //ptr to UTF 8 string
    INT64 iv_curpos;              //cursor position
    INT32 iv_size;                //total size of stream in bytes
    bool iv_isstring;             //is sofa data stored in a String Feature
    bool iv_isintarray;           //is sofa data stored in a FS array of ints
    bool iv_isfloatarray;         //is sofa data stored in a FS array of floats
    bool iv_isbooleanarray;         //is sofa data stored in a FS array of floats
    bool iv_isbytearray;         //is sofa data stored in a FS array of floats
    bool iv_isshortarray;         //is sofa data stored in a FS array of floats
    bool iv_islongarray;         //is sofa data stored in a FS array of floats
    bool iv_isdoublearray;         //is sofa data stored in a FS array of floats
  };


  /**
   * This class implements stream access to Sofa data for
   * a remote Sofa
   */
  class UIMA_LINK_IMPORTSPEC RemoteSofaDataStream : public SofaDataStream {
  public:
    RemoteSofaDataStream(SofaFS& sofaFS);
    ~RemoteSofaDataStream();
    int open(size_t minbufsize=0);
    INT64 getTotalStreamSizeInBytes();
    INT64 howManyBytesAvailable();
    int read( void * pbuffer, int elementSize, size_t numElements);
    int seek(INT64 offset, int origin);
    void close();
    const TySofaDataPointer getDataPointer();

  private:
    SofaFS * iv_psofafs;
    SofaStreamHandler  * iv_pHandler;
    util::DllProcLoaderFile *  iv_pHandlerDll;
  };

//return codes

#define SOFASTREAMHANDLER_SUCCESS                    0
#define SOFASTREAMHANDLER_EOF                       -1
#define SOFASTREAMHANDLER_OPEN_ENDED                -1

}

#endif
