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

-----------------------------------------------------------------------------

   \brief  An example implementation of the Sofa Stream Handler APIs
           defined in uima/sofastreamhandler.hpp
           to support the file URI scheme

-----------------------------------------------------------------------------

   Description:  This implements a SofaStreamHandler for a file URI scheme.
     The file to be read is specified as an argument to
     openStream(char * uriString). uriString is expected to be
     contain the URI scheme and path to file as follows:
                   file:myfile
                   eg: file://C:\myfile.txt  on Windows
                       file:///home/myid/myfile.txt on UNIX

                 To build the SofaStreamHandlerFile DLL:
                   On linux:
                     make -f SofaStreamHandlerFile.mak
                   On Windows:
                     devenv SofaStreamHandlerFile.vcproj /build Release

                 The handler dll must be registered with the UIMA framework
                 by setting an environement variable as follows:
                   On Linux:
                     export UIMACPP_STREAMHANDLERS=file:SofaStreamHandlerFile
                   On Windows:
                     set UIMACPP_STREAMHANDLERS=file:SofaStreamHandlerFile

-------------------------------------------------------------------------- */



#include <uima/api.hpp>
#include <uima/sofastreamhandler.hpp>
#include <sys/stat.h>
#if defined(__OS_UNIX__)
#define LSEEK lseek
#include <unistd.h>
#elif defined(__OS_WIN32__)
#define LSEEK _lseeki64
#include <io.h>
#endif
#include <stdio.h>


using namespace uima;
using namespace std;


class SofaStreamHandlerFile : public SofaStreamHandler {


private:
  FILE * stream;
  char * buffer;
  INT64 filesize;
  size_t buflen;
  INT64 cur_pos ;

public:
//--------------------------------------------------------------
// openStream
// @param - uriString in UTF-8
//--------------------------------------------------------------
  void openStream(const char * uriString) {
    return openStream(uriString,BUFSIZ);
    ;
  }

//--------------------------------------------------------------
// openStreamWithMinInternalBufSize
// open the file for reading and allocate an internal buffer of at least the
// specified size.
// @param - uriString in UTF-8
//          minimumBufferSize
//--------------------------------------------------------------
  void openStream (const char * uriString, size_t minimumBufferSize) {
    cout << "SofaStreamHandlerFile: openStream() " << uriString << endl;
    if (strncmp(uriString, "file://", 7) == 0) {
      buffer = new char[minimumBufferSize];
      buflen = minimumBufferSize;
      cur_pos=0;
      const char * filename = uriString+7;
      stream = fopen(filename,"rb");
      if (stream == NULL) {
        UIMA_EXC_THROW_NEW(Exception,
                           UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT,
                           UIMA_MSG_ID_EXCON_CONFIG_VALUE_EXTRACT,
                           ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, "Invalid value for SegmentDelimiter"),
                           ErrorInfo::unrecoverable);

      } else  {
        int rc = setvbuf (stream, buffer, _IOFBF, buflen);
        if (rc==0) {
          struct stat fstat;
          stat(filename, &fstat);
          filesize = fstat.st_size;
        }
      }
    } else {
      UIMA_EXC_THROW_NEW(Exception,
                         UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT,
                         UIMA_MSG_ID_EXCON_CONFIG_VALUE_EXTRACT,
                         ErrorMessage(UIMA_MSG_ID_LITERAL_STRING, "Invalid value for SegmentDelimiter"),
                         ErrorInfo::unrecoverable);

    }
    return  ;
  }

//--------------------------------------------------------------
// getTotalStreamSize
// @return total size of the data
//--------------------------------------------------------------
  INT64 getTotalStreamSize() {
    return filesize;
  }

//--------------------------------------------------------------
// getBufferSize
// @return buffer length
//--------------------------------------------------------------
  size_t getBufferSize() {
    return buflen;
  }

//--------------------------------------------------------------
// howManyAvailable
// @return number of bytes available for read
//         -1 indicates EOF
//--------------------------------------------------------------
  INT64 howManyAvailable() {
    if (filesize-cur_pos < 0) return -1;
    else return (filesize-cur_pos);
  }

//--------------------------------------------------------------
// getNext - deliver the specified number of bytes in network
//           byte order.
// reads the specified number of bytes from the
// current position.  Advances the current position
// by the number of bytes read. This implementation
// assumes that the data in the file is in network
// byte order.
// This call blocks till read request is satisfied or
// EOF is reached.
//
// @param - numBytes, the number of bytes to read
//        - pBuffer, buffer into which the bytes are to be copied.
//
// @returns number of bytes actually copied into buffer.
//          This may be less than the number of
//          bytes requested.
//          -1 indicates EOF.
//--------------------------------------------------------------
  INT64 getNext(size_t numBytes,
                void * pBuffer) {
    cout << "SofaStreamHandlerFile: getNext() " << numBytes << endl;
    if (feof(stream)) {
      return -1;
    }

    size_t numread = fread(pBuffer,1,numBytes,stream);
    cur_pos += numread;

    return numread;
  }

//--------------------------------------------------------------
// seek
// sets the position within the current stream.
// @param - offset - number of bytes from origin
//        - origin is one of the following (taken from lseek spec):
//          If SEEK_SET, the position is set to offset bytes.
//          If SEEK_CUR, the position is  set  to  its
//              current location plus offset bytes.
//          If SEEK_END,  the position is set to the total length
//              plus offset bytes.
//          These constants are defined in stdio.h
//
// @return 0 indicates success
//--------------------------------------------------------------
  int  seek (INT64 offset, int origin) {

    if (offset == 0) {
      cur_pos = fseek(stream, offset, origin );
      return 0;
    }

    INT64 remaining = offset;
    long off = 0;

    while (remaining > 0) {
      if (remaining > LONG_MAX) {
        off = LONG_MAX;
      } else {
        off = remaining;
      }
      remaining = remaining-off;
      cur_pos = fseek(stream, off, origin );
    }

    if (cur_pos == -1L)
      return -3;
    else return 0 ;

  }

//--------------------------------------------------------------
// closeStream
// close the stream. Delete the internal buffer.
//-------------------------------------------------------------
  void closeStream() {
    if (stream != NULL) {
      fclose(stream);
      stream=NULL;
    }
    if (buffer != NULL)  {
      delete[] buffer;
      buffer=NULL;
    }
    return;
  }

//--------------------------------------------------------------
// getDataPointer
// This returns a pointer to the data in memory.
// @return  NULL.
//--------------------------------------------------------------

  void *  getDataPointer() {
    return NULL;
  }
};

MAKE_HANDLER(SofaStreamHandlerFile);
