/** \file ftools.cpp .
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

#include <uima/ftools.hpp>

#include <stdio.h>
#include <limits.h>

#include <fstream>
#include <iomanip>
#include <string>
using namespace std;

#include <uima/filename.hpp>
using namespace uima;

#include <uima/strtools.hpp>

/* ----------------------------------------------------------------------- */
/*      Constants                                                          */
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
/*      Implementation                                                     */
/* ----------------------------------------------------------------------- */




size_t
ftool_ReadFileToBuffer(
  const util::Filename& rclFileName,
  char *&             rpszData
) {
  rpszData = NULL;
  unsigned long lFSize = rclFileName.getFileSize();
  if (lFSize == 0) {
    return 0;
  }
// **need to check this**    assert(sizeof(size_t) == sizeof(unsigned int)); //make sure size_t is indeed an unsigned int
  assert((unsigned long)lFSize <= (unsigned long)UINT_MAX-1); //we cannot read more than that in one go

  FILE * pfStream = NULL;
  pfStream = fopen(rclFileName.getAsCString(), "rb");
  if (pfStream == NULL) {
    return 0;
  }

  rpszData = (char*)malloc((size_t)lFSize + 1);
  assert(rpszData != NULL);

  size_t uiBytesRead;
  uiBytesRead = fread(rpszData, 1, (size_t)lFSize, pfStream );
  fclose(pfStream);

  if (uiBytesRead != lFSize) {
    free(rpszData);
    rpszData = NULL;
    return 0;
  }
  assert((unsigned long)uiBytesRead <= lFSize);
  rpszData[uiBytesRead] = '\0';  //zero terminate buffer
  return (size_t) uiBytesRead;
}


size_t
ftool_ReadFileToString(
  const util::Filename& rclFileName,
  string&             rsData
) {
  size_t                     uiRetVal;

  rsData = "";
  char * pszBuff  = NULL;

  uiRetVal = ftool_ReadFileToBuffer(rclFileName, pszBuff);
  if (uiRetVal != 0) {
    rsData.insert(0, pszBuff, uiRetVal);
    free(pszBuff);
  }

  return uiRetVal;
}


/* counts the number of lines in a text file                          */
/* return: -1 if file not readable, number of lines else              */
long
ftool_NbrOfLines(
  const util::Filename& rclFileName
) {
  ifstream sin(rclFileName.getAsCString());
  if (!sin) {
    return -1;
  }
  string data;
  long lines = 0;
  while (!sin.eof()) {
    lines++;
    // ... we can use ANSI function getline
    (void)getline(sin, data, '\n');
  }
  return lines;
}



