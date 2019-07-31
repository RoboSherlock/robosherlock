/** \file ftools.hpp .



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

    \brief  Assorted tools for file manipulation/management.


-------------------------------------------------------------------------------
*/

#ifndef UIMA_FILETOOLS_HPP_
#define UIMA_FILETOOLS_HPP_

#include <uima/pragmas.hpp> // must be first file to be included to get pragmas
#include <string>
#include <uima/filename.hpp>
#include <uima/strtools.hpp>

/* ----------------------------------------------------------------------- */
/*      Interface dependencies                                             */
/* ----------------------------------------------------------------------- */



/**
   Tries to read a text file into a string buffer.
   Returns true if successful, for example, the file exists and is readable.
   FileSize()+1 bytes will allocated for rpszData.
   The calling application is responsible to free rpszData.
   Returns the size of the file read, or 0 if the file does not exist
*/
UIMA_LINK_IMPORTSPEC size_t
ftool_ReadFileToBuffer(
  const uima::util::Filename& rclFileName,
  char *&             rpszData
);


/**
   Tries to read a text file into a string buffer.
   Returns the size of the file read, or 0 if the file does not exist
*/
UIMA_LINK_IMPORTSPEC size_t
ftool_ReadFileToString(
  const uima::util::Filename& rclFileName,
  std::string&             rsData
);

/**
  Counts the number of lines in a text file
  Returns -1 if the file is not readable, otherwise the number of lines in the file.
*/
UIMA_LINK_IMPORTSPEC long
ftool_NbrOfLines(
  uima::util::Filename& rclFileName
);

#endif // UIMA_FILETOOLS_HPP_


