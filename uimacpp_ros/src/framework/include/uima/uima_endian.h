/** \file endian.h .
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

    \brief Macros to convert endianess

-------------------------------------------------------------------------- */

#ifndef __UIMA_ENDIAN_H
#define __UIMA_ENDIAN_H

#ifdef __cplusplus
#define INLINEorSTATIC inline
#else
#define INLINEorSTATIC static
#endif

INLINEorSTATIC void __reverse(void *buffer, unsigned int size) {
  char *x = (char *) buffer;
  for (unsigned int i=0; i<size/2; i++) {
    char t = x[i];
    x[i] = x[size-1-i];
    x[size-1-i]=t;
  }
}


INLINEorSTATIC void __revarray(void *elem_array, unsigned int elem_size, unsigned int n_elem) {
  char *p = (char *) elem_array;
  for (unsigned int i=0; i<n_elem; i++) {
    __reverse((void *) (p + i*elem_size),elem_size);
  }
}

/* UIMA_LOBYTEFIRST will convert n_elem of size elem_size from little-endian to
   native byte-order or the reverse, i.e swapping if on a big-endian cpu.       */

#ifdef WORDS_BIGENDIAN
#define UIMA_LOBYTEFIRST(array,elem_size,n_elem) __revarray(array,elem_size,n_elem)
#define UIMA_HIBYTEFIRST(array,elem_size,n_elem)

#else
#define UIMA_HIBYTEFIRST(array,elem_size,n_elem) __revarray(array,elem_size,n_elem)
#define UIMA_LOBYTEFIRST(array,elem_size,n_elem)
#endif


/* Test if first UCS2 character is a byte-order mark */

#define hasUCS2BigEndianBOM(x)     ( *((WORD16*)x) == 0xFEFF )
#define hasUCS2LittleEndianBOM(x)  ( *((WORD16*)x) == 0xFFFE )

#endif /*__UIMA_ENDIAN_H*/
