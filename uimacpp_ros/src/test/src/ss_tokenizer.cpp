/**

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

   Description: A Unicode Tokenizer

-------------------------------------------------------------------------- */



/* ----------------------------------------------------------------------- */
/*       Include dependencies                                              */
/* ----------------------------------------------------------------------- */
// must be first include file to surpress silly compiler warnings
#include "uima/pragmas.hpp"
#include "uima/assertmsg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "unicode/uchar.h"

#include "uima/ss_tokenizer.hpp"

#include "uima/language.hpp"
#include "uima/resmgr.hpp"
#include "uima/err_ids.h"
#include "uima/msg.h"

namespace uima {

  static TyCharmap gs_cauiCharMapWard = {
                                          /*
                                           * character map for unicode character
                                           * The table is made up in "ward" tables. A "ward" is the first
                                           * byte in a unicode character.
                                           * Characters with ward 0 are the same as in codepage 819 (ISRI8859-1)
                                           */
                                        // WARD 0 (start 0x000)
                                          { /* 0x01, 0x02 required for masking, leave part of token! */
                                            CH_SPC, CH_USC, CH_USC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 00-07 '        ' */
                                            CH_SPC, CH_BLK, CH_NWL, CH_SPC, CH_SPC, CH_BLK, CH_SPC, CH_SPC, /* 08-0F '        ' */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 10-17 '        ' */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 18-1F '        ' */
                                            CH_BLK, CH_SND, CH_SPC, CH_SPC, CH_CUR, CH_SPC, CH_SPC, CH_APS, /* 20-27 ' !"#$%&'' */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_NSP, CH_CWS, CH_PRD, CH_CWS, /* 28-2F '()*+,-./' */
                                            CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM, /* 30-37 '01234567' */
                                            CH_NUM, CH_NUM, CH_CWS, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SND, /* 38-3F '89:;<=>?' */
                                            CH_CWS, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, /* 40-47 '@ABCDEFG' */
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, /* 48-4F 'HIJKLMNO' */
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, /* 50-57 'PQRSTUVW' */
                                            CH_UPR, CH_UPR, CH_UPR, CH_SPC, CH_CWS, CH_SPC, CH_SPC, CH_USC, /* 58-5F 'XYZ[\]^_' */
                                            CH_APS, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 60-67 '`abcdefg' */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 68-6F 'hijklmno' */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 70-77 'pqrstuvw' */
                                            CH_LWR, CH_LWR, CH_LWR, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 78-7F 'xyz     ' */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 80-87 '        ' */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 88-8F '        ' */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 90-97 '        ' */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 98-9F '        ' */
                                            CH_BLK, CH_SND, CH_SPC, CH_CUR, CH_CUR, CH_CUR, CH_SPC, CH_SPC, /* A0-A7 '        ' */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* A8-AF '        ' */
                                            CH_CUR, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* B0-B7 '°       ' */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SND, /* B8-BF '        ' */
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, /* C0-C7 '        ' */
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, /* C8-CF '        ' */
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_SPC, /* D0-D7 '        ' */
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_LWR, /* D8-DF '        ' */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* E0-E7 '        ' */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* E8-EF '        ' */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_SPC, /* F0-F7 '        ' */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR  /* F8-FF '        ' */
                                          },

                                        // WARD 1 (start 0x010)
                                          {
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 00-07  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 08-0F  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 10-17  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 18-1F  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 20-27  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 28-2F  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 30-37  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 38-3F  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 40-47  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 48-4F  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 50-57  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 58-5F  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 60-67  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 68-6F  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 70-77  */
                                            CH_UPR, CH_UPR, CH_LWR, CH_UPR,  CH_LWR, CH_UPR, CH_LWR, CH_LWR, /* 78-7F  */
                                            CH_LWR, CH_UPR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_UPR, /* 80-87  */
                                            CH_LWR, CH_UPR, CH_UPR, CH_UPR,  CH_LWR, CH_LWR, CH_UPR, CH_UPR, /* 88-8F  */
                                            CH_UPR, CH_UPR, CH_LWR, CH_UPR,  CH_UPR, CH_LWR, CH_UPR, CH_UPR, /* 90-97  */
                                            CH_UPR, CH_LWR, CH_LWR, CH_LWR,  CH_UPR, CH_UPR, CH_LWR, CH_UPR, /* 98-9F  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_LWR, CH_UPR, /* A0-A7  */
                                            CH_LWR, CH_UPR, CH_LWR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_UPR, /* A8-AF  */
                                            CH_LWR, CH_UPR, CH_UPR, CH_UPR,  CH_LWR, CH_UPR, CH_LWR, CH_UPR, /* B0-B7  */
                                            CH_UPR, CH_LWR, CH_LWR, CH_LWR,  CH_UPR, CH_LWR, CH_LWR, CH_LWR, /* B8-BF  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_UPR, CH_UPR, CH_LWR, CH_UPR, /* C0-C7  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_UPR,  CH_LWR, CH_UPR, CH_LWR, CH_UPR, /* C8-CF  */
                                            CH_LWR, CH_UPR, CH_LWR, CH_UPR,  CH_LWR, CH_UPR, CH_LWR, CH_UPR, /* D0-D7  */
                                            CH_LWR, CH_UPR, CH_LWR, CH_UPR,  CH_LWR, CH_LWR, CH_UPR, CH_LWR, /* D8-DF  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* E0-E7  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* E8-EF  */
                                            CH_LWR, CH_UPR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_UPR, /* F0-F7  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR  /* F8-FF  */
                                          },
                                        // WARD 2  (start 0x020)
                                          {
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 00-07  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 08-0F  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 10-17  */
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR,  CH_UPR, CH_LWR, CH_UPR, CH_LWR, /* 18-1F  */
                                            CH_SPC, CH_SPC, CH_UPR, CH_LWR,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 20-27  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 28-2F  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 30-37  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 38-3F  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 40-47  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* 48-4F  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 50-57  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 58-5F  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 60-67  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 68-6F  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 70-77  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 78-7F  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 80-87  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 88-8F  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 90-97  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* 98-9F  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* A0-A7  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_SPC, CH_SPC, /* A8-AF  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_LWR, CH_LWR, CH_LWR, CH_LWR, /* B0-B7  */
                                            CH_LWR, CH_SPC, CH_SPC, CH_SPC,  CH_APS, CH_SPC, CH_SPC, CH_SPC, /* B8-BF  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* C0-C7  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* C8-CF  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_CWS, /* D0-D7  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* D8-DF  */
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* E0-E7  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_APS, CH_SPC, /* E8-EF  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_SPC, /* F0-F7  */
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC,  CH_SPC, CH_SPC, CH_SPC, CH_SPC  /* F8-FF  */
                                          },
                                        // WARD 3 (start 0x030)
                                          {
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 00-07
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 08-0f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 10-17
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 18-1f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 20-27
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 28-2f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 30-37
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 38-3f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 40-47
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 48-4f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 50-57
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 58-5f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 60-67
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 68-6f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 70-77
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SND, CH_SPC,  // 78-7f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_UPR, CH_SPC,  // 80-87
                                            CH_UPR, CH_UPR, CH_UPR, CH_SPC, CH_UPR, CH_SPC, CH_UPR, CH_UPR,  // 88-8f
                                            CH_LWR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 90-97
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 98-9f
                                            CH_UPR, CH_UPR, CH_SPC, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // a0-a7
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // a8-af
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // b0-b7
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // b8-bf
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // c0-c7
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_SPC,  // c8-cf
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // d0-d7
                                            CH_SPC, CH_SPC, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // d8-df
                                            CH_LWR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // e0-e7
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // e8-ef
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // f0-f7
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC   // f8-ff
                                          },
                                        // WARD 4 (0x040)
                                          {
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 00-07
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 08-0f
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 10-17
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 18-1f
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 20-27
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 28-2f
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 30-37
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 38-3f
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 40-47
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 48-4f
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 50-57
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 58-5f
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // 60-67
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // 68-6f
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // 70-77
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // 78-7f
                                            CH_UPR, CH_LWR, CH_CUR, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 80-87
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // 88-8f
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // 90-97
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // 98-9f
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // a0-a7
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // a8-af
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // b0-b7
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // b8-bf
                                            CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_SPC, CH_SPC, CH_UPR,  // c0-c7
                                            CH_LWR, CH_SPC, CH_SPC, CH_UPR, CH_LWR, CH_SPC, CH_SPC, CH_SPC,  // c8-cf
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // d0-d7
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // d8-df
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // e0-e7
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR,  // e8-ef
                                            CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_UPR, CH_LWR, CH_SPC, CH_SPC,  // f0-f7
                                            CH_UPR, CH_LWR, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC   // f8-ff
                                          },
                                        // WARD 5 (0x050)
                                          {
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 00-07
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 08-0f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 10-17
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 18-1f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 20-27
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 28-2f
                                            CH_SPC, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 30-37
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 38-3f
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 40-47
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR,  // 48-4f
                                            CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_UPR, CH_SPC,  // 50-57
                                            CH_SPC, CH_LWR, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SND, CH_SPC,  // 58-5f
                                            CH_SPC, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 60-67
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 68-6f
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 70-77
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 78-7f
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 80-87
                                            CH_SPC, CH_PRD, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 88-8f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 90-97
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 98-9f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // a0-a7
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // a8-af
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // b0-b7
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // b8-bf
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // c0-c7
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // c8-cf
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // d0-d7
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // d8-df
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // e0-e7
                                            CH_LWR, CH_LWR, CH_LWR, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // e8-ef
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // f0-f7
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC   // f8-ff
                                          },
                                        // WARD 6 (0x060)
                                          {
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 00-07
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 08-0f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 10-17
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SND,  // 18-1f
                                            CH_SPC, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 20-27
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 28-2f
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 30-37
                                            CH_LWR, CH_LWR, CH_LWR, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 38-3f
                                            CH_SPC, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 40-47
                                            CH_LWR, CH_LWR, CH_LWR, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 48-4f
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 50-57
                                            CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_SPC,  // 58-5f
                                            CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM,  // 60-67
                                            CH_NUM, CH_NUM, CH_SPC, CH_SPC, CH_NUM, CH_SPC, CH_SPC, CH_SPC,  // 68-6f
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 70-77
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 78-7f
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 80-87
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 88-8f
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 90-97
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // 98-9f
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // a0-a7
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // a8-af
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // b0-b7
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // b8-bf
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // c0-c7
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // c8-cf
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_PRD, CH_LWR, CH_LWR, CH_LWR,  // d0-d7
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_SPC, CH_SPC, CH_LWR,  // d8-df
                                            CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR, CH_LWR,  // e0-e7
                                            CH_LWR, CH_SPC, CH_SPC, CH_SPC, CH_SPC, CH_LWR, CH_SPC, CH_SPC,  // e8-ef
                                            CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM, CH_NUM,  // f0-f7
                                            CH_NUM, CH_NUM, CH_LWR, CH_LWR, CH_LWR, CH_SPC, CH_SPC, CH_SPC   // f8-ff
                                          }
                                        };




  /**********************************************************************/
  /*                                                                    */
  /*                          A P I   Function                          */
  /*                                                                    */
  /**********************************************************************/

//lint -save -e909 : Implicit conversion from enum/pointer to bool

  Tokenizer::Tokenizer(void) :
      iv_bUseAlternateTerritories(true),
      iv_pauiCharMapWard(NULL) {
    assert(sizeof(TyCharmap) == ((MAXWARD+1)* 256 * sizeof(unsigned short)));
    assert(sizeof(gs_cauiCharMapWard) == sizeof(*iv_pauiCharMapWard));
    assert(sizeof(gs_cauiCharMapWard) == sizeof(TyCharmap));
    // we don't won't modify the global static map but we don't want to create a
    // writable copy until there is the need to do so (setCharClass() is called)
    iv_pauiCharMapWard = &gs_cauiCharMapWard;
  }

  Tokenizer::~Tokenizer(void) {
    resetCharClasses();
    iv_pauiCharMapWard = NULL;
  }

  void Tokenizer::resetCharClasses(void) {
    if (iv_pauiCharMapWard != &gs_cauiCharMapWard) {
      free(iv_pauiCharMapWard);
      iv_pauiCharMapWard = &gs_cauiCharMapWard;
    }
  }

  void Tokenizer::setCharClass(WORD16 uiUnicodeCodePoint, EnCharClass enCharClass)
  /* ----------------------------------------------------------------------- */
  {
    assert(EXISTS(iv_pauiCharMapWard));
    // This function is called rarely, so it is optimized for clarity rather than speed
    size_t uiWard = uiUnicodeCodePoint/256;  //determine the ward for the codepoint
    if (uiWard > (sizeof(TyCharmap)/256)) {
      return;
    }

    if (iv_pauiCharMapWard == &gs_cauiCharMapWard) {
      // allocate memory for writable copy
      iv_pauiCharMapWard = (TyCharmap*)malloc(sizeof(TyCharmap));
      if (iv_pauiCharMapWard == NULL) {
        UIMA_EXC_THROW_NEW(ExcOutOfMemory,
                           UIMA_ERR_USER_ANNOTATOR_OUT_OF_MEMORY,
                           UIMA_MSG_ID_EXC_OUT_OF_MEMORY,
                           uima::ErrorMessage(UIMA_MSG_ID_EXCON_TOK_ALLOCATING_CHARTABLE),
                           uima::ErrorInfo::unrecoverable);
      }
      // copy values from default map
      memcpy(*iv_pauiCharMapWard, gs_cauiCharMapWard, sizeof(TyCharmap));
    }

    size_t uiWardOffset = uiUnicodeCodePoint%256;
    (*iv_pauiCharMapWard)[uiWard][uiWardOffset] = (unsigned short)enCharClass;
  }

  static const EnCharClass gs_aenIcuCharCat2TokCharClass [U_CHAR_CATEGORY_COUNT+1] = {
        /** Non-category for unassigned and non-character code points.
               U_UNASSIGNED              = 0, */ CH_SPC,
        /** Lu U_UPPERCASE_LETTER        = 1, */ CH_UPR,
        /** Ll U_LOWERCASE_LETTER        = 2, */ CH_LWR,
        /** Lt U_TITLECASE_LETTER        = 3, */ CH_UPR,
        /** Lm U_MODIFIER_LETTER         = 4, */ CH_USC,
        /** Lo U_OTHER_LETTER            = 5, */ CH_USC,
        /** Mn U_NON_SPACING_MARK        = 6, */ CH_USC,
        /** Me U_ENCLOSING_MARK          = 7, */ CH_USC,
        /** Mc U_COMBINING_SPACING_MARK  = 8, */ CH_USC,
        /** Nd U_DECIMAL_DIGIT_NUMBER    = 9, */ CH_NUM,
        /** Nl U_LETTER_NUMBER           = 10,*/ CH_NUM,
        /** No U_OTHER_NUMBER            = 11,*/ CH_NUM,
        /** Zs U_SPACE_SEPARATOR         = 12,*/ CH_BLK,
        /** Zl U_LINE_SEPARATOR          = 13,*/ CH_NWL,
        /** Zp U_PARAGRAPH_SEPARATOR     = 14,*/ CH_NPA,
        /** Cc U_CONTROL_CHAR            = 15,*/ CH_SPC,
        /** Cf U_FORMAT_CHAR             = 16,*/ CH_SPC,
        /** Co U_PRIVATE_USE_CHAR        = 17,*/ CH_USC,
        /** Cs U_SURROGATE               = 18,*/ CH_USC,
        /** Pd U_DASH_PUNCTUATION        = 19,*/ CH_CWS,
        /** Ps U_START_PUNCTUATION       = 20,*/ CH_SPC,
        /** Pe U_END_PUNCTUATION         = 21,*/ CH_SPC,
        /** Pc U_CONNECTOR_PUNCTUATION   = 22,*/ CH_CWS,
        /** Po U_OTHER_PUNCTUATION       = 23,*/ CH_SPC,
        /** Sm U_MATH_SYMBOL             = 24,*/ CH_SPC,
        /** Sc U_CURRENCY_SYMBOL         = 25,*/ CH_CUR,
        /** Sk U_MODIFIER_SYMBOL         = 26,*/ CH_USC,
        /** So U_OTHER_SYMBOL            = 27,*/ CH_SPC,
        /** Pi U_INITIAL_PUNCTUATION     = 28,*/ CH_SPC,
        /** Pf U_FINAL_PUNCTUATION       = 29,*/ CH_SPC,
        /** Cn U_GENERAL_OTHER_TYPES     = 30,*/ CH_SPC
        /** One higher than the last enum UCharCategory constant.
            U_CHAR_CATEGORY_COUNT */
      };


// inline function used in this file
  inline EnCharClass
  Tokenizer::getCharClassInl( UChar c ) {
    // isolate first byte which designates ward
    unsigned char              c1 = c >> 8;

    // mapping tables only defined for the first WARDS
    if (c1 <= MAXWARD) {
      // isolate second byte
      unsigned char c2 = c & 0xFF;

      // use both byte parts for lookup in ward table
      return(EnCharClass) (*iv_pauiCharMapWard)[c1][c2];
    }

    assert(u_charType(c) >= 0);
    assert(u_charType(c) < U_CHAR_CATEGORY_COUNT);
    // for all other characters get unicode character type from ICU
    // and map the unicode character type to our character class using table
    return ( gs_aenIcuCharCat2TokCharClass[(UCharCategory)u_charType(c)] );

  }

  /* class function used in annotator_tok,cpp */
  EnCharClass Tokenizer::getCharClass( UChar c ) {
    return getCharClassInl(c);
  }



  /**********************************************************************/
  /*                                                                    */
  /*                          A P I   Function                          */
  /*                                                                    */
  /**********************************************************************/

  inline int Tokenizer::tokenEntry(
    const UChar *pToken, size_t ulLocation,  size_t ulLength,
    TokenProperties &rclTokenProperties,
    bool &bNewPara, bool &bNewSent, size_t & rulNewlines) {

    // send token to UIMA
    tokenCallback( ulLocation, ulLength, rclTokenProperties, bNewPara, bNewSent );

    // actions after the token was sent to UIMA:

    // reset token class for next token
    rclTokenProperties.reset();
    // reset new paragraph / new sentence flags
    bNewPara = bNewSent = false;
    // reset count for newlines (even if there was only one)
    rulNewlines = 0;

    return 0;
  }


  void Tokenizer::process(const UChar *text_start, const UChar *text_end) {
    assert(EXISTS(text_start));
    assert(EXISTS(text_end));
    assert(EXISTS(iv_pauiCharMapWard));

    //? UString str((UniChar *) text_start, (size_t) (text_end - text_start) + 1);
    //? cout << ">>> '" << str.prv_asSingleByteString(CCSID(819)) << "' <<<" << endl;

    const UChar *pText = text_start;   // curent pointer in text
    const UChar *pWordStart = NULL;    // start of current word or NULL
    // if not in a word
    bool bNewSent = false;             // next Word is in new sentence
    bool bNewPara = false;             // next Word is in new paragraph
    size_t uiNewlines = 0;             // number of subsequent newlines
    // (more than 2 indicate new paragraph)
    TokenProperties clTokenProperties; // class of current word (e.g. all upper)

    //clTokenProperties.reset()

    while ( pText <= text_end ) {
      EnCharClass             charClass = getCharClassInl( *pText );
      const UChar             chTextNext = (pText < text_end) ? *(pText + 1) : 0;

      // Default case: current character is upper or lower case character or digit:
      if ( charClass & (CH_LWR | CH_UPR | CH_NUM | CH_CUR | CH_USC )  ) {
        if ( pWordStart == NULL ) {
          // the start of a new word
          pWordStart = pText;
        }
        // token class classification (most frequent checked first)
        if ( charClass & CH_LWR )
          clTokenProperties.setLower();
        else if ( charClass & CH_UPR ) {
          if ( pWordStart == pText )
            clTokenProperties.setLeadingUpper();
          else
            clTokenProperties.setTrailingUpper();
        } else if ( charClass & CH_NUM )
          clTokenProperties.setNumeric();
        else if ( charClass & CH_USC )
          clTokenProperties.setSpecial();
        else if ( charClass & CH_CUR ) {
          if ( pWordStart == pText ) {
            // accept currency only as a first character, if a digit
            // is following
            if ( getCharClassInl( chTextNext) != CH_NUM ) {
              pWordStart = NULL;    // reset word pointer ("not in a word")
            } else {
              clTokenProperties.setSpecial();
            }
          } else {
            clTokenProperties.setSpecial();
          }
        }
        // move to next character
        pText++;
        continue;
      }

      switch ( charClass ) {
      case CH_BLK:    // blank
        // unconditionally terminates the current word as a token
        // and starts a new word
        if ( pWordStart ) {
          // end of current word
          tokenEntry( pWordStart, pWordStart-text_start, pText-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
          pWordStart = NULL;
        }
        break;
      case CH_SPC:    // special character
        // unconditionally terminates the current word as a token
        // and starts a new word
        if ( pWordStart ) {
          // end of current word
          tokenEntry( pWordStart, pWordStart-text_start, pText-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
        }
        // the start of a new "word" containing the special char(s)
        pWordStart = pText;
        clTokenProperties.setSpecial();
        // check if the next char is end of a special char(s sequence)
        if (pWordStart && !(getCharClassInl( chTextNext ) & (CH_SPC))) {
          // create the special char(s sequence)
          tokenEntry( pWordStart, pWordStart-text_start, pText+1-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
          pWordStart = NULL;
        }
        break;
      case CH_SND:    // sentence end ("?" or "!")
        // terminates the current sentence
        if ( pWordStart ) {
          if (!(getCharClassInl( *(pText-1) ) & (CH_SND))) {
            // create the token immediately to the left of ? e.g. "abc?"
            tokenEntry( pWordStart, pWordStart-text_start, pText-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
            // the start of a new "word" containing the '''
            pWordStart = pText;
          }
        } else {
          // the start of a new "word" containing the '''
          pWordStart = pText;
        }
        clTokenProperties.setSpecial();
        // check if the next char is end of a ? or ??? sequence
        if (pWordStart && !(getCharClassInl( chTextNext ) & (CH_SND))) {
          // create the ? or ??? token
          tokenEntry( pWordStart, pWordStart-text_start, pText+1-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
          // start a new sentence
          bNewSent = true;
          pWordStart = NULL;
        }
        break;
      case CH_NWL:   // newline
        if ( pWordStart ) {
          // end of current word
          tokenEntry( pWordStart, pWordStart-text_start, pText-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines );
          pWordStart = NULL;
        }
        // count occuring newlines, if a new word starts and there were more
        // than one newlines, this is the begin of a new paragraph
        ++uiNewlines;
        // if there were some newlines before
        // start a new paragraph
        if ( uiNewlines > 1 ) {
          // new paragraph (and new sentence)
          bNewPara = true;
          bNewSent = true;
        }
        break;
      case CH_NPA:   // newpara
        if ( pWordStart ) {
          // end of current word
          tokenEntry( pWordStart, pWordStart-text_start, pText-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines );
          pWordStart = NULL;
        }
        // new paragraph (and new sentence)
        bNewPara = true;
        bNewSent = true;
        break;
      case CH_PRD:    // period
        // if not in a word, ignore a leading point
        if ( pWordStart ) {
          if ( pText == text_end ) {
            // period is the last character in the text:
            // Since no characters are following, this can only be the
            // end of the sentence.
            // end of current word
            tokenEntry( pWordStart, pWordStart-text_start, pText-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines );
            // now emmit the final period token itself
            // the start of a new "word" containing the . or ..
            pWordStart = pText;
            clTokenProperties.setSpecial();
            tokenEntry( pWordStart, pWordStart-text_start, 1, clTokenProperties, bNewPara, bNewSent, uiNewlines );
            // setting bNewSent to true here is not really neccessary.
            // since we are at the end of the text. However, to have
            // the same code for all "sentence end" conditions, this is
            // left here.
            bNewSent = true;
            pWordStart = NULL;
          } else {
            // period is not at the end of the text - action depends on leading and following character
            // note: since pWordStart is not NULL here, pText points not
            // to the very beginning of the text.
            //
            // part of the word if between numbers of alpha characters (like conditional whitespaces)
            // This is for tokens like "9.164.220.12"
            if ( (getCharClassInl( *(pText-1)) & (CH_UPR | CH_LWR | CH_NUM )) &&
                 (getCharClassInl( chTextNext) & (CH_UPR | CH_LWR | CH_NUM ))) {
              clTokenProperties.setSpecial();
              break;
            }
            unsigned long ulWordLen = pText-pWordStart;
            const UChar    chTextNextNext = (pText < (text_end - 1)) ? *(pText + 2) : 0;

            if (   (ulWordLen ==1 && clTokenProperties.hasUpper())
                   // OR beginning of next token is lower: must be abrev
                   || (getCharClassInl( chTextNextNext) & (CH_LWR))
                   // OR found in abbreviation list
       //                       || isAbreviation( pWordStart, ulWordLen ) ) {
               ) {
              clTokenProperties.setSpecial();
              // is an abbreviation, ignore this word, do not end the sentence
              // pass token WITH period.
              tokenEntry( pWordStart, pWordStart-text_start, ulWordLen+1, clTokenProperties, bNewPara, bNewSent, uiNewlines );
              pWordStart = NULL;
            } else if (!(getCharClassInl( *(pText-1) ) & (CH_PRD))) {
              // must be the end of a sentence
              // end of current word (without period)
              tokenEntry( pWordStart, pWordStart-text_start, ulWordLen, clTokenProperties, bNewPara, bNewSent, uiNewlines );
              pWordStart = pText;
            }
          }
        } else {
          // the start of a new "word" containing the ...
          pWordStart = pText;
        }
        if (pWordStart) {
          clTokenProperties.setSpecial();
        }
        // check if the next char is end of a . or ... sequence
        // Note: we allow for ".12" or ".Net" or ..12 to be one token
        if (pWordStart && !(getCharClassInl( chTextNext ) & (CH_PRD | CH_NUM | CH_LWR | CH_UPR))) {
          clTokenProperties.setSpecial();
          // create the . or ... token
          tokenEntry( pWordStart, pWordStart-text_start, pText+1-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
          if (getCharClassInl( chTextNext ) & (CH_BLK | CH_NWL)) {
            bNewSent = true;
          }
          pWordStart = NULL;
        }
        break;
      case CH_NSP:    // number seperator or ','
        if ( pWordStart ) {
          // part of a number if between digits
          if ( getCharClassInl( *(pText-1)) == CH_NUM && getCharClassInl(chTextNext) == CH_NUM  ) {
            clTokenProperties.setSpecial();
            break;
          } else if (!(getCharClassInl( *(pText-1) ) & (CH_NSP))) {
            // create the token immediately to the left of , e.g. "abc,"
            tokenEntry( pWordStart, pWordStart-text_start, pText-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
            // the start of a new "word" containing the '''
            pWordStart = pText;
          }
        } else {
          // the start of a new "word" containing the '''
          pWordStart = pText;
        }
        clTokenProperties.setSpecial();
        // check if the next char is end of a , or ,,, sequence
        if (pWordStart && !(getCharClassInl( chTextNext ) & (CH_NSP))) {
          // create the , or ,,, token
          tokenEntry( pWordStart, pWordStart-text_start, pText+1-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
          pWordStart = NULL;
        }
        break;
      case CH_CWS:    // conditional whitespace
        if ( pWordStart ) {
          // part of the word if between alphanumeric character follows
          if ( (getCharClassInl( *(pText-1)) & (CH_UPR | CH_LWR | CH_NUM )) &&
               (getCharClassInl( chTextNext) & (CH_UPR | CH_LWR | CH_NUM ))) {
            clTokenProperties.setSpecial();
            break;
          } else if (!(getCharClassInl( *(pText-1) ) & (CH_CWS))) {
            // create the token immediately to the left of , e.g. "abc,"
            tokenEntry( pWordStart, pWordStart-text_start, pText-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
            // the start of a new "word" containing the '''
            pWordStart = pText;
          }
        } else {
          // the start of a new "word" containing the '''
          pWordStart = pText;
        }
        clTokenProperties.setSpecial();
        // check if the next char is end of a - or --- sequence
        if (pWordStart && !(getCharClassInl( chTextNext ) & (CH_CWS))) {
          // create the - or --- token
          tokenEntry( pWordStart, pWordStart-text_start, pText+1-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
          pWordStart = NULL;
        }
        break;
      case CH_APS:   // apostroph is part of the word within words (l'oreal, Tom's, don't)
        if ( pWordStart ) {
          // part of the word if between alphanumeric character follows
          if ( (getCharClassInl( *(pText-1)) & (CH_UPR | CH_LWR | CH_NUM )) &&
               (getCharClassInl( chTextNext) & (CH_UPR | CH_LWR | CH_NUM ))) {
            clTokenProperties.setSpecial();
            break;
          } else if (!(getCharClassInl( *(pText-1) ) & (CH_APS))) {
            // create the token immediately to the left of , e.g. "abc,"
            tokenEntry( pWordStart, pWordStart-text_start, pText-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
            // the start of a new "word" containing the '''
            pWordStart = pText;
          }
        } else {
          // the start of a new "word" containing the '''
          pWordStart = pText;
        }
        clTokenProperties.setSpecial();
        // check if the next char is end of a ' or ''' sequence
        if (pWordStart && !(getCharClassInl( chTextNext ) & (CH_APS))) {
          // create the ' or ''' token
          tokenEntry( pWordStart, pWordStart-text_start, pText+1-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
          pWordStart = NULL;
        }
        break;
      case CH_LWR: // all the following cases are handled in the first loop
      case CH_UPR:
      case CH_NUM:
      case CH_USC:
      case CH_CUR:
      default:
        assert( false );
        break;
      }
      ++pText;
    }

    // if end of text and still in a word
    // send the word to UIMA
    if ( pWordStart ) {
      // end of current word
      tokenEntry( pWordStart, pWordStart-text_start, pText-pWordStart, clTokenProperties, bNewPara, bNewSent, uiNewlines  );
      pWordStart = NULL;
    }
  }

//lint -restore : Implicit conversion from enum/pointer to bool

} // namespace uima

/* <EOF> */

