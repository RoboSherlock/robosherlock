#ifndef UIMA_ANALYSIS_COMPONENT_H
#define UIMA_ANALYSIS_COMPONENT_H
/** @name analysis_component.hpp
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

   Description:  This file contains a generic wrapper module
                 to map a user written C++ UIMA Component object to the
                 required UIMA Annotator API C functions.
                 It assumes that the the user typedefs its analysis component type
                 to the name UserDefinedAnnotator

     This replaces taf_annotator_generic.inl
     and taf_casconsumer_generic.inl.
-----------------------------------------------------------------------------

   5/12/2006  Initial creation

-------------------------------------------------------------------------- */

MAKE_AE(UserDefinedAnnotator);


#endif /* UIMA_ANALYSIS_COMPONENT_H */
/* <EOF> */

