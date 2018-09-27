# -*- makefile -*-

 # Licensed to the Apache Software Foundation (ASF) under one
 # or more contributor license agreements.  See the NOTICE file
 # distributed with this work for additional information
 # regarding copyright ownership.  The ASF licenses this file
 # to you under the Apache License, Version 2.0 (the
 # "License"); you may not use this file except in compliance
 # with the License.  You may obtain a copy of the License at
 # 
 #   http://www.apache.org/licenses/LICENSE-2.0
 # 
 # Unless required by applicable law or agreed to in writing,
 # software distributed under the License is distributed on an
 # "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 # KIND, either express or implied.  See the License for the
 # specific language governing permissions and limitations
 # under the License.

bin_PROGRAMS = $(UTILS_BIN)

if ACTIVEMQ_FOUND
UTILS_BIN= runAECpp deployCppService
else
UTILS_BIN= runAECpp
endif

test_tgt: $(UTILS_BIN)

clean:
	rm -f $(UTILS_BIN) $(runAECpp_OBJECTS) $(deployCppService_OBJECTS)

if DEBUG_TEST
AM_CPPFLAGS = -g -DDEBUG
else
AM_CPPFLAGS = -O3 -DNDEBUG -DTRACEOFF
endif
AM_CPPFLAGS += -D_GNU_SOURCE

# Set default for all target_LDADD values
deployCppService_LDADD = $(UIMA_LIB) $(UIMA_APR_LIB) $(UIMA_XERCES_LIB) $(UIMA_ICU_LIB) $(UIMA_ACTIVEMQ_LIB)  -ldl $(UIMACPP_HOME_LIB)
runAECpp_LDADD = $(UIMA_LIB) $(UIMA_APR_LIB) $(UIMA_XERCES_LIB) $(UIMA_ICU_LIB)   -ldl $(UIMACPP_HOME_LIB)


runAECpp_SOURCES = runAECpp.cpp
runAECpp_CPPFLAGS  = -g -DDEBUG
runAECpp_CPPFLAGS += -D_GNU_SOURCE

deployCppService_SOURCES = deployCppService.cpp ActiveMQAnalysisEngineService.cpp deployCppService.hpp ActiveMQAnalysisEngineService.hpp



deployCppService_CPPFLAGS  = -g -DDEBUG
deployCppService_CPPFLAGS += -D_GNU_SOURCE


INCLUDES  = -I.
INCLUDES += -I../framework
INCLUDES += -I../cas
INCLUDES += $(UIMA_XERCES_INCLUDE)
INCLUDES += $(UIMA_APR_INCLUDE)
INCLUDES += $(UIMA_ICU_INCLUDE)
INCLUDES += $(UIMA_ACTIVEMQ_INCLUDE)
INCLUDES += $(UIMACPP_HOME_INCLUDE)
