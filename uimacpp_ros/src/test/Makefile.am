# ---------------------------------------------------------------------------
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The ASF licenses this file to You under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with
# the License.  You may obtain a copy of the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------

TESTS = fvt.sh
check_PROGRAMS = $(TST_BIN)
#noinst_libdir = $(libdir)
check_LTLIBRARIES = $(TST_LIB)

EXTRA_DIST = data fvtTestfile fvt.sh
dist-hook:
	rm -rf `find $(distdir)/src/test/data -name .svn`



TST_LIB=libtoknz.la libdump.la libSofaStreamHandlerFile.la libSimpleTextSegmenter.la libTextSegmentConsumer.la libMeetingAnnotator.la

TST_BIN=test_cas test_engine test_typepriority test_language  \
 test_iterators test_casserializer test_sofa test_primitivetypes test_xcasdeserialization test_xmideserialization

test_tgt: $(TST_BIN) $(TST_LIB)

clean:
	rm -f $(TST_BIN) $(test_engine_OBJECTS) $(test_typepriority_OBJECTS) $(test_sofa_OBJECTS) $(test_language_OBJECTS) $(test_cas_OBJECTS)  $(test_iterators_OBJECTS) $(test_casserializer_OBJECTS) $(test_primitivetypes_OBJECTS) $(test_xcasdeserialization_OBJECTS) $(test_xmideserialization_OBJECTS)
	rm -f $(TST_LIB) $(libtoknz_la_OBJECTS) $(libdump_la_OBJECTS) $(libSofaStreamHandlerFile_la_OBJECTS) $(libSimpleTextSegmenter_la_OBJECTS) $(libTextSegmentConsumer_la_OBJECTS) $(libMeetingAnnotator_la_OBJECTS)

INCLUDES  = -Isrc
INCLUDES += -I../cas
INCLUDES += -I../framework
INCLUDES += $(UIMA_XERCES_INCLUDE)
INCLUDES += $(UIMA_APR_INCLUDE)
INCLUDES += $(UIMA_ICU_INCLUDE)
INCLUDES += $(UIMACPP_HOME_INCLUDE)

if DEBUG_TEST
AM_CPPFLAGS = -g -DDEBUG
else
AM_CPPFLAGS = -O3 -DNDEBUG -DTRACEOFF
endif
AM_CPPFLAGS += -D_GNU_SOURCE -DUIMA_SUPPRESS_TIMING

# Set default for all target_LDADD values
LDADD = $(UIMA_LIB) $(UIMA_XERCES_LIB) $(UIMA_APR_LIB) $(UIMA_ICU_LIB) $(UIMACPP_HOME_LIB) -ldl

# Set default for all target_LIBADD values
LIBADD = $(UIMA_LIB) $(UIMA_XERCES_LIB) $(UIMA_APR_LIB) $(UIMA_ICU_LIB) $(UIMACPP_HOME_LIB) -ldl

# Common routines formerly in library
# src/cmdline_driver.cpp src/doc_buffer.cpp src/htmlparser.cpp src/htmlparser_tools.cpp src/parse_handlers.cpp src/parser_config.cpp src/tt_types.cpp src/xmlparse_handlers.cpp
include_HEADERS = src/uima/*.hpp
test_casserializer_SOURCES  = src/test_casserializer.cpp

test_engine_SOURCES  = src/test_engine.cpp
test_engine_SOURCES += src/conui.cpp
test_engine_SOURCES += src/doc_buffer.cpp

test_cas_SOURCES  = src/test_cas.cpp
test_cas_SOURCES += src/tt_types.cpp

test_iterators_SOURCES  = src/test_iterators.cpp
test_iterators_SOURCES += src/tt_types.cpp

test_language_SOURCES = src/test_language.cpp

test_typepriority_SOURCES  = src/test_typepriority.cpp
test_typepriority_SOURCES += src/tt_types.cpp

test_sofa_SOURCES = src/test_sofa.cpp

test_primitivetypes_SOURCES = src/test_primitivetypes.cpp

test_xcasdeserialization_SOURCES = src/test_xcasdeserialization.cpp

test_xmideserialization_SOURCES = src/test_xmideserialization.cpp


# Plugin libraries
#
#   Since tt_types is also used in the exes we need to pretend that the libraries 
#   that use it have diffeerent compile flags so thy'll get different obj names.
#   Otherwise we get: object `tt_types.$(OBJEXT)' created both with libtool and without
#

LT_FLAGS = -avoid-version -module -rpath /somewhere

libtoknz_la_SOURCES  = src/annotator_tok.cpp
libtoknz_la_SOURCES += src/ss_tokenizer.cpp
libtoknz_la_SOURCES += src/tt_types.cpp
libtoknz_la_LIBADD = $(LIBADD)
libtoknz_la_CXXFLAGS = $(AM_CXXFLAGSi)
libtoknz_la_LDFLAGS = $(LT_FLAGS)

libdump_la_SOURCES = src/annotator_dump.cpp
libdump_la_LIBADD = $(LIBADD)
libdump_la_LDFLAGS = $(LT_FLAGS)

libSofaStreamHandlerFile_la_SOURCES = src/SofaStreamHandlerFile.cpp
libSofaStreamHandlerFile_la_LIBADD = $(LIBADD)
libSofaStreamHandlerFile_la_LDFLAGS = $(LT_FLAGS)

libSimpleTextSegmenter_la_SOURCES = src/SimpleTextSegmenter.cpp
libSimpleTextSegmenter_la_LIBADD = $(LIBADD)
libSimpleTextSegmenter_la_LDFLAGS = $(LT_FLAGS)

libTextSegmentConsumer_la_SOURCES = src/TextSegmentConsumer.cpp
libTextSegmentConsumer_la_LIBADD = $(LIBADD)
libTextSegmentConsumer_la_LDFLAGS = $(LT_FLAGS)

libMeetingAnnotator_la_SOURCES = src/MeetingAnnotator.cpp
libMeetingAnnotator_la_LIBADD = $(LIBADD)
libMeetingAnnotator_la_LDFLAGS = $(LT_FLAGS)
