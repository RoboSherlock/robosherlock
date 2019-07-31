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

lib_LTLIBRARIES = libuima.la

nobase_include_HEADERS =
nobase_include_HEADERS += uima/*.h
nobase_include_HEADERS += uima/*.hpp
cas_includedir = $(includedir)/uima
cas_include_HEADERS = ../cas/uima/*.hpp
jni_includedir = $(includedir)/uima
jni_include_HEADERS = ../jni/uima/*.hpp
jni_include_HEADERS += ../jni/uima/*.h

uima_tgt: libuima.la

clean:
	rm -f libuima.la $(libuima_la_OBJECTS)

install-data-local:
if APPLE
	$(INSTALL) -d $(DESTDIR)$(prefix)/data
	$(INSTALL) ../../data/resourceSpecifierSchema.xsd $(DESTDIR)$(prefix)/data/resourceSpecifierSchema.xsd
else
	$(INSTALL) -D ../../data/resourceSpecifierSchema.xsd $(DESTDIR)$(prefix)/data/resourceSpecifierSchema.xsd
endif

INCLUDES  = -I.
INCLUDES += -I../cas
INCLUDES += -I../jni
INCLUDES += $(UIMA_XERCES_INCLUDE)
INCLUDES += $(UIMA_APR_INCLUDE)
INCLUDES += $(UIMA_ICU_INCLUDE)
INCLUDES += $(UIMACPP_HOME_INCLUDE)
INCLUDES += $(UIMA_JDK_INCLUDE)
INCLUDES += $(UIMA_JDK_INCLUDE)/genunix
INCLUDES += $(UIMA_JDK_INCLUDE)/linux

libuima_la_LDFLAGS = -version-info $(LT_VERSION_NUMBER) 
#-release $(LT_RELEASE_NUMBER)
#AM_LDFLAGS = -release $(LIB_VERSION)

if DEBUG_UIMA
AM_CPPFLAGS = -g -DDEBUG
else
AM_CPPFLAGS = -O3 -DNDEBUG -DTRACEOFF
endif

libuima_la_CPPFLAGS = $(AM_CPPFLAGS)
libuima_la_CPPFLAGS += -DUIMA_PREFIX=$(PFX)
libuima_la_CPPFLAGS += -DUIMA_VERSION=$(LIB_VERSION)
libuima_la_CPPFLAGS += -D_UIMA_LIBRARY_
# APR flags from apr_rules.mk
libuima_la_CPPFLAGS += -DLINUX=2 -D_REENTRANT -D_GNU_SOURCE -D_LARGEFILE64_SOURCE

libuima_la_SOURCES  = annotator_context.cpp				   
libuima_la_SOURCES += annotator.cpp					   
libuima_la_SOURCES += annotator_mgr.cpp				   
libuima_la_SOURCES += casdefinition.cpp				   
libuima_la_SOURCES += casiterator.cpp					   
libuima_la_SOURCES += caspool.cpp					   
libuima_la_SOURCES += config_param.cpp				   
libuima_la_SOURCES += consoleui.cpp					   
libuima_la_SOURCES += cp2ucnvrt.cpp					   
libuima_la_SOURCES += dottypesystemwriter.cpp				   
libuima_la_SOURCES += engine.cpp					   
libuima_la_SOURCES += exceptions.cpp					   
libuima_la_SOURCES += ftools.cpp					   
libuima_la_SOURCES += internal_aggregate_engine.cpp			   
libuima_la_SOURCES += internal_capability_container.cpp		   
libuima_la_SOURCES += internal_engine_base.cpp			   
libuima_la_SOURCES += internal_jedii_engine.cpp			   
libuima_la_SOURCES += internal_primitive_engine.cpp			   
libuima_la_SOURCES += language.cpp					   
libuima_la_SOURCES += log.cpp						   
libuima_la_SOURCES += res_abase.cpp					   
libuima_la_SOURCES += res_annotator.cpp				   
libuima_la_SOURCES += resmgr.cpp					   
libuima_la_SOURCES += strtools.cpp					   
libuima_la_SOURCES += taemetadata.cpp					   
libuima_la_SOURCES += taespecifierbuilder.cpp				   
libuima_la_SOURCES += taespecifier.cpp				      
libuima_la_SOURCES += timedatetools.cpp				   
libuima_la_SOURCES += token_properties.cpp				   
libuima_la_SOURCES += typesystemdescription.cpp			   
libuima_la_SOURCES += u2cpcnvrt.cpp					   
libuima_la_SOURCES += unistrref.cpp                                      
libuima_la_SOURCES += ../cas/arrayfs.cpp				       
libuima_la_SOURCES += ../cas/cas.cpp					       
libuima_la_SOURCES += ../cas/casexception.cpp				       
libuima_la_SOURCES += ../cas/caswriter_abase.cpp			       
libuima_la_SOURCES += ../cas/featurestructure.cpp			       
libuima_la_SOURCES += ../cas/fsfilterbuilder.cpp			       
libuima_la_SOURCES += ../cas/fsindex.cpp				       
libuima_la_SOURCES += ../cas/fsindexrepository.cpp			       
libuima_la_SOURCES += ../cas/fsiterator.cpp				       
libuima_la_SOURCES += ../cas/internal_casdeserializer.cpp		       
libuima_la_SOURCES += ../cas/internal_casimpl.cpp			       
libuima_la_SOURCES += ../cas/internal_casserializer.cpp		       
libuima_la_SOURCES += ../cas/internal_fspromoter.cpp			       
libuima_la_SOURCES += ../cas/internal_serializedcas.cpp		       
libuima_la_SOURCES += ../cas/internal_typeshortcuts.cpp		       
libuima_la_SOURCES += ../cas/internal_xmlconstants.cpp		       
libuima_la_SOURCES += ../cas/listfs.cpp				       
libuima_la_SOURCES += ../cas/lowlevel_fsheap.cpp			       
libuima_la_SOURCES += ../cas/lowlevel_indexcomparator.cpp		       
libuima_la_SOURCES += ../cas/lowlevel_index.cpp			       
libuima_la_SOURCES += ../cas/lowlevel_indexdefinition.cpp		       
libuima_la_SOURCES += ../cas/lowlevel_indexrepository.cpp		       
libuima_la_SOURCES += ../cas/lowlevel_internal_indexes.cpp		       
libuima_la_SOURCES += ../cas/lowlevel_typesystem.cpp			       
libuima_la_SOURCES += ../cas/sofastream.cpp				       
libuima_la_SOURCES += ../cas/typenamespace.cpp			       
libuima_la_SOURCES += ../cas/typesystem.cpp				       
libuima_la_SOURCES += ../cas/xcasdeserializer.cpp			       
libuima_la_SOURCES += ../cas/xcasdeserializer_handler.cpp		       
libuima_la_SOURCES += ../cas/xmlerror_handler.cpp			       
libuima_la_SOURCES += ../cas/xmltypesystemreader.cpp			       
libuima_la_SOURCES += ../cas/xmltypesystemwriter.cpp			       
libuima_la_SOURCES += ../cas/xmlwriter.cpp                                   
libuima_la_SOURCES += ../jni/jni.cpp
libuima_la_SOURCES += ../jni/jni_utils.cpp
libuima_la_SOURCES += ../jni/jni_string.cpp
libuima_la_SOURCES += ../cas/xmideserializer.cpp			       
libuima_la_SOURCES += ../cas/xmideserializer_handler.cpp
libuima_la_SOURCES += ../cas/xmiwriter.cpp

#libuima_la_LDFLAGS = -avoid-version -release 2.3.1 -soname libuima.2.3
libuima_la_LIBADD = $(UIMA_XERCES_LIB) $(UIMA_APR_LIB) $(UIMA_ICU_LIB) $(UIMACPP_HOME_LIB)
