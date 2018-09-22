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

ifeq ($(UIMACPP_HOME),)
  $(error UIMACPP_HOME not set)
endif

SWIGVERSION:=$(shell swig -version  | grep SWIG | sed "s/SWIG Version //" )
#SWIGVERSION:=$(subst SWIG Version ,,$(filter SWIG Version 1.3.%,$(shell swig -version 2>&1)))

ifeq ($(SWIGVERSION),)
  $(error unable to determine SWIGVERSION)
endif

SWIGDEPS=uimatclwrap.h
SWIGFLAGS=-DSWIG$(subst .,_,$(SWIGVERSION))


###################################
# This portion was divined by looking at the base.mak file

# name of the annotator to be created
TARGET_FILE=tclator

# list of user's object files to be linked when building the annotator
OBJS=Tclator.o uima_wrap.o

#Use this var to pass additional user-defined parameters to the compiler
USER_CFLAGS=-fPIC -g $(SWIGFLAGS) -I..

#Use this var to pass additional user-defined parameters to the linker
USER_LINKFLAGS=-g -ltcl8.4 -lpthread

# Define the symbol DEBUG=1 if you want to build a debug version
DEBUG=1

# build a shared library
DLL_BUILD=1

# include file with generic compiler instructions
include $(UIMACPP_HOME)/lib/base.mak
###################################

test:
	echo "$(SWIGVERSION)"
	echo done

tclator.pm uima_wrap.cxx: ../uima.i
	swig -o uima_wrap.cxx -outdir . -c++ -tcl ../uima.i

uimatclwrap.h:
	swig -outdir . -c++ -tcl -external-runtime $@

Tclator.o: $(SWIGDEPS)

uima_wrap.o: uima_wrap.cxx
	$(CC) $(CFLAGS) -c $<

distclean: clean
	rm -f uima_wrap.cxx uimatclwrap.h 
