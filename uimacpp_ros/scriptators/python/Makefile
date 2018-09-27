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
#SWIGVERSION:=$(subst SWIG Version ,,$(filter SWIG Version $(SWIGMAJORMINOR).% ,$(shell swig -version 2>&1)))

ifeq ($(SWIGVERSION),)
  $(error unable to determine SWIGVERSION looking for $(SWIGMAJOR).$(MAJOR)) Rerun passing in SWIGMAJOR and SWIGMINOR)
endif

SWIGDEPS=uimapywrap.h
SWIGFLAGS=-DSWIG$(subst .,_,$(SWIGVERSION))

PYTHONVERSION:=$(subst ., ,$(subst Python ,,$(shell python -V 2>&1)))
ifeq ($(PYTHONVERSION),)
  $(error PYTHONVERSION could not be determined)
endif
PYTHONDIR=python$(word 1,$(PYTHONVERSION)).$(word 2,$(PYTHONVERSION))
PYTHONLIB=-l$(PYTHONDIR)
PYTHONFLAGS=-DPYTHONLIBRARYNAME=\"lib$(PYTHONDIR).so\" -DPYTHON$(word 1,$(PYTHONVERSION))_$(word 2,$(PYTHONVERSION))

###################################
# This portion was divined by looking at the base.mak file

# name of the annotator to be created
TARGET_FILE=_pythonnator

# list of user's object files to be linked when building the annotator
OBJS=pythonnator.o uima_wrap.o

#Use this var to pass additional user-defined parameters to the compiler
USER_CFLAGS=-g -I/usr/include/$(PYTHONDIR) $(SWIGFLAGS) $(PYTHONFLAGS)

#Use this var to pass additional user-defined parameters to the linker
USER_LINKFLAGS=-g $(PYTHONLIB)

# Define the symbol DEBUG=1 if you want to build a debug version
DEBUG=1

# build a shared library
DLL_BUILD=1

#python bug(?) on MacOSX: PyImport_ImportModule() is looking for .so!
UNAME = $(shell uname -s)
ifeq ($(UNAME), Darwin)
  DLL_SUFFIX=dylib
  USER_XTARGET=$(TARGET_FILE).so
endif

# include file with generic compiler instructions
include $(UIMACPP_HOME)/lib/base.mak
###################################

pythonnator.py uima_wrap.cxx: ../uima.i
	swig -o uima_wrap.cxx -outdir . -c++ -python ../uima.i

uimapywrap.h:
	swig -outdir . -c++ -python -external-runtime $@

pythonnator.o: $(SWIGDEPS)

$(USER_XTARGET): $(TARGET_FILE).$(DLL_SUFFIX)
	ln -s $(TARGET_FILE).$(DLL_SUFFIX) $(USER_XTARGET)

uima_wrap.o: uima_wrap.cxx 
	$(CC) $(CFLAGS) -c $<

distclean: clean
	rm -f uima_wrap.cxx _pythonnator.so pythonnator.py \
		pythonnator.pyc uimapywrap.h
