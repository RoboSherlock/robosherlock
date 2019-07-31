#####################################
# UNIX Makefile for a UIMACPP annotator
#####################################

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

# name of the annotator to be created
TARGET_FILE=XCasWriterCasConsumer

# list of user's object files to be linked when building the annotator
OBJS=XCasWriterCasConsumer.o

#Use this var to pass additional user-defined parameters to the compiler
USER_CFLAGS=

#Use this var to pass additional user-defined parameters to the linker
USER_LINKFLAGS=

# Set DEBUG=1 for a debug build (if not 1 a ship build will result)
DEBUG=1

# Set DLL_BUILD=1 to build an annotator (shared library)
#    if not 1 an executable binary will be built
DLL_BUILD=1

# include file with generic compiler instructions
include $(UIMACPP_HOME)/lib/base.mak
