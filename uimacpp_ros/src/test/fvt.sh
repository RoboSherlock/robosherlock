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

if [ ! $UIMACPP_HOME ]
then
   export UIMACPP_HOME=../..
fi

 Testlib=.libs:../framework/.libs
 Testbin=.libs

export LD_LIBRARY_PATH=$Testlib:$APR_HOME/lib:$ICU_HOME/lib:$XERCES_HOME/lib:$UIMACPP_HOME/lib
export DYLD_LIBRARY_PATH=$Testlib:$APR_HOME/lib:$ICU_HOME/lib:$XERCES_HOME/lib:$UIMACPP_HOME/lib
export PATH=$Testbin:$PATH

export UIMACPP_DATAPATH=./data

if [ ! $TEMP ]
then
  export TEMP=/tmp/$USER
  mkdir -p $TEMP
fi

if [ "$2" = "debug" ]; then
  export DEBUG_FVT="gdb $Testbin/"
fi
if [ ! $UIMACPP_STREAMHANDLERS ]
then
   export UIMACPP_STREAMHANDLERS=file:libSofaStreamHandlerFile
fi

export RM=rm
export UIMACPPTEST_JNI="$UIMA_HOME/bin/runAE.sh data/descriptors/javaaggregate.xml data/segmenterinput"
if [ ! $UIMA_HOME ] 
then
  export UIMACPPTEST_JNI="@echo UIMA_HOME is not set. JNI test was not run."
fi

make -f fvtTestfile $1




