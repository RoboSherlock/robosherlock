@echo off

REM   Licensed to the Apache Software Foundation (ASF) under one
REM   or more contributor license agreements.  See the NOTICE file
REM   distributed with this work for additional information
REM   regarding copyright ownership.  The ASF licenses this file
REM   to you under the Apache License, Version 2.0 (the
REM   "License"); you may not use this file except in compliance
REM   with the License.  You may obtain a copy of the License at
REM
REM    http://www.apache.org/licenses/LICENSE-2.0
REM
REM   Unless required by applicable law or agreed to in writing,
REM   software distributed under the License is distributed on an
REM   "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
REM   KIND, either express or implied.  See the License for the
REM   specific language governing permissions and limitations
REM   under the License.

setlocal

if "%UIMACPP_HOME%"=="" (
  set UIMACPP_HOME=..\..\install
)

REM default location of descriptors 
if "%UIMACPP_DATAPATH%"=="" set UIMACPP_DATAPATH=.\data

REM add location of apr, icu, xerces and uimacpp binaries to the path 
set PATH=..\..\install\bin;%APR_HOME%\Release;%ICU_HOME%\bin;%XERCES_HOME%\bin;%UIMACPP_HOME%\bin;%PATH%

set RM=\windows\system32\cmd /C del

if "%~2"=="debug" (
  set DEBUG_FVT="devenv /debugexe "
)
if "%UIMACPP_STREAMHANDLERS%"=="" set UIMACPP_STREAMHANDLERS=file:SofaStreamHandlerFile

set UIMACPPTEST_JNI=call "%UIMA_HOME%\bin\runAE" data\descriptors\javaaggregate.xml data\segmenterinput

if "%UIMA_HOME%" == "" (
	set UIMACPPTEST_JNI=@echo "UIMA_HOME is not set. The JNI test was not run."
)

if "%~1" == "debug" (
  nmake -f fvtTestfile.debug %2
)
if "%~2" == "debug" (
  nmake -f fvtTestfile.debug %1
)
if not "%~2" == "debug" (
  if not "%~1" == "debug" (
    nmake -f fvtTestfile %1
  )
)

goto end

:end
