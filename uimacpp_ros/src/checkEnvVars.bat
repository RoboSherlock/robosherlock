@ECHO OFF

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

@REM Called as a pre-build step to check some environment variables
@REM UIMACPP_HOME is used as the default location of the apr & icu & xerces builds
@REM which are include in the SDK
@REM These may be overridden by setting APR_HOME or ICU_HOME or XERCES_HOME

if not exist "%JAVA_HOME%\include\jni.h" (
  echo Error: failed to find JDK headers in "%JAVA_HOME%"
  goto end
)

if "%UIMACPP_HOME%"=="" goto noTAF
if "%APR_HOME%"=="" echo Note: Using APR build under UIMACPP_HOME
if "%ICU_HOME%"=="" echo Note: Using ICU build under UIMACPP_HOME
if "%XERCES_HOME%"=="" echo Note: Using XERCES build under UIMACPP_HOME
goto end

:noTAF
if "%APR_HOME%"=="" (
  echo Error: Must set APR_HOME [or use SDK via UIMACPP_HOME]
) else (
  if "%ICU_HOME%"=="" (
    echo Error: Must set ICU_HOME [or use SDK via UIMACPP_HOME]
  ) else (
    if "%XERCES_HOME%"=="" (
      echo Error: Must set XERCES_HOME [or use SDK via UIMACPP_HOME]
    ) else ( goto end ) 
  )
)
echo **** Pre-build check failed ***
exit 99
:end
