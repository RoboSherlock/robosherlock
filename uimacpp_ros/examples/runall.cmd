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

@echo on

@setlocal
@set runae=runAECpp
@set exampapp=ExampleApplication
@set sofaexampapp=SofaExampleApplication
@if "%~1" == "debug" (
  @set runae=runAECppD
  @set exampapp=ExampleApplicationD
  @set sofaexampapp=SofaExampleApplicationD
)
%runae% descriptors/DaveDetector.xml data/example.txt
@if not errorlevel 0 goto error
%runae% descriptors/DaveDetector.xml -xmi data/tcas.xmi
@if not errorlevel 0 goto error
%runae% descriptors/DaveDetector.xml -xmi data/sofa.xmi -s EnglishDocument
@if not errorlevel 0 goto error
%runae% -xmi descriptors/SofaExampleAnnotator.xml data/sofa.xmi
@if not errorlevel 0 goto error
set UIMACPP_STREAMHANDLERS=file:SofaStreamHandlerFile
%runae% -xmi descriptors/SofaDataAnnotator.xml data/sofa.xmi
@if not errorlevel 0 goto error
%runae% -xmi descriptors/SofaDataAnnotator.xml data/filetcas.xmi
@if not errorlevel 0 goto error
%runae% -xmi descriptors/SimpleTextSegmenter.xml data/docforsegmenter.xmi
@if not errorlevel 0 goto error
%runae% -xmi descriptors/XCasWriterCasConsumer.xml data/tcas.xmi
@if not errorlevel 0 goto error
%runae% -xmi descriptors/XCasWriterCasConsumer.xml data/sofa.xmi
@if not errorlevel 0 goto error
%exampapp% descriptors/DaveDetector.xml data
@if not errorlevel 0 goto error
%sofaexampapp% descriptors/SofaExampleAnnotator.xml
@if not errorlevel 0 goto error
@echo .................
@echo All examples ran!
@echo .................
:error
