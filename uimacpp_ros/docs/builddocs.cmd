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

@REM  Builds the UIMACPP doxygen docs.
@REM	Requires Doxygen 1.3.6 and Graphviz 1.8.10 installed and 
@REM  PATH environment variable must include
@REM	doxygen\bin;graphviz\bin;graphviz\bin\tools
@REM  This script must be run from the uimacpp\docs subdirectory

@if "%~1"=="" goto build
@if "%~1"=="clean" goto clean
@if "%~1"=="rebuild" goto rebuild

:build
	@echo building uimacpp docs in ..\docs
	nmake -f uimacppdocs.mak build CP=copy DEL=DEL RD="RM /s /q" MDFILES=html\*.md5  MAPFILES=html\*.map DOTFILES=html\*.dot HTMLDIR=.\html\
	goto TheEnd

:rebuild
	@echo rebuilding uimacpp docs in ..\docs
	nmake -f uimacppdocs.mak rebuild CP=copy DEL=DEL RD="RM /s /q" MDFILES=html\*.md5  MAPFILES=html\*.map DOTFILES=html\*.dot HTMLDIR=.\html\
	goto TheEnd

:clean
	@echo cleaning uimacpp docs in ..\docs
	nmake -f uimacppdocs.mak clean RD="RM /s /q" HTMLDIR=.\html\
	goto TheEnd

:TheEnd
	@echo done 

