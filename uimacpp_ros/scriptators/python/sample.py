#!/usr/bin/python 

 # Licensed to the Apache Software Foundation (ASF) under one
 # or more contributor license agreements.  See the NOTICE file
 # distributed with this work for additional information
 # regarding copyright ownership.  The ASF licenses this file
 # to you under the Apache License, Version 2.0 (the
 # "License"); you may not use this file except in compliance
 # with the License.  You may obtain a copy of the License at
 # 
 #   http://www.apache.org/licenses/LICENSE-2.0
 # 
 # Unless required by applicable law or agreed to in writing,
 # software distributed under the License is distributed on an
 # "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 # KIND, either express or implied.  See the License for the
 # specific language governing permissions and limitations
 # under the License.

import re
import math

# everything in the global namespace is eval'ed during initialization
# by the Pythonnator UIMA annotator
def initialize(annotContext):
  global pattern
  global source
  global debug
  global ac
  ac = annotContext
  pattern = re.compile(ac.extractValue("matchString"), re.IGNORECASE)
  source = ac.extractValue("SourceFile")
  debug = ac.extractIntegerValue("DebugLevel")
  if debug > 0:
    print source + ": initialize with matchString =" + ac.extractValue("matchString")


def typeSystemInit(ts):
  global source
  global debug
  global ac
  if debug > 10:
    print source + ": Type sytem init called"
  global keywordtype
  keywordtype =ts.getType('com.ibm.uima.examples.keyword')
  if not keywordtype.isValid():
    error = source + ": com.ibm.uima.examples.keyword is NOT found in type system!"
    ac.logError(error)
    raise Exception, error 

#
# the process method is passed two parameters, the CAS and
# the ResultsSpecification
def process(tcas, rs):
  global keywordtype
  global source
  global debug
  global ac
  if debug > 10:
    print source + ": This is a process function"
    ac.logMessage("process called")
  text = tcas.getDocumentText()
  index = tcas.getIndexRepository()
  iterator = pattern.finditer(text)
  annotCount = 0
  for match in iterator:
    fs = tcas.createAnnotation(keywordtype, match.start(), match.end())
    index.addFS(fs)
    annotCount += 1
  if debug > 0:
    print source + ": created " + str(annotCount) + " annotation"s
  if debug > 20:
    annots = 0
    iterator = tcas.getAnnotationIndex(keywordtype).iterator()
    while iterator.isValid():
      annots += 1
      iterator.moveToNext()
    print source + ": found " + str(annots) + " annotations"
