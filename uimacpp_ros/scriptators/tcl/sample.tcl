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

proc initialize {annotContext} {
  global matchString thisScript ac debug
  AnnotatorContext ac -this $annotContext
  set matchString [ac extractValue "matchString"]
  set thisScript [ac extractValue "SourceFile"]
  set debug [ac extractIntegerValue "DebugLevel"]
  if {$debug > 0} then {
    puts "$thisScript: initialize match string = $matchString"
  }
}

proc typeSystemInit {ts} {
  TypeSystem ts -this $ts
  global keywordtype thisScript ac debug
  if {$debug > 10} then {
    puts "$thisScript: Type sytem init called"
  }
  set keywordtype [ts getType "com.ibm.uima.examples.keyword"]
  if {![$keywordtype isValid]} then {
    set error "$thisScript: com.ibm.uima.examples.keyword NOT found in type system\n"
    ac logError "$error"
    error "$error"
  }
}

#
# the process method is passed two parameters, the CAS and
# the ResultsSpecification
proc process {cas rs} {
  global keywordtype thisScript matchString debug
  CAS cas -this $cas
  if {$debug > 10} then {
    puts "$thisScript: This is a process function"
  }

  set text [cas getDocumentText]
  set indexRep [cas getIndexRepository]
  set total 0
  set matches [regexp -nocase -indices -all -inline -- $matchString $text]
  foreach pair $matches {
    set begin [lindex $pair 0]
    set end [expr [lindex $pair 1]+1]
    set fs [cas createAnnotation $keywordtype $begin $end]
    $indexRep addFS $fs
    $fs -delete
    incr total
  }
  $indexRep -delete
  if {$debug > 0} then {
    puts "$thisScript: created $total annotations"
  }
  if {$debug > 20} then {
    set annots 0
    set anIndex [cas getAnnotationIndex $keywordtype]
    set iterator [$anIndex iterator]
    while {[$iterator isValid]} {
      incr annots
      $iterator moveToNext
    }
    puts "$thisScript: found $annots annotations"
    $anIndex -delete
    $iterator -delete
  }
}
