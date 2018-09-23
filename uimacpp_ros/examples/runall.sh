
#   Licensed to the Apache Software Foundation (ASF) under one
#   or more contributor license agreements.  See the NOTICE file
#   distributed with this work for additional information
#   regarding copyright ownership.  The ASF licenses this file
#   to you under the Apache License, Version 2.0 (the
#   "License"); you may not use this file except in compliance
#   with the License.  You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing,
#   software distributed under the License is distributed on an
#   "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#   KIND, either express or implied.  See the License for the
#   specific language governing permissions and limitations
#   under the License.

echo .
echo runAECpp descriptors/DaveDetector.xml data/example.txt
runAECpp descriptors/DaveDetector.xml data/example.txt || exit 99
echo .
echo runAECpp descriptors/DaveDetector.xml -x data/tcas.xmi
runAECpp descriptors/DaveDetector.xml -xmi data/tcas.xmi || exit 99
echo .
echo runAECpp descriptors/DaveDetector.xml -x data/sofa.xmi -s EnglishDocument
runAECpp descriptors/DaveDetector.xml -xmi data/sofa.xmi -s EnglishDocument || exit 99
echo .
echo runAECpp -x descriptors/SofaExampleAnnotator.xml data/sofa.xmi
runAECpp -xmi descriptors/SofaExampleAnnotator.xml data/sofa.xmi || exit 99
echo .
echo export UIMACPP_STREAMHANDLERS=file:SofaStreamHandlerFile
export UIMACPP_STREAMHANDLERS=file:SofaStreamHandlerFile || exit 99
echo .
echo runAECpp -xmi descriptors/SofaDataAnnotator.xml data/sofa.xmi
runAECpp -xmi descriptors/SofaDataAnnotator.xml data/sofa.xmi || exit 99
echo .
echo runAECpp -xmi descriptors/SofaDataAnnotator.xml data/filetcas.xmi
runAECpp -xmi descriptors/SofaDataAnnotator.xml data/filetcas.xmi || exit 99
echo .
echo runAECpp -xmi descriptors/SimpleTextSegmenter.xml data/docforsegmenter.xmi
runAECpp -xmi descriptors/SimpleTextSegmenter.xml data/docforsegmenter.xmi || exit 99
echo .
echo runAECpp -xmi descriptors/XCasWriterCasConsumer.xml data/tcas.xmi
runAECpp -xmi descriptors/XCasWriterCasConsumer.xml data/tcas.xmi || exit 99
echo .
echo runAECpp -xmi descriptors/XCasWriterCasConsumer.xml data/sofa.xmi
runAECpp -xmi descriptors/XCasWriterCasConsumer.xml data/sofa.xmi || exit 99
echo .
echo src/ExampleApplication descriptors/DaveDetector.xml data
src/ExampleApplication descriptors/DaveDetector.xml data || exit 99
echo .
echo src/SofaExampleApplication descriptors/SofaExampleAnnotator.xml
src/SofaExampleApplication descriptors/SofaExampleAnnotator.xml || exit 99
echo .................
echo All examples ran!
echo .................
