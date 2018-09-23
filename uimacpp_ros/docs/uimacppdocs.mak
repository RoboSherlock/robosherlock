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

#This makefile builds the UIMACPP documentation
#Requires Doxygen 1.3.6
#PATH must include doxygen command
#Must be run from the uimacpp/docs subdirectory

DOXYGENCMD=doxygen

build : ../docs/html/index.html
	
../docs/html/index.html : uimacpp.dox
	@echo building docs....  
	$(DOXYGENCMD) uimacpp.dox
	$(DEL) $(MDFILES)
	$(DEL) $(MAPFILES)
	$(DEL) $(DOTFILES)
# can't get doxygen to build correctly without graphical hierarchy
	$(CP) $(HTMLDIR)hierarchy.html $(HTMLDIR)inherits.html

rebuild : clean build

clean : 
	@echo deleting $(DOCDIR)/html 
	$(RD) $(HTMLDIR)
	@echo deleting tags file
	$(DEL) uimadoxytags.tag

