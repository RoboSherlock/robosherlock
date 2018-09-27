/* The following was once generated from msg ... now maintained by hand! */
/*   >>>>>                                                        <<<<<  */
/*   >>>>>   MUST MUST be kept in sync with msg.h                 <<<<<  */
/*   >>>>>                                                        <<<<<  */

/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

/*lint -e786 // String concatenation within initializer */

#ifndef UIMA_MSGSTRTAB_H
#define UIMA_MSGSTRTAB_H

static const TCHAR * gs_aszMessageStringTable[] = {
      /*    0 - number of elements in message string table (not used) */
      "332",
      /*    1 - UIMA_MSG_ID_SIGNATURE_BEGIN: */
      "[UIMA-LIBRARY]",
      /*    2 - UIMA_MSG_ID_LITERAL_STRING: */
      "%1",
      /*    3 - UIMA_MSG_ID_NO_MESSAGE_AVAILABLE: */
      "No detailed message available",
      /*    4 - UIMA_MSG_ID_EXC_UNEXPECTED_ERROR: */
      "Unexpected error",
      /*    5 - UIMA_MSG_ID_EXC_OUT_OF_MEMORY: */
      "Out of memory",
      /*    6 - UIMA_MSG_ID_EXC_RESOURCE_KEY: */
      "Resource key",
      /*    7 - UIMA_MSG_ID_EXC_INVALID_ITERATOR: */
      "Invalid iterator",
      /*    8 - UIMA_MSG_ID_EXC_DOCUMENT_INVALID_IDX: */
      "Invalid document index",
      /*    9 - UIMA_MSG_ID_EXC_DOCUMENT_EMPTY_REF: */
      "Illegal empty document reference",
      /*   10 - UIMA_MSG_ID_EXC_DOCUMENT_INVAL_COPY_OP: */
      "Invalid copy request",
      /*   11 - UIMA_MSG_ID_EXC_CPCONVERSION_ILLEGAL_CALL: */
      "Illegal call to invalid conversion object",
      /*   12 - UIMA_MSG_ID_EXC_CPCONVERSION_OVERFLOW: */
      "Target buffer overflow",
      /*   13 - UIMA_MSG_ID_EXC_CPCONVERSION_INVALID_SRCSEQ: */
      "Invalid character sequence",
      /*   14 - UIMA_MSG_ID_EXC_CPCONVERSION_INVALID_DBCS_SRC: */
      "Invalid DBCS character sequence",
      /*   15 - UIMA_MSG_ID_EXC_CPCONVERSION_NOT_SUPPORTED: */
      "Conversion not supported",
      /*   16 - UIMA_MSG_ID_EXC_ANNOTATORMGR_INVAL_ANNOTATOR_REQ: */
      "Invalid annotator",
      /*   17 - UIMA_MSG_ID_EXC_ENUMERATION_OVERFLOW: */
      "Enumeration overflow",
      /*   18 - UIMA_MSG_ID_EXC_WINDOWS_EXCEPTION: */
      "Windows exception %1",
      /*   19 - UIMA_MSG_ID_EXC_XML_SAXPARSE_WARNING: */
      "XML parse warning in %1 at line %2 character %3: %4",
      /*   20 - UIMA_MSG_ID_EXC_XML_SAXPARSE_ERROR: */
      "XML parse error in %1 at line %2 character %3 %4",
      /*   21 - UIMA_MSG_ID_EXC_XML_SAXPARSE_FATALERROR: */
      "XML parse fatal error in %1 at line %2 character %3: %4",
      /*   22 - UIMA_MSG_ID_EXC_XML_XMLEXCEPTION: */
      "Generic XML error",
      /*   23 - UIMA_MSG_ID_EXC_INVALID_LANGUAGE: */
      "'%1' is not a valid ISO language identifier",
      /*   24 - UIMA_MSG_ID_ANNOTATOR_COULD_NOT_FIND: */
      "Could not find annotator '%2'. '%1'",
      /*   25 - UIMA_MSG_ID_ANNOTATOR_COULD_NOT_FIND_MAKEAE: */
      "Could not find MakeAE procedure in '%1'",
      /*   26 - UIMA_MSG_ID_ANNOTATOR_COULD_NOT_LOAD: */
      "Error loading annotator '%2'. '%1'",
      /*   27 - UIMA_MSG_ID_ANNOTATOR_COULD_NOT_INITIALIZE: */
      "Annotator %1 could not be initialized successfully",
      /*   28 - UIMA_MSG_ID_EXC_UNKNOWN_JAVA_EXCEPTION: */
      "An unexpected Java exception occurred, error message: %1",
      /*   29 - UIMA_MSG_ID_EXC_JAVA_EXCEPTION: */
      "A Java exception occurred. JNI return code: %1, error message: %2",
      /*   30 - UIMA_MSG_ID_EXC_JNI_CALL_FAILED: */
      "A JNI call failed with return code %1",
      /*   31 - UIMA_MSG_ID_EXC_JAVA_VM_COULD_NOT_BE_CREATED: */
      "The Java VM could not be created (error code: %1)",
      /*   32 - UIMA_MSG_ID_EXC_COULD_NOT_LOAD_JAVA_DLL: */
      "The Java library could not be found or loaded",
      /*   33 - UIMA_MSG_ID_EXC_INCOMPATIBLE_CAS: */
      "CAS definitions are incompatible with this AnalysisEngine",
      /*   34 - UIMA_MSG_ID_EXC_NO_CAS: */
      "CAS object could not be converted to a TCAS (because it is not a TCAS)",
      /*   35 - UIMA_MSG_ID_RESMGR_DATADIR_DOES_NOT_EXIST: */
      "Data directory '%1' does not exist",
      /*   36 - UIMA_MSG_ID_RESMGR_WORKDIR_DOES_NOT_EXIST: */
      "Working directory '%1' does not exist",
      /*   37 - UIMA_MSG_ID_EXC_INVALID_FS_OBJECT: */
      "Invalid FeatureStructure object",
      /*   38 - UIMA_MSG_ID_EXC_INVALID_FSTYPE_OBJECT: */
      "Invalid Type object",
      /*   39 - UIMA_MSG_ID_EXC_INVALID_FSFEATURE_OBJECT: */
      "Invalid Feature object",
      /*   40 - UIMA_MSG_ID_EXC_FEATURE_NOT_APPROPRIATE: */
      "Feature '%1' not appropriate for type '%2'",
      /*   41 - UIMA_MSG_ID_EXC_INCOMPATIBLE_RANGE_TYPE: */
      "Incompatible range type '%2' for requested feature '%1'",
      /*   42 - UIMA_MSG_ID_EXC_INVALID_INDEX_OBJECT: */
      "Invalid Index object",
      /*   43 - UIMA_MSG_ID_EXC_FS_IS_NOT_STRING: */
      "Feature structure is not a string",
      /*   44 - UIMA_MSG_ID_EXC_TYPESYSTEM_ALREADY_COMMITTED: */
      "Type system is already committed",
      /*   45 - UIMA_MSG_ID_EXC_WRONG_FSTYPE: */
      "Wrong type for feature structure",
      /*   46 - UIMA_MSG_ID_EXC_LIST_IS_EMPTY: */
      "List is empty",
      /*   47 - UIMA_MSG_ID_EXC_LIST_IS_CIRCULAR: */
      "List is circular",
      /*   48 - UIMA_MSG_ID_EXC_FS_IS_NOT_LIST: */
      "Feature structure is not of type list",
      /*   49 - UIMA_MSG_ID_EXC_TYPESYSTEM_NOT_YET_COMMITTED: */
      "Type system is not yet committed",
      /*   50 - UIMA_MSG_ID_EXC_INDEX_ALREADY_EXISTS: */
      "Index with specified ID already exists",
      /*   51 - UIMA_MSG_ID_EXC_WRONG_FSTYPE_FOR_INDEX: */
      "Index does not contain feature structures with the requested type",
      /*   52 - UIMA_MSG_ID_EXC_INVALID_INDEX_ID: */
      "Index ID is invalid",
      /*   53 - UIMA_MSG_ID_EXC_XMLTYPESYSTEMREADER: */
      "Error while reading type system XML file",
      /*   54 - UIMA_MSG_ID_EXC_WRONG_XML_TYPESYSTEM_FORMAT: */
      "Wrong XML type system format near string '%1'",
      /*   55 - UIMA_MSG_ID_EXC_INVALID_INTRO_TYPE: */
      "Invalid intro type name '%2' for feature '%1'",
      /*   56 - UIMA_MSG_ID_EXC_INVALID_RANGE_TYPE: */
      "Invalid range type name '%2' for feature '%1'",
      /*   57 - UIMA_MSG_ID_EXC_WRONG_PARENT_TYPE: */
      "Type '%1' already exists but has different parent type",
      /*   58 - UIMA_MSG_ID_EXC_FS_IS_NOT_ARRAY: */
      "Feature structure is not an array",
      /*   59 - UIMA_MSG_ID_EXC_ARRAY_OUT_OF_BOUNDS: */
      "FS Array index %1 out of bounds (array size is %2)",
      /*   60 - UIMA_MSG_ID_EXC_NOT_YET_IMPLEMENTED: */
      "Function not yet implemented",
      /*   61 - UIMA_MSG_ID_EXC_TYPE_CREATION_FAILED_FINAL_TYPE: */
      "Creation of type failed (parent type is not allowed to have new subtyp"
      "es)",
      /*   62 - UIMA_MSG_ID_EXC_FEATURE_INTRO_FAILED_FINAL_TYPE: */
      "Creation of feature failed (introduction type is not allowd to have ne"
      "w features)",
      /*   63 - UIMA_MSG_ID_EXC_TYPE_ALREADY_EXISTS: */
      "Type already exists",
      /*   64 - UIMA_MSG_ID_EXC_FEATURE_ALREADY_EXISTS: */
      "Feature already exists",
      /*   65 - UIMA_MSG_ID_EXC_COULD_NOT_CREATE_FS_FINAL_TYPE: */
      "Could not create feature structure of type '%1' (type is final)",
      /*   66 - UIMA_MSG_ID_EXC_UNKNOWN_TYPE_NAME: */
      "A type with name '%1' does not exist in the type system",
      /*   67 - UIMA_MSG_ID_EXC_UNKNOWN_FEATURE_NAME: */
      "A feature with name '%1' does not exist in the type system",
      /*   68 - UIMA_MSG_ID_EXC_WRONG_STRING_VALUE: */
      "The string value '%1' is not allowed for the string sub type '%2'",
      /*   69 - UIMA_MSG_ID_EXC_ALLOWED_STRING_VALUES_INCOMPATIBLE: */
      "There are conflicting allowed string values for type '%1'.",
      /*   70 - UIMA_MSG_ID_EXC_TYPE_PRIORITY_CONFLICT: */
      "The specified type priority ('%1', '%2') raises a conflict.",
      /*   71 - UIMA_MSG_ID_EXC_INCOMPATIBLE_INDEX_DEFINITIONS: */
      "There are incompatible index definitions for index with ID '%1'",
      /*   72 - UIMA_MSG_ID_EXC_WRONG_DESERIALIZED_DATA: */
      "Deserialized CAS corrupted (line %1)",
      /*   73 - UIMA_MSG_ID_EXC_SOFA_NAME_ALREADY_EXISTS: */
      "SofaFS with name '%1' previously instantiated in the CAS",
      /*   74 - UIMA_MSG_ID_EXC_DUPLICATE_INDEX_LABEL: */
      "Index label '%1' occurs more than once",
      /*   75 - UIMA_MSG_ID_EXC_DUPLICATE_TYPE_NAME: */
      "Type name '%1' is defined more than once, with different supertype nam"
      "es.",
      /*   76 - UIMA_MSG_ID_EXC_DUPLICATE_FEATURE_NAME: */
      "Type '%1' contains more than one feature named '%2'. The features have"
      " different range type names.",
      /*   77 - UIMA_MSG_ID_EXC_UNKNOWN_CONFIG_XML_TAG: */
      "Tag '%1' is unknown. There is a mismatch between the constants in "
      "taespecifierbuilder.hpp and the XML Schema file '%2'.",
      /*   78 - UIMA_MSG_ID_EXC_CONFIG_XML_ATTRIBUTE_VALUE_NOT_ALLOWED: */
      "Value '%1' of XML attribute '%2' in tag '%3' is not allowed.",
      /*   79 - UIMA_MSG_ID_UNKNOWN_TYPE_IN_CAPBILITY_SPEC: */
      "The type '%1' used in the capability specification is not defined.",
      /*   80 - UIMA_MSG_ID_UNKNOWN_FEATURE_IN_CAPBILITY_SPEC: */
      "The feature '%1' used in the capability specification is not defined.",
      /*   81 - UIMA_MSG_ID_EXC_NO_VALUE_FOR_MANDATORY_PARAM: */
      "No value for the mandatory parameter '%1' was found",
      /*   82 - UIMA_MSG_ID_EXC_NO_VALUE_FOR_MANDATORY_PARAM_IN_GROUP: */
      "No value for the mandatory parameter '%2' was found in group '%1'",
      /*   83 - UIMA_MSG_ID_EXC_CONFIG_DUPLICATE_CONFIG_PARAM: */
      "Configuration parameter '%1' is already defined in this specifier",
      /*   84 - UIMA_MSG_ID_EXC_CONFIG_DUPLICATE_NAME_VALUE_PAIR: */
      "A nameValuePair for parameter '%1' already exists",
      /*   85 - UIMA_MSG_ID_EXC_CONFIG_DUPLICATE_GROUP: */
      "The group '%1' already exists",
      /*   86 - UIMA_MSG_ID_EXC_ALLOWEDVAL_DEF_FOR_NONSTRINGTYPE: */
      "Allowed values are defined on type '%1' which is not a subtype of uima"
      ".cas.String",
      /*   87 - UIMA_MSG_ID_EXC_XINCLUSION_IS_CYCLIC: */
      "XInclusion of file '%1' is cyclic",
      /*   88 - UIMA_MSG_ID_EXC_INVALID_XINCLUDE_TAG: */
      "The xi:include tag does not comply to standard (one href attribute)",
      /*   89 - UIMA_MSG_ID_EXC_DUPLICATE_ALLOWED_VALUE: */
      "Type '%1' contains more than one allowed value named '%2'.",
      /*   90 - UIMA_MSG_ID_EXC_CONFIG_VALUE_TYPE_MISMATCH: */
      "Wrong type for configuration value",
      /*   91 - UIMA_MSG_ID_EXC_CONFIG_VALUE_MUST_BE_SINGLE: */
      "Configuration value is used as multi-value but defined as single value",
      /*   92 - UIMA_MSG_ID_EXC_CONFIG_VALUE_MUST_BE_MULTI: */
      "Configuration value is used as single value but defined as multi-value",
      /*   93 - UIMA_MSG_ID_EXCON_CONFIG_VALUE_EXTRACT: */
      "Extracting a configuration value",
      /*   94 - UIMA_MSG_ID_EXC_CONFIG_PARAM_NOT_DEFINED: */
      "The configuration parameter '%1' is not defined",
      /*   95 - UIMA_MSG_ID_EXC_CONFIG_PARAM_NOT_DEFINED_IN_GROUP: */
      "The configuration parameter '%1' is not defined in configuration group"
      " '%2'",
      /*   96 - UIMA_MSG_ID_EXC_CONFIG_NO_DEFAULT_GROUP_DEFINED: */
      "No default configuration group defined",
      /*   97 - UIMA_MSG_ID_EXC_CONFIG_NO_GROUPS_DEFINED: */
      "No configuration groups defined",
      /*   98 - UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC_FROM_FILE: */
      "Building TextAnalysisEngineSpecifier from file '%1'",
      /*   99 - UIMA_MSG_ID_EXCON_BUILD_TAE_SPEC: */
      "Building TextAnalysisEngineSpecifier",
      /*  100 - UIMA_MSG_ID_EXCON_VALIDATE_TAE_SPEC_FROM_FILE: */
      "Validating TextAnalysisEngineSpecifier from file '%1'",
      /*  101 - UIMA_MSG_ID_EXCON_VALIDATE_TAE_SPEC: */
      "Validating TextAnalysisEngineSpecifier",
      /*  102 - UIMA_MSG_ID_EXCON_CONFIG_PARAM_SEARCH: */
      "Looking for configuration parameter '%1'",
      /*  103 - UIMA_MSG_ID_EXCON_CONFIG_PARAM_IN_GROUP_SEARCH: */
      "Looking for configuration parameter '%1' in group '%2'",
      /*  104 - UIMA_MSG_ID_EXCON_RESOLVING_XINCLUDES: */
      "Resolving xi:includes",
      /*  105 - UIMA_MSG_ID_EXCON_UNKNOWN_CONTEXT: */
      "Unknown context",
      /*  106 - UIMA_MSG_ID_EXCON_ITERATOR_ACCESS: */
      "Accessing/Dereferencing Iterator",
      /*  107 - UIMA_MSG_ID_EXCON_DOCUMENT_INVALID_IDX: */
      "Requesting document text at invalid index '%1'",
      /*  108 - UIMA_MSG_ID_EXCON_CPCONVERSION: */
      "Converting from CCSID %1 to CCSID %2",
      /*  109 - UIMA_MSG_ID_EXCON_ANNOTATORMGR_INVAL_ANNOTATOR_REQ: */
      "Requesting non-existent annotator '%1'",
      /*  110 - UIMA_MSG_ID_EXCON_ENUMERATION_OVERFLOW: */
      "Enumerating supported languages",
      /*  111 - UIMA_MSG_ID_EXCON_ALLOCATE_MEMPOOL_INST: */
      "Allocating from memory pool: instance",
      /*  112 - UIMA_MSG_ID_EXCON_ALLOCATE_MEMPOOL_DOC_BUF: */
      "Allocating from memory pool: document buffer",
      /*  113 - UIMA_MSG_ID_EXCON_ALLOCATE_MEMPOOL_DOC_MISC: */
      "Allocating from memory pool: document misc.",
      /*  114 - UIMA_MSG_ID_EXCON_EXECUTING_ANNOTATOR: */
      "Executing annotator '%1'",
      /*  115 - UIMA_MSG_ID_EXCON_CALLING_ANNOTATOR_FUNCTION: */
      "Calling annotator function '%1'",
      /*  116 - UIMA_MSG_ID_EXCON_GENERIC_PARAM: */
      "Working on '%1'",
      /*  117 - UIMA_MSG_ID_EXCON_NO_RES_FOR_LANGUAGE: */
      "No resource found for language '%1'",
      /*  118 - UIMA_MSG_ID_EXCON_PARSING_TAGGED_DOCUMENT: */
      "Parsing XML or HTML document",
      /*  119 - UIMA_MSG_ID_EXCON_CHECKING_CAPABILITY_SPEC: */
      "Checking capability specification from configuration file '%1'",
      /*  120 - UIMA_MSG_ID_EXCON_CHECKING_INDEX_DEFINITION: */
      "Checking index specification of configuration",
      /*  121 - UIMA_MSG_ID_EXCON_PROCESSING_JAVA_TAE: */
      "Processing Java TAE from UIMACPP",
      /*  122 - UIMA_MSG_ID_EXCON_PROCESSING_CAS: */
      "Processing a CAS",
      /*  123 - UIMA_MSG_ID_EXCON_PROMOTING_CAS: */
      "Converting a CAS to a TCAS",
      /*  124 - UIMA_MSG_ID_EXCON_CREATING_FS: */
      "Creating feature structure",
      /*  125 - UIMA_MSG_ID_EXCON_GETTING_INDEX: */
      "Getting index with id %1",
      /*  126 - UIMA_MSG_ID_EXCON_CREATING_SET_INDEX: */
      "Creating set index with id %1",
      /*  127 - UIMA_MSG_ID_EXCON_CREATING_ORDERED_INDEX: */
      "Creating ordered index with id %1",
      /*  128 - UIMA_MSG_ID_EXCON_CHECKING_INDEX_ID: */
      "Checking index id %1",
      /*  129 - UIMA_MSG_ID_EXCON_ADDING_FS_TO_INDEX: */
      "Adding feature structure to index",
      /*  130 - UIMA_MSG_ID_EXCON_FINDING_FS_IN_INDEX: */
      "Finding feature structure in index",
      /*  131 - UIMA_MSG_ID_EXCON_REMOVING_FS_FROM_INDEX: */
      "Removing feature structure from index",
      /*  132 - UIMA_MSG_ID_EXCON_INTRODUCING_FEATURE: */
      "Introducing new feature '%1'",
      /*  133 - UIMA_MSG_ID_EXCON_ADDING_TYPE: */
      "Adding new type '%1'",
      /*  134 - UIMA_MSG_ID_EXCON_SETTING_STRINGVALUE_IN_FS: */
      "Setting string value in feature structure",
      /*  135 - UIMA_MSG_ID_EXCON_GETTING_STRINGVALUE_FROM_FS: */
      "Getting string value from feature structure",
      /*  136 - UIMA_MSG_ID_EXCON_GETTING_ARRAYDATA_FROM_FS: */
      "Getting array data from feature structure",
      /*  137 - UIMA_MSG_ID_EXCON_GETTING_ARRAYSIZE_FROM_FS: */
      "Getting array size from feature structure",
      /*  138 - UIMA_MSG_ID_EXCON_GETTING_FEATURE_VALUE: */
      "Getting value of a feature",
      /*  139 - UIMA_MSG_ID_EXCON_SETTING_FEATURE_VALUE: */
      "Setting value of a feature",
      /*  140 - UIMA_MSG_ID_EXCON_INITIALIZING_KEYFEATURECOMPARATOR: */
      "Initializing KeyFeatureComparator",
      /*  141 - UIMA_MSG_ID_EXCON_COMPARING_FSS: */
      "Comparing feature structures",
      /*  142 - UIMA_MSG_ID_EXCON_GETTING_FSTYPE: */
      "Getting the type of a feature structures",
      /*  143 - UIMA_MSG_ID_EXCON_CREATING_LISTFS: */
      "Creating list feature structure",
      /*  144 - UIMA_MSG_ID_EXCON_APPENDING_TO_LIST: */
      "Appending to a list",
      /*  145 - UIMA_MSG_ID_EXCON_PREPENDING_TO_LIST: */
      "Prepending to a list",
      /*  146 - UIMA_MSG_ID_EXCON_GETTING_LIST_LENGTH: */
      "Getting list length",
      /*  147 - UIMA_MSG_ID_EXCON_GETTING_FIRST_LIST_ELEMENT: */
      "Getting first list element",
      /*  148 - UIMA_MSG_ID_EXCON_GETTING_LAST_LIST_ELEMENT: */
      "Getting last list element",
      /*  149 - UIMA_MSG_ID_EXCON_SETTING_FIRST_LIST_ELEMENT: */
      "Setting first list element",
      /*  150 - UIMA_MSG_ID_EXCON_SETTING_LAST_LIST_ELEMENT: */
      "Setting last list element",
      /*  151 - UIMA_MSG_ID_EXCON_GETTING_LIST_ISEMPTY: */
      "Checking if list is empty",
      /*  152 - UIMA_MSG_ID_EXCON_GETTING_LIST_HEAD: */
      "Getting list head feature",
      /*  153 - UIMA_MSG_ID_EXCON_SETTING_LIST_HEAD: */
      "Setting list head feature",
      /*  154 - UIMA_MSG_ID_EXCON_GETTING_LIST_TAIL: */
      "Getting list tail feature",
      /*  155 - UIMA_MSG_ID_EXCON_SETTING_LIST_TAIL: */
      "Setting list tail feature",
      /*  156 - UIMA_MSG_ID_EXCON_ADDING_LIST_VALUE: */
      "Adding list value",
      /*  157 - UIMA_MSG_ID_EXCON_MOVING_LIST_TO_NEXT: */
      "Moving list to next element",
      /*  158 - UIMA_MSG_ID_EXCON_CREATING_ARRAYFS: */
      "Creating array feature structure",
      /*  159 - UIMA_MSG_ID_EXCON_GETTING_FS_FROM_ARRAY: */
      "Getting feature structure from array",
      /*  160 - UIMA_MSG_ID_EXCON_SETTING_FS_IN_ARRAY: */
      "Setting feature structure in array",
      /*  161 - UIMA_MSG_ID_EXCON_CREATING_STRINGFS: */
      "Creating string feature structure",
      /*  162 - UIMA_MSG_ID_EXCON_READING_TYPESYSTEM_FROM_XML: */
      "Reading type system from xml file",
      /*  163 - UIMA_MSG_ID_EXCON_CREATING_TYPE: */
      "Trying to create type '%1' as subtype of type '%2'",
      /*  164 - UIMA_MSG_ID_EXCON_CREATING_FEATURE: */
      "Trying to create feature '%1' on type '%2'",
      /*  165 - UIMA_MSG_ID_EXCON_CREATING_ITERATOR: */
      "Creating iterator for index",
      /*  166 - UIMA_MSG_ID_EXCON_CREATING_TYPESYSTEM_FROM_CONFIG: */
      "Creating type system from configuration",
      /*  167 - UIMA_MSG_ID_EXCON_CREATING_TYPEPRIORITIES_FROM_CONFIG: */
      "Creating type priorities from configuration",
      /*  168 - UIMA_MSG_ID_EXCON_CREATING_INDEXES_FROM_CONFIG: */
      "Creating indexes from configuration",
      /*  169 - UIMA_MSG_ID_EXCON_DESERIALIZING_CAS: */
      "Deserializing CAS",
      /*  170 - UIMA_MSG_ID_ERR_PARSER_CONFIG_INVALID_RULE_CONTENT_ERROR: */
      "Invalid content found in parser config expression: %1",
      /*  171 - UIMA_MSG_ID_ERRCON_PARSER_CONFIG_FUNCTION: */
      "executing parser config function: %1",
      /*  172 - UIMA_MSG_ID_ERRCON_PARSER_CONFIG_FILE: */
      "loading parser config file: %1",
      /*  173 - UIMA_MSG_ID_ERRCON_PARSER_CONFIG_TAG: */
      "parsing parser config tag: %1",
      /*  174 - UIMA_MSG_ID_ERRCON_PARSER_CONFIG_LINE: */
      "parsing parser config line: %1",
      /*  175 - UIMA_MSG_ID_EXC_FILE_ACCESS: */
      "Error trying to access file '%1'",
      /*  176 - UIMA_MSG_ID_EXC_FILE_OPEN: */
      "Error trying to open file '%1'",
      /*  177 - UIMA_MSG_ID_EXC_FILE_WRITE: */
      "Error trying to write to file '%1'",
      /*  178 - UIMA_MSG_ID_EXC_FILE_READ: */
      "Error trying to read from file '%1'",
      /*  179 - UIMA_MSG_ID_LOG_TO_ERROR_INFO: */
      "%1: %2",
      /*  180 - UIMA_MSG_ID_LOG_MESSAGE: */
      "Message (%1)",
      /*  181 - UIMA_MSG_ID_LOG_WARNING: */
      "Warning (%1)",
      /*  182 - UIMA_MSG_ID_LOG_ERROR: */
      "Error (%1)",
      /*  183 - UIMA_MSG_ID_LOG_FATAL_ERROR: */
      "Fatal (%1)",
      /*  184 - UIMA_MSG_ID_LOG_USER_CODE: */
      "User code",
      /*  185 - UIMA_MSG_ID_EXC_CONV_BUFFER_PROBLEM: */
      "Could not allocate codepage-conversion buffer",
      /*  186 - UIMA_MSG_ID_EXC_U2CP_CONVERSION_PROBLEM: */
      "Could not convert '%1' to a codepage string",
      /*  187 - UIMA_MSG_ID_EXC_CP2U_CONVERSION_PROBLEM: */
      "Could not convert '%1' to a unicode string",
      /*  188 - UIMA_MSG_ID_EXC_POE_DICTIONARY_UNLOAD: */
      "Error unloading dictionaries",
      /*  189 - UIMA_MSG_ID_EXC_POE_DICTIONARY_NOT_FOUND: */
      "Dictionary not found",
      /*  190 - UIMA_MSG_ID_EXC_POE_DICTIONARY_NOT_LOADABLE: */
      "Dictionary not loadable (%1)",
      /*  191 - UIMA_MSG_ID_EXC_POE_LEXICAL_ANALYSIS_FAILED: */
      "Lexical Analysis failed",
      /*  192 - UIMA_MSG_ID_EXC_POE_UNEXPECTED_RC: */
      "Unexpected/Unknown POE return code %1",
      /*  193 - UIMA_MSG_ID_EXC_POE_LEXICAL_ANALYSIS_ABORTED: */
      "Lexical Analysis aborted by POE exception",
      /*  194 - UIMA_MSG_ID_EXCON_POE_ABBREV_DICT_OPEN: */
      "Trying to open abbreviation dictionary '%1'",
      /*  195 - UIMA_MSG_ID_EXCON_POE_STOPWORD_DICT_OPEN: */
      "Trying to open stopword dictionary '%1'",
      /*  196 - UIMA_MSG_ID_EXCON_POE_ADDENDA_DICT_OPEN: */
      "Trying to open addenda dictionary '%1'",
      /*  197 - UIMA_MSG_ID_EXCON_POE_MAIN_DICT_OPEN: */
      "Trying to open main dictionary '%1'",
      /*  198 - UIMA_MSG_ID_EXCON_POE_USING_DICT_PATH: */
      "Using dictionary search path '%1'",
      /*  199 - UIMA_MSG_ID_EXCON_POE_CALLING_LEXICAL_ANALYSIS: */
      "Calling POE lexical analysis function",
      /*  200 - UIMA_MSG_ID_EXCON_POE_CALLING_MORPHID: */
      "Calling POE dictionary lookup (morph-id) function",
      /*  201 - UIMA_MSG_ID_EXCON_TOK_ALLOCATING_CHARTABLE: */
      "Trying to allocate charclass table",
      /*  202 - UIMA_MSG_ID_EXC_THES_PATH_NOT_FOUND: */
      "Thesaurus path '%1' not found",
      /*  203 - UIMA_MSG_ID_EXC_THES_FILE_NOT_FOUND: */
      "Thesaurus file '%1' not found",
      /*  204 - UIMA_MSG_ID_EXC_THES_NO_PATH_CREATED: */
      "Failed creating the Path-Location ?????.",
      /*  205 - UIMA_MSG_ID_EXC_THES_NO_FILE_NAME: */
      "No filename specification for thesaurus",
      /*  206 - UIMA_MSG_ID_EXC_THES_NO_PATH_NAME: */
      "No path specification for thesaurus",
      /*  207 - UIMA_MSG_ID_EXC_THES_NO_PATH_PARAM: */
      "Thesaurus path not specified in Configuration.",
      /*  208 - UIMA_MSG_ID_EXC_THES_NO_FILE_PARAM: */
      "Thesaurus file not specified in Configuration.",
      /*  209 - UIMA_MSG_ID_EXC_THES_RELATION_NOT_FOUND: */
      "Relation '%1' not found in Thesaurus",
      /*  210 - UIMA_MSG_ID_EXC_THES_DEPTH_OVERFLOW: */
      "Specified depth greater than MAX_DEPTH.",
      /*  211 - UIMA_MSG_ID_EXC_THES_MAXCOUNT_OVERFLOW: */
      "Specified MaxCount greater than MAX_MAXCOUNT.",
      /*  212 - UIMA_MSG_ID_EXC_THES_NO_ERRCODE_MSG: */
      "No further error information available (ThGetLastError failed).",
      /*  213 - UIMA_MSG_ID_EXC_THES_FAIL_MSG: */
      "%1 failed. ErrorCode from the Thesaurus: %2",
      /*  214 - UIMA_MSG_ID_EXC_THES_INVALID_PATH: */
      "Invalid Option for Thesaurus Path.",
      /*  215 - UIMA_MSG_ID_EXC_THES_INVALID_FILE: */
      "Invalid Option for Thesaurus File.",
      /*  216 - UIMA_MSG_ID_EXC_THES_INVALID_RELATION: */
      "No valid relation name specified.",
      /*  217 - UIMA_MSG_ID_EXC_THES_INVALID_DEPTH: */
      "No valid depth specified.",
      /*  218 - UIMA_MSG_ID_EXC_THES_INVALID_MAXCOUNT: */
      "No valid MaxCount specified.",
      /*  219 - UIMA_MSG_ID_EXC_CATRULE_PARSING_RULE_FILE_FAILED: */
      "XML parsing of rule file %1 failed",
      /*  220 - UIMA_MSG_ID_EXCON_CATRULE_INITIALIZING: */
      "Initializing categorization annotator",
      /*  221 - UIMA_MSG_ID_EXCON_SUM_WFTABLE_LOAD: */
      "Trying to load word frequency table for language %1",
      /*  222 - UIMA_MSG_ID_EXCON_SUM_WFTABLE_SAVE: */
      "Trying to save word frequency table for language %1",
      /*  223 - UIMA_MSG_ID_EXCON_DOX_DOXTRACT_ERROR: */
      "doxtract processing in function %1",
      /*  224 - UIMA_MSG_ID_LOGACCESS: */
      "doxtract Log file is not accessible.",
      /*  225 - UIMA_MSG_ID_FILENLEN: */
      "Path/file name too long: %1.",
      /*  226 - UIMA_MSG_ID_AUTHFNLEN: */
      "Error in configuration file: Name of doxtract authority file is too lo"
      "ng.",
      /*  227 - UIMA_MSG_ID_STOPFNLEN: */
      "Error in configuration file: Name of doxtract stopword file is too lon"
      "g.",
      /*  228 - UIMA_MSG_ID_NO_POEDICT: */
      "Unable to open doxtract POE dictionary.",
      /*  229 - UIMA_MSG_ID_NO_TSTOP_FILE: */
      "Unable to open doxtract stop word file.",
      /*  230 - UIMA_MSG_ID_NO_AUTH_FILE: */
      "Unable to open doxtract authority file.",
      /*  231 - UIMA_MSG_ID_CREA_LOGFILE: */
      "Set LOGGING_DIRECTORY in doxtract configuration to see details.",
      /*  232 - UIMA_MSG_ID_IMZ_ERROR_1STRING: */
      "Original component error message: (%1) %2 [%3]",
      /*  233 - UIMA_MSG_ID_IMZ_ERROR_2STRING: */
      "Original component error message: (%1) %2 %3 [%4]",
      /*  234 - UIMA_MSG_ID_EXCON_LIN_RESFILE_LOAD: */
      "Trying to load language detection resource file",
      /*  235 - UIMA_MSG_ID_EXC_LIN_MAXCAT: */
      "Cannot have any more than %1 categories",
      /*  236 - UIMA_MSG_ID_EXC_LIN_WORKDICT_CR_FAILED: */
      "Failed to create working dictionary",
      /*  237 - UIMA_MSG_ID_EXC_LIN_WORKDICT_INSERT_FAILED: */
      "Insert into new working dictionary failed",
      /*  238 - UIMA_MSG_ID_EXC_LIN_MASTDICT_CR_FAILED: */
      "Failed to create master dictionary",
      /*  239 - UIMA_MSG_ID_EXC_LIN_NEWMASTDICT_CR_FAILED: */
      "Failed to create new master dictionary",
      /*  240 - UIMA_MSG_ID_EXC_LIN_MASTDICT_NR_COMPONENTS: */
      "Too many components for master dictionary",
      /*  241 - UIMA_MSG_ID_EXC_LIN_PREFIXES_NOT_FOUND: */
      "PREFIXES %1 not found",
      /*  242 - UIMA_MSG_ID_EXC_LIN_MASTDICT_STR_NOT_FOUND: */
      "String |%1| not found in master dict",
      /*  243 - UIMA_MSG_ID_EXCON_LIN_MASTDICT_SAVE: */
      "Saving master dictionary",
      /*  244 - UIMA_MSG_ID_EXCON_LIN_MASTDICT_OPEN: */
      "Opening master dictionary",
      /*  245 - UIMA_MSG_ID_EXCON_LIN_MASTDICT_TRYOPEN: */
      "Trying to open master dictionary",
      /*  246 - UIMA_MSG_ID_EXC_LIN_MASTDICT_NO_MINWLENGTH: */
      "Master min word length unspecified",
      /*  247 - UIMA_MSG_ID_EXC_LIN_MASTDICT_NO_MAXWLENGTH: */
      "Master max word length unspecified",
      /*  248 - UIMA_MSG_ID_EXC_LIN_MASTDICT_NO_GRAMLENGTH: */
      "Master n-gram length unspecified",
      /*  249 - UIMA_MSG_ID_EXC_LIN_MASTDICT_WRONG_MINWLENGTH: */
      "Wrong master min word length of %1",
      /*  250 - UIMA_MSG_ID_EXC_LIN_MASTDICT_WRONG_MAXWLENGTH: */
      "Wrong master max word length of %1",
      /*  251 - UIMA_MSG_ID_EXC_LIN_MASTDICT_WRONG_GRAMLENGTH: */
      "Wrong master n-gram length of %1",
      /*  252 - UIMA_MSG_ID_EXC_LIN_MASTDICT_PREFX_USED: */
      "Already using prefix |%1|",
      /*  253 - UIMA_MSG_ID_EXC_LIN_DICT_FEATTOTAL: */
      "FEAT_TOTAL for %1 not found",
      /*  254 - UIMA_MSG_ID_EXC_LIN_DICT_FEATSS: */
      "FEAT_SS for %1 not found",
      /*  255 - UIMA_MSG_ID_EXC_LIN_DICT_BALANCED: */
      "Dictionary already balanced",
      /*  256 - UIMA_MSG_ID_EXC_LIN_MASTDICT_MISSING_ENTRY: */
      "Missing entry |%1| in master dictionary",
      /*  257 - UIMA_MSG_ID_EXC_LIN_TEMPDICT_MISSING_ENTRY: */
      "Missing entry |%1| in temporary dictionary",
      /*  258 - UIMA_MSG_ID_EXC_LIN_PREFIX_LENGTH: */
      "Prefix too long",
      /*  259 - UIMA_MSG_ID_EXC_LIN_BALANCE_NOT_SUPPORTED: */
      "Blancing not (yet) supported",
      /*  260 - UIMA_MSG_ID_EXC_LIN_BALANCE_AND_TRAIN: */
      "Blancing and training cannot be run simultaneously",
      /*  261 - UIMA_MSG_ID_EXC_LIN_MISSING_NEWDICTNAME: */
      "Missing output dictionary name",
      /*  262 - UIMA_MSG_ID_EXC_LIN_DICTNAME_EQ_NEWDICTNAME: */
      "For balancing new dictionary name must differ from current one",
      /*  263 - UIMA_MSG_ID_ERR_LIN_PARAM_OUT_OF_RANGE: */
      "Parameter value out of range",
      /*  264 - UIMA_MSG_ID_ERRCON_LIN_SET_MAXCHARS: */
      "Setting MaxCharsToExamine",
      /*  265 - UIMA_MSG_ID_ERRCON_LIN_SET_MAXLANGS: */
      "Setting MaxTopLanguages",
      /*  266 - UIMA_MSG_ID_EXC_DOX_GENERIC_ERROR: */
      "Error occured in annotator Doxtract-JP",
      /*  267 - UIMA_MSG_ID_EXCON_DOX_FUNCTION_CONTEXT: */
      "executing Doxtract-JP function %1",
      /*  268 - UIMA_MSG_ID_EXC_REGEX_GENERIC_ERROR: */
      "Error occured in annotator regex",
      /*  269 - UIMA_MSG_ID_ERR_REGEX_PARSE_ERROR: */
      "Errror %1 in regular expression %2",
      /*  270 - UIMA_MSG_ID_ERR_REGEX_NO_RULES_ERROR: */
      "No regex rules found",
      /*  271 - UIMA_MSG_ID_ERR_REGEX_INVALID_RULE_CONTENT_ERROR: */
      "Invalid content found in regex rule: %1",
      /*  272 - UIMA_MSG_ID_ERR_REGEX_UNKNOWN_TYPE_ERROR: */
      "Unknown UIMACPP type encountered: %1",
      /*  273 - UIMA_MSG_ID_ERR_REGEX_UNKNOWN_FEAT_ERROR: */
      "Unknown UIMACPP feature encountered: %1",
      /*  274 - UIMA_MSG_ID_ERR_REGEX_INVALID_FEAT_ERROR: */
      "Invalid UIMACPP feature encountered: %1 (must be of type int, float or str"
      "ing)",
      /*  275 - UIMA_MSG_ID_EXC_REGEX_TYPESYSTEM_FILE: */
      "regex typesystem file not found: %1",
      /*  276 - UIMA_MSG_ID_ERRCON_REGEX_FUNCTION: */
      "executing regex function: %1",
      /*  277 - UIMA_MSG_ID_ERRCON_REGEX_RULE_FILE: */
      "loading regex rule file: %1",
      /*  278 - UIMA_MSG_ID_ERRCON_REGEX_RULE: */
      "parsing regex rule:\n%1",
      /*  279 - UIMA_MSG_ID_ERRCON_REGEX_RULE_LINE: */
      "parsing regex rule line: %1",
      /*  280 - UIMA_MSG_ID_EXCCON_REGEX_SET_FEATURE: */
      "setting feature to new value",
      /*  281 - UIMA_MSG_ID_EXCON_PARSING_TOTEM_XMLFILE: */
      "Parsing XML totem file at line %1 in column %2",
      /*  282 - UIMA_MSG_ID_EXC_UNKOWN_XMLTAG: */
      "Found unknown XML tag",
      /*  283 - UIMA_MSG_ID_EXC_UNKNOWN_SYMBOLTABLE: */
      "Unknown symbol table name",
      /*  284 - UIMA_MSG_ID_EXC_SYMBOLTABLE_ALREADY_FILLED: */
      "Symbol table already filled",
      /*  285 - UIMA_MSG_ID_EXC_INSTRUCTIONS_ALREADY_FILLED: */
      "Instructions already filled",
      /*  286 - UIMA_MSG_ID_EXC_WRONG_ARGUMENT_TYPE: */
      "Wrong argument type for instruction",
      /*  287 - UIMA_MSG_ID_EXC_INVALID_INPUT_ANNOTATION_TYPE: */
      "Invalid input annotation type",
      /*  288 - UIMA_MSG_ID_EXC_WRONG_ARGUMENT_NUMBER: */
      "Wrong argument number for instruction",
      /*  289 - UIMA_MSG_ID_EXC_UNKNOWN_INSTRUCTION: */
      "Unknown instruction",
      /*  290 - UIMA_MSG_ID_EXC_UNSUPPORTED_XML_ATTRIBUTE: */
      "Unsupported XML attribute in file %1. %2. ",
      /*  291 - UIMA_MSG_ID_EXCON_CREATING_POOL_FOR_CLASS: */
      "Creating APR pool for class %1",
      /*  292 - UIMA_MSG_ID_EXC_APR_ERROR: */
      "APR error: %1",
      /*  293 - UIMA_MSG_ID_EXCON_APR_FUNCTION: */
      "Calling APR function: %1",
      /*  294 - UIMA_MSG_ID_EXCON_CONSTRUCTING_CLASS: */
      "Constructing class %1",
      /*  295 - UIMA_MSG_ID_RESERVED_05: */
      "?",
      /*  296 - UIMA_MSG_ID_RESERVED_06: */
      "?",
      /*  297 - UIMA_MSG_ID_RESERVED_07: */
      "?",
      /*  298 - UIMA_MSG_ID_RESERVED_08: */
      "?",
      /*  299 - UIMA_MSG_ID_RESERVED_09: */
      "?",
      /*  300 - UIMA_MSG_ID_RESERVED_10: */
      "?",
      /*  301 - UIMA_MSG_ID_RESERVED_11: */
      "?",
      /*  302 - UIMA_MSG_ID_RESERVED_12: */
      "?",
      /*  303 - UIMA_MSG_ID_RESERVED_13: */
      "?",
      /*  304 - UIMA_MSG_ID_RESERVED_14: */
      "?",
      /*  305 - UIMA_MSG_ID_RESERVED_15: */
      "?",
      /*  306 - UIMA_MSG_ID_RESERVED_16: */
      "?",
      /*  307 - UIMA_MSG_ID_RESERVED_17: */
      "?",
      /*  308 - UIMA_MSG_ID_RESERVED_18: */
      "?",
      /*  309 - UIMA_MSG_ID_RESERVED_19: */
      "?",
      /*  310 - UIMA_MSG_ID_RESERVED_20: */
      "?",
      /*  311 - UIMA_MSG_ID_RESERVED_21: */
      "?",
      /*  312 - Incompatible UIMA Java version: */
      "Incompatible UIMA Java version '%1'. Supported versions: '%2'",
      /*  313 - CASPool not created */
      "CASPool not created.",
      /*  314 - Error accessing Sofa Data. */
      "Error accessing Sofa Data Stream.",
      /*  315 - Invalid format Sofa URI*/
      "Invalid format Sofa URI %1.",
      /*  316 - Scheme handler not registered. */
      "Scheme handler for the '%1' scheme not registered. Cannot access '%2'.",
      /*  317 - Scheme handler could not register / load. */
      "Could not register or load '%1' scheme handler for '%2'.",
      /*  318 - Could not find method. */
      "Could not find procedure '%1' in schema handler '%2' for sofa uri '%3'.",
      /*  319 - Schemehandler open failed. */
      "Open sofa data stream failed for sofa uri '%1' using schema handler '%2'.",
      /*  320 - SofaDataStream not open. */
      "%1 failed. SofaDataStream not opened for sofa uri '%2'.",
      /*  321 - Local SofaDataStream open failed. */
      "Open failed. Unsupported local sofa data type '%1'.",
      /*  322 - Local SofaDataStream open failed. */
      "Open failed. Local sofa data not set '%1'.",
      /*  323 - Local SofaDataStream read failed. Invalid element size for type. */
      "Read failed. Invalid element size '%1' for local sofa data type '%2'.",
      /*  324 - Local SofaDataStream seek failed . */
      "Seek failed. Invalid seek origin '%1'.",
      /*  325 - Duplicate registration. */
      "Duplicate registration '%1' for uri scheme '%2'.",
      /*  326 - Codepage conversion error */
      "Codepage conversion error.  ICU error code '%1' from '%2'.",
      /*  327 - No Sofa found error */
      "No Sofa named '%1' found in the CAS.",
      /*  328 - UIMA_MSG_ID_EXC_INVALID_BASE_CAS_METHOD: */
      "Invalid method: Base CAS has no Sofa.",
      /*  329 - UIMA_MSG_ID_EXC_NO_FREE_CAS */
      "There are no free CASs in the CASPool. ",
      /*  330 - UIMA_MSG_ID_EXC_INVALID_CALL_TO_NEXT */
      "Invalid call to next(). ",
      /*  331 - UIMA_MSG_ID_SIGNATURE_END: */
      "[UIMA-LIBRARY]",
    } ;

#endif /* UIMA_MSGSTRTAB_H */

