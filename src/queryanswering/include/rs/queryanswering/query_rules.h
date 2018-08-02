/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Patrick Mania <pmania@cs.uni-bremen.de>
 *         Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __QUERY_RULES_H__
#define __QUERY_RULES_H__

#define QUERY_ANNOTATOR_INPUTS_VAR           "Input"
#define QUERY_ANNOTATOR_INPUTS(annotator)    "owl_class_properties(rs_components:\'" + annotator + "\',rs_components:\'perceptualInputRequired\'," + QUERY_ANNOTATOR_INPUTS_VAR + ")."

#define QUERY_ANNOTATOR_OUTPUTS_VAR          "Output"
#define QUERY_ANNOTATOR_OUTPUTS(annotator)   "owl_class_properties(rs_components:\'" + annotator + "\',rs_components:\'perceptualOutput\'," + QUERY_ANNOTATOR_OUTPUTS_VAR + ")."

#define ROBOSHERLOCK_QUERY_PREFIX            "http://knowrob.org/kb/rs_components.owl#"

#endif // __QUERY_RULES_H__
