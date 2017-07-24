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

#include <rs/utils/RSPipelineManager.h>


std::vector<icu::UnicodeString> &RSPipelineManager::getFlowConstraintNodes()
{
  std::vector<icu::UnicodeString> const &nodes = flow->getNodes();
  std::vector<icu::UnicodeString> &flow_constraint_nodes = const_cast<std::vector<icu::UnicodeString> &>(nodes);
  return flow_constraint_nodes;
}

void RSPipelineManager::resetPipelineOrdering()
{
  aengine->iv_annotatorMgr.iv_vecEntries = original_annotators; // Reset to the original pipeline

  // Set default pipeline annotators, if set
  if(use_default_pipeline)
  {
    setPipelineOrdering(default_pipeline_annotators);
  }
}
void RSPipelineManager::setDefaultPipelineOrdering(std::vector<std::string> annotators)
{
  use_default_pipeline = true;
  default_pipeline_annotators = annotators;
}

int RSPipelineManager::getIndexOfAnnotator(std::string annotator_name)
{
  icu::UnicodeString icu_annotator_name = icu::UnicodeString::fromUTF8(StringPiece(annotator_name.c_str()));

  std::vector<icu::UnicodeString> &nodes = getFlowConstraintNodes();
  auto it = std::find(nodes.begin(), nodes.end(), icu_annotator_name);
  if(it == nodes.end())
  {
    return -1;
  }

  return std::distance(nodes.begin(), it);
}

void RSPipelineManager::setPipelineOrdering(std::vector<std::string> annotators)
{
  // Create empty list of
  //  typedef std::vector < EngineEntry > TyAnnotatorEntries;
  //  called 'new_annotators'
  //
  //
  // For each given annotator:
  //  1) Look up the index of the desired annotator
  //  2) Add a copy of the respectie EngineEntry from the original_annotators to the new list
  //
  uima::internal::AnnotatorManager::TyAnnotatorEntries new_annotators;
  for(int i = 0; i < annotators.size(); i++)
  {
    //  1) Look up the index of the desired annotator
    int idx = getIndexOfAnnotator(annotators.at(i));
    if(idx >= 0)
    {
      //  2) Add a copy of the respectie EngineEntry from the original_annotators to the new list
      uima::internal::AnnotatorManager::EngineEntry ee = original_annotators.at(idx);
      new_annotators.push_back(ee);
      continue;
    }

    // Right now, we just skip this annotator if it can't be found.
    outWarn(annotators.at(i) <<
            " can't be found in your loaded AnalysisEngine. Maybe it has not been "
            "defined in your given AnalysisEngine XML file? - Skipping it....");
    // return;
  }
  // Pass the new pipeline to uima's annotator manager
  aengine->iv_annotatorMgr.iv_vecEntries = new_annotators;
}
