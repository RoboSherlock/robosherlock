%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Rules for planing a new pipeline based on keys extracted from a query
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


:- module(rs_query_reasoning,
    [
  compute_annotators/1,
  annotators/1,
  compute_annotator_outputs/2,
  compute_annotator_inputs/2,
  reset_planning/0,
  annotator_outputs/2,
  annotator_requires_input_type/2,
  type_available/1,
  annotator_in_dependency_chain_of/2,
  dependency_chain_as_set_for_annotator/2,
  dependency_chain_ordering/3,
  ordered_dependency_chain_for_annotator/2,
  annotator_missing_inputs/2,
  annotators_satisfying_atleast_one_input/2,
  annotators_satisfying_direct_inputs_of/2,
  get_required_annotators_for_annotator_list/2,
  annotatorlist_requirements_fulfilled/1,
  get_missing_annotators/2,
  can_inputs_be_provided_for_annotator_list/1,
  build_pipeline/2,
  annotators_for_predicate/2,
  annotators_for_predicates_no_constraint/2,
  annotators_satisfying_domain_constraints/2,
  pipeline_from_predicates_with_domain_constraint/2,
  pipeline_for_key_value_pair/2,
  build_pipeline_from_predicates_no_constraints/2,
  set_annotator_domain/2,
  annotator_satisfies_domain_constraints/2,
  set_annotator_output_type_domain/3,
  set_annotator_input_type_constraint/3,
  compute_annotator_output_type_domain/3,
  compute_annotator_input_type_restriction/3,
  assert_test_pipeline/0,
  assert_query_lang/0,
  retract_query_lang/0,
  retract_all_annotators/0,
  retract_query_assertions/0
]).

:- rdf_meta
   annotator_outputs(r,r),
   compute_annotator_inputs(r,r),
   build_pipeline(t,t),
   set_annotator_domain(r,t),
   annotator_satisfies_domain_constraints(r,t),
   annotator_in_dependency_chain_of(t,t),
   annotator_requires_input_type(t,t),
   set_annotator_output_type_domain(r,t,r),
   set_annotator_input_type_constraint(r,t,r),
   compute_annotator_output_type_domain(r,r,t),
   compute_annotator_input_type_restriction(r,r,t).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pipeline Planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



compute_annotators(A) :- 
	owl_subclass_of(A,rs_components:'RoboSherlockComponent'),
        not(A = 'http://knowrob.org/kb/rs_components.owl#RoboSherlockComponent'), 
        not(A = 'http://knowrob.org/kb/rs_components.owl#AnnotationComponent'), 
        not(A = 'http://knowrob.org/kb/rs_components.owl#DetectionComponent'), 
        not(A = 'http://knowrob.org/kb/rs_components.owl#IoComponent'), 
        not(A = 'http://knowrob.org/kb/rs_components.owl#PeopleComponent'), 
        not(A = 'http://knowrob.org/kb/rs_components.owl#SegmentationComponent').
                
        
% cache the annotators
:- forall(compute_annotators(A), assert(annotators(A)) ).


% assert domain restriction for an individual generated from a RoboSherlockComponents
set_annotator_domain(AnnotatorI, Domain):-
    owl_individual_of(AnnotatorI,rs_components:'RoboSherlockComponent'),
    owl_restriction_assert(restriction(rs_components:'outputDomain',all_values_from(union_of(Domain))),R),
    rdf_assert(AnnotatorI,rdf:type,R).

%%%% set a domain constraint on the type of the annotator e.g. Primitive Shape annotator returns Shape annotations with value one of [a,b,c]
%%%% 
set_annotator_output_type_domain(AnnotatorI, Domain, Type):-
    owl_individual_of(AnnotatorI,rs_components:'RoboSherlockComponent'),
    owl_individual_of(AnnotatorI,Annotator),!,
    compute_annotator_outputs(Annotator, Type),%% you can only set this restriction if the Type is defined as an output type 
    owl_restriction_assert(restriction(Type,all_values_from(union_of(Domain))),R),
    rdf_assert(AnnotatorI,rdf:type,R).
    
set_annotator_input_type_constraint(AnnotatorI, Constraint, Type):-
    owl_individual_of(AnnotatorI,rs_components:'RoboSherlockComponent'),
    owl_individual_of(AnnotatorI,Annotator),!,
    compute_annotator_inputs(Annotator, Type),%% you can only set this restriction if the Type is defined as an output type 
    owl_restriction_assert(restriction(Type,all_values_from(union_of(Constraint))),R),
    rdf_assert(AnnotatorI,rdf:type,R).

compute_annotator_output_type_domain(AnnotatorI, Type, Domain):-
    owl_individual_of(AnnotatorI,rs_components:'RoboSherlockComponent'),
    owl_individual_of(AnnotatorI,Annotator),!,
    compute_annotator_outputs(Annotator, Type),
    owl_has(AnnotatorI,rdf:type,R),   
    owl_has(R,owl:onProperty,Type), 
    rdf_has(R,owl:allValuesFrom,V),owl_description(V,union_of(Domain)).
    
compute_annotator_input_type_restriction(AnnotatorI, Type, Domain):-
    owl_individual_of(AnnotatorI,rs_components:'RoboSherlockComponent'),
    owl_individual_of(AnnotatorI,Annotator),!,
    compute_annotator_inputs(Annotator, Type),
    owl_has(AnnotatorI,rdf:type,R),   
    owl_has(R,owl:onProperty,Type), 
    rdf_has(R,owl:allValuesFrom,V),owl_description(V,union_of(Domain)).


% Get outputs of Annotator
compute_annotator_outputs(Annotator,Output) :- 
	annotators(Annotator), 
	owl_class_properties(Annotator,rs_components:'perceptualOutput',Output).

% Get inputs of Annotator
compute_annotator_inputs(Annotator,Input) :- 
	annotators(Annotator), 
	owl_class_properties(Annotator,rs_components:'perceptualInputRequired',Input).

% cache outputs/inputs
:- forall(compute_annotator_outputs(A,O), assert(annotator_outputs(A,O)) ).
:- forall(compute_annotator_inputs(A,I), assert(annotator_requires_input_type(A,I)) ).

% If you changed the robot model or something else in this code,
% call this rule to cache the annotators and their I/Os again.
reset_planning:- retractall(annotator_outputs(_,_)),
	retractall(annotator_requires_input_type(_,_)),
	retractall(annotators(_)),
	forall(compute_annotators(A), assert(annotators(A)) ),
	forall(compute_annotator_outputs(A,O), assert(annotator_outputs(A,O)) ),
	forall(compute_annotator_inputs(A,I), assert(annotator_requires_input_type(A,I)) ).

% Get every type that can be put out by any annotator
type_available(Output) :- 
	annotator_outputs(_,Output).


% Di - annotator requiring input
% Type of Inpute
% Ai - annotator providing output
% this rule gets called only for pairs of Di, Ai where Ai produces a type that Di needs; 
input_constraints_satisfied(Di, InputType, Ai):-
    compute_annotator_input_type_restriction(Di,InputType,Restriction) ->
      (
      compute_annotator_output_type_domain(Ai,InputType,Domain),
      %write('outputDomain: '),writeln(Domain),
      %write('inputRestriction:'),writeln(Restriction),
      member(R, Restriction),member(R,Domain)%,writeln('Yay')
     );
     true.

% Check if Annotator A is somewhere in the Depedency chain of D.
% This means for example in RoboSherlock, where the CollectionReader should be at
% the first place in every pipeline:
% annotator_in_dependency_chain_of(CollectionReader,SomeShapeAnnotator) should be true.
% annotator_in_dependency_chain_of(SomeShapeAnnotator, CollectionReader) should be false.
% annotator_in_dependency_chain_of(SomeShapeAnnotator, imagePreprocessor) should be false.
% annotator_in_dependency_chain_of(X, Collectionreader) should be false.
%
% Trivial case: A is in the dependency chain of D, if A provides a type that D needs.
annotator_in_dependency_chain_of(A, D) :- 
	owl_individual_of(Di, D),
	annotator_requires_input_type(D,InputType),
	annotator_outputs(A,InputType),
	owl_individual_of(Ai,A), %and we have an individual of A
	input_constraints_satisfied(Di,InputType, Ai).

% Recursive case: A is in the Dependency chain of D, if A provides a Type
% that X needs, and X provides a type that D needs.
annotator_in_dependency_chain_of(A, D) :-
	annotator_outputs(A,InputType),
	owl_individual_of(Ai,A),    
	annotator_requires_input_type(X, InputType),
	owl_individual_of(Xi,X),
	input_constraints_satisfied(Xi,InputType,Ai),
	annotator_in_dependency_chain_of(X, D).
	

% calculate the full dependency chain for a given
% Annotator, include the annotator itself. The chain 
% MUST NOT be in the correct order
dependency_chain_as_set_for_annotator(Annotator,S) :-
	L=[Annotator],
	setof(X,annotator_in_dependency_chain_of(X,Annotator),D),
	append(L,D,S); % Either a set of dependencies can be calculated
	S=[]. % or we return an empty list, when no dependencies are present

% AnnotatorA < AnnotatorB when A is somewhere in the beginning
% of the dependency chain of B
dependency_chain_ordering(R, AnnotatorA, AnnotatorB) :-
	annotator_in_dependency_chain_of(AnnotatorA, AnnotatorB) -> R = '<' ; R = '>'.

% Order the output of dependency_chain_as_set_for_annotator in a manner
% that the evaluation order in L is correct.
ordered_dependency_chain_for_annotator(Annotator,L) :-
	dependency_chain_as_set_for_annotator(Annotator,AnnotatorChainSet),
	predsort(dependency_chain_ordering, AnnotatorChainSet, L).


% Can an input never be satisified?
annotator_missing_inputs(Annotator,Missing) :- 
	findall(Input, (annotator_requires_input_type(Annotator, Input),
	not(type_available(Input)) ), Missing).

% Get a list caled AnnotatorSatisfyingInput, that
% includes all annotators that provide _one_ input of Annotator A.
annotators_satisfying_atleast_one_input(Annotator, AnnotatorSatisfyingInput):-
	annotator_requires_input_type(Annotator, Input),
	setof(X, annotator_outputs(X,Input), AnnotatorSatisfyingInput).

% Get a List of Annotators, that provide the required inputs of
% _ALL_ inputs of A
annotators_satisfying_direct_inputs_of(Annotator, AnnotatorSet):-
	setof(X, annotators_satisfying_atleast_one_input(Annotator, X), L),
	flatten(L, AnnotatorSet);
	AnnotatorSet = []. % Return empty set, when a annotator doesn't need any inputs

get_required_annotators_for_annotator_list(AnnotatorList,RequiredAnnotators):-
	maplist(annotators_satisfying_direct_inputs_of, AnnotatorList, List),
	flatten(List, FlattenedList),
	list_to_set(FlattenedList, RequiredAnnotators).

% Check, if all the required annotators of the annotators are in the given list.
% WARNING: This does NOT work if you pass a list, that has unsatisfiable
% input requirements. This means, that the input of an Annotator
% is not the Result of ANY Annotator in the KnowledgeBase.
annotatorlist_requirements_fulfilled(AnnotatorList):-
	get_required_annotators_for_annotator_list(AnnotatorList, ReqA),!,
	% is ReqA a Subset of AnnotatorList? 
	subset(ReqA, AnnotatorList).

% Take a List of Annotators called L, calculate all the required inputs
% and the Annotators that do provide them.
% Add the Annotators to L.
% Repeat, until the size of L doesn't change.
% add_required_annotators_until_inputs_satisfied(AnnotatorList, ResultList).

% Take a List of Annotators, calculate it's dependencies on other
% Annotators and add them to the ResultList.
get_missing_annotators(AnnotatorList, ResultList):-
	maplist(dependency_chain_as_set_for_annotator,AnnotatorList, L),
	flatten(L, FlattenedList),
	list_to_set(FlattenedList, ResultList).

% Check, if the required inputs of the Anntators in AnnotatorList
% can be provided by any of the Annotators in the System.
% If the Annotator doesn't require any inputs, the method will be true.
can_inputs_be_provided_for_annotator_list(AnnotatorList):-
	% check for all members of AnnotatorList
	forall(member(R,AnnotatorList),can_inputs_be_provided_for_annotator(R)
	).
	
% Check, if the required inputs of an Annotator
% can be provided by any of the Annotators in the System.
% If the Annotator doesn't require any inputs, the method will be true.
can_inputs_be_provided_for_annotator(Annotator):-
    % The Annotator doesn't need any inputs
    \+ annotator_requires_input_type(Annotator,_) ;
	% or: EVERY input will be provided by some annotator.
    forall(
            annotator_requires_input_type(Annotator,T), 
            (annotator_outputs(D,T),
            owl_individual_of(Ri,Annotator),owl_individual_of(Di,D),
            input_constraints_satisfied(Ri, T, Di))
        ).

% TODO: Consistency Checks! Check the dependency graph for the absence of cycles.
% TODO: Test with multiple inputs 

% ListOfAnnotators: A List of Annotators that should be run. The list does not have to include the necessary dependencies for the Annotators nor must be in the correct order.
% EvaluationList: A List of Annotators that form a complete Pipeline. The Annotators should be in the correct evaluation order

build_pipeline(ListOfAnnotators,EvaluationList):-
	% Are there any requested types that can't be calculated by the system?
	can_inputs_be_provided_for_annotator_list(ListOfAnnotators) ->
	  (annotatorlist_requirements_fulfilled(ListOfAnnotators) ->
	    % If requirements are already fulfilled, bring everything in the correct order and return
	    predsort(dependency_chain_ordering, ListOfAnnotators, EvaluationList);
	    % else: get the missing annotators to complete the list and sort it.
	    get_missing_annotators(ListOfAnnotators, FullListOfAnnotators),!,
	    %for now just call this twice...hacky but seemed to work;
	    predsort(dependency_chain_ordering, FullListOfAnnotators, EvalList),
	    predsort(dependency_chain_ordering, EvalList, EvaluationList)
	  )	
	; write('** WARNING: One or more inputs of the given List of Annotators can not be computed by an Algorithm in the KnowledgeBase **'),
	fail.

% Map a predefined set of predicates to Annotator Outputs
annotators_for_predicate(P,A) :-
        rs_type_for_predicate(P,T),
        annotator_outputs(A, T).


annotator_satisfies_domain_constraints(Key,A):-
        annotators_for_predicate(Key, A),
        rs_type_for_predicate(Key, Type),
        owl_individual_of(I,A),
        ((compute_annotator_output_type_domain(I,Type,DList),requestedValueForKey(Key,Val)) -> % these relations get asserted when RoboSherlock starts; TODO: requested value for type; 
          ( Val\='' ->
	  member(class(D),DList),
	  rdf_global_id(Val,ValURI),
	  owl_subclass_of(D,ValURI);true);false)	.
  

% Predicates : list of predicates
% Annotators that satisfy the value constraint set ona  key;
annotators_satisfying_domain_constraints(Predicates, A):-
	member(P,Predicates), 
	annotator_satisfies_domain_constraints(P, A). 

% given a list of predicates get a list of pipelines
pipeline_from_predicates_with_domain_constraint(ListOfPredicates,Pipeline):-
	setof(X,annotators_satisfying_domain_constraints(ListOfPredicates, X), Annotators), % Only build one list of annotators for the given Predicates
	build_pipeline(Annotators, Pipeline),
	forall(member(P,Pipeline), can_inputs_be_provided_for_annotator(P)).
	

pipeline_for_key_value_pair((K,V),P):-
    retract_query_assertions,
    assert(requestedValueForKey(K,V)),
    pipeline_from_predicates_with_domain_constraint([K],P).
    

% OLD implementation without domain constraings(keeping as a reference for now)
annotators_for_predicates_no_constraint(Predicates, A):-
	member(P,Predicates), 
	annotators_for_predicate(P, A).

build_pipeline_from_predicates_no_constraints(ListOfPredicates,Pipeline):-
	setof(X,annotators_for_predicates_no_constraint(ListOfPredicates, X), Annotators), % Only build one list of annotators for the given Predicates
	build_pipeline(Annotators, Pipeline).
	
assert_test_pipeline:-
    owl_instance_from_class(rs_components:'CollectionReader',_),owl_instance_from_class(rs_components:'ImagePreprocessor',_),
    owl_instance_from_class(rs_components:'RegionFilter',_),
    owl_instance_from_class(rs_components:'NormalEstimator',_),
    owl_instance_from_class(rs_components:'PlaneAnnotator',_),
    owl_instance_from_class(rs_components:'ImageSegmentationAnnotator',_),
    owl_instance_from_class(rs_components:'PointCloudClusterExtractor',_),
    owl_instance_from_class(rs_components:'ClusterMerger',_),
    owl_instance_from_class(rs_components:'Cluster3DGeometryAnnotator',GI),set_annotator_output_type_domain(GI,[rs_components:'Small',rs_components:'Big',rs_components:'Medium'],rs_components:'RsAnnotationGeometry'),
    owl_instance_from_class(rs_components:'PrimitiveShapeAnnotator',PI),set_annotator_output_type_domain(PI,[rs_components:'Box',rs_components:'Round'],rs_components:'RsAnnotationShape'),
    owl_instance_from_class(rs_components:'ClusterColorHistogramCalculator',CI),set_annotator_output_type_domain(CI,[rs_components:'Yellow',rs_components:'Blue'],rs_components:'RsAnnotationSemanticcolor'),
    owl_instance_from_class(rs_components:'SacModelAnnotator',SI),set_annotator_output_type_domain(SI,[rs_components:'Cylinder'],rs_components:'RsAnnotationShape'),
    owl_instance_from_class(rs_components:'PCLDescriptorExtractor',_),	
    owl_instance_from_class(rs_components:'CaffeAnnotator',_),
    owl_instance_from_class(rs_components:'KnnAnnotator',KNNI),set_annotator_output_type_domain(KNNI,[kitchen:'WhiteCeramicIkeaBowl'], rs_components:'RsAnnotationClassification'),
    owl_instance_from_class(rs_components:'HandleAnnotator',HI),set_annotator_output_type_domain(HI,[rs_components:'Handle'], rs_components:'RsAnnotationDetection'),
    assert(requestedValueForKey(shape,rs_components:'Box')).
    
assert_query_lang:-
	assert(rs_query_predicate(shape)),
	assert(rs_query_predicate(color)),
	assert(rs_query_predicate(size)),
	assert(rs_query_predicate(detection)),
	assert(rs_query_predicate(class)),
	assert(rs_query_predicate(type)),
	assert(rs_query_predicate(pose)),
	assert(rs_query_predicate(obj-part)),
	assert(rs_query_predicate(cad-model)),
	assert(rs_query_predicate(volume)),
	assert(rs_query_predicate(contains)),
	assert(rs_query_predicate(timestamp)),
	assert(rs_query_predicate(location)),
	rdf_global_id(rs_components:'RsAnnotationShape',A),assert(rs_type_for_predicate(shape, A)),
	rdf_global_id(rs_components:'RsAnnotationSemanticcolor',B),assert(rs_type_for_predicate(color, B)),
	rdf_global_id(rs_components:'RsAnnotationGeometry',C),
	rdf_global_id(rs_components:'RsAnnotationDetection',D),
 	rdf_global_id(rs_components:'RsAnnotationClassification',E),
	rdf_global_id(rs_components:'RsAnnotationPose',F),
        assert(rs_type_for_predicate(size, C)),
	assert(rs_type_for_predicate(detection, D)),
	assert(rs_type_for_predicate(class, E)),
	assert(rs_type_for_predicate(class, D)),
	assert(rs_type_for_predicate(type, E)),
	assert(rs_type_for_predicate(type, D)),
	assert(rs_type_for_predicate(location, C)),
	assert(rs_type_for_predicate(pose, F)).

retract_query_lang:-
	retractall(rs_query_predicate(_)),
	retractall(rs_type_for_predicate(_,_)).
   	
retract_all_annotators:-
    forall(owl_subclass_of(S,rs_components:'RoboSherlockComponent'),
    rdf_retractall(_,rdf:type,S)).   
    
retract_query_assertions:-
    retract(requestedValueForKey(_,_)).
    
