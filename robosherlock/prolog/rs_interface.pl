:- module(rs_query_interface,
  [
  rs_pause/1,
  rs_stop/0,
  execute_pipeline/1,  
  execute_annotator/1,
  run_annotator/1,
  execute_individual_of_annotator/1,
  detect/1,
  get_list_of_predicates/2, 
  parse_description/2
]).

:- rdf_meta
  execute_annotator(t),
  run_annotator(t).

:- use_foreign_library('librs_prologQueries.so').
:- use_module(library(rs_query_reasoning)).

%%%%%%%%%%%%%%%% BEGIN: C++ Interface %%%%%%%%%%%%%%%%%%%%
%%Queries written using this interface need a sanity check
%%e,g,. spatial relations do not make sense inside a color determiner 

%% OLD QUERIES starting RS process from within Prolog
%rs_interface :-
   %rs_interface(_).

%:- assert(rs_interf(fail)).

%rs_interface(Client,Ae) :-
   %rs_interf(fail),
   %cpp_init_ae(Client,Ae),
   %retract(rs_interf(fail)),
   %assert(rs_interf(Client)),!.
    
%rs_interface(Cl):-
   %rs_interf(Cl).
   
execute_pipeline(A):-
  cpp_execute_pipeline(A).
  
run_annotator(A):-
  execute_annotator(A);
  execute_individual_of_annotator(A).
  
execute_annotator(A):-
  \+owl_individual_of(A,rs_components:'RoboSherlockComponent'),
  build_pipeline([A],P),
  execute_pipeline(P).
  
execute_individual_of_annotator(A):-
  owl_individual_of(A,C),compute_annotators(C),
  build_pipeline([C],P),
  execute_pipeline(P).

rs_pause(A):-
   cpp_rs_pause(A).

rs_stop:-
   rs_interf(Cl),
   cpp_stop_rs(Cl),
   assert(rs_interf(fail)).

rs_clear_ae:-
   rs_interf(Ae),
   cpp_remove_ae(Ae),
   assert(rs_interf(fail)).

   
%%START PARSING the Query   
%defs for syntax checks
designator_type([an,object],'object').
designator_type([an,obj],'object').
designator_type([the,object],'object').
designator_type([a,location],'location').

%defs for designator types
designator(location).
designator(object).

% for simplifying query writing spatial relation can also be keyword
spatial_relation(on).
spatial_relation(in).
spatial_relation(next-to).
spatial_relation(left-of).
spatial_relation(right-of).
spatial_relation(behind).
spatial_relation(in-front-of).

%keyword(A):-
%  spatial_relation(A).

% check if key can exist and add it to designator
add_kvp(Key,Value,D):-
    %rs_query_predicate(Key),
    cpp_add_kvp(Key,Value,D).

% handle case when key hints at a nested designator
add_kvp(Key,Value,D):-
    designator(Key),
    parse_nested_description(Value,D).

% needed to manage 'on' as a spatial relation
add_kvp(Key,Value,D):-
    spatial_relation(Key),
    is_list(Value),
    cpp_init_kvp(D,Key,Kvp),
    parse_nested_description(Value,Kvp).

% if query term following spatial relation is not a description (list in our case)
% add the key value pair
add_kvp(Key,Value,D):-
    spatial_relation(Key),
    \+is_list(Value),
    cpp_add_kvp(Key,Value,D).
%return true once List is empty

add_kvp([],_).

%main rule for adding kvps
add_kvp([Head|Tail],D):-
    length(Head, Hl),
    (Hl=2->nth1(1,Head,Key),
	   nth1(2,Head,Value),
	   add_kvp(Key,Value,D);
    	   designator_type(Head),
	   parse_nested_description(Head,D)
    ),
    add_kvp(Tail,D).

% add a nested desig to the main obj-designator
parse_nested_description([A,B|Tail],D):-
    designator_type([A,B],T),
    cpp_init_kvp(D,T,KVP),
    add_kvp(Tail,KVP).

% pars the designator given by detect
parse_description([ A,B | Tail],D):-
    designator_type([A,B],T),
    cpp_add_designator(T,D), 
    add_kvp(Tail,D).

designator_type([ A,B | _ ] ):-
    designator_type([A,B],_).

    
detect(List):-
    %rs_interface(A),
    parse_description(List,D),
    thread_create((cpp_print_desig(D),
    cpp_query_rs(D)),_,[]).
     %thread_join(Th,Status).

%%%%%%%%%%%%%%%%%%%%% NEW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

get_keys([],[]).
get_keys([H|T],L1):-
        %rs_query_predicate(H),
        L1=[H|T1],get_keys(T,T1);
        get_keys(T,L1).


rs_pipeline_from_query(Q,P):-
    get_keys(Q,Keys),
    build_pipeline_from_predicates(Keys,P).

%%%%%%%%%%%%%%%%%%%%% %%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%% END: C++ Interface %%%%%%%%%%%%%%%%%%%%    

get_list_of_predicates([],[]).
get_list_of_predicates([Pred|T],[Pred|Result]):-
	is_predicate(Pred),
	get_list_of_predicates(T,Result).
get_list_of_predicates([_|Tail],Result):-
get_list_of_predicates(Tail,Result).  
