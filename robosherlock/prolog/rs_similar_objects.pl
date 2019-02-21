:- module(rs_similar_objects,
  [
  rs_object_candidates/1
]).

rs_object_candidates([]).

rs_object_candidates([Head|Tail]):-
  is_valid_candidate(Head,_),
  rs_object_candidates(Tail).

is_valid_candidate(List,Obj):-
  length(List,Ll),
  (Ll=2 -> nth1(1,List,Obj),
           owl_class_properties(Obj,knowrob:'pathToCadModel',_)
  ).
