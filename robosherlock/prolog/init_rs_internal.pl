%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies


:- use_module(library(semweb/rdfs)). % reasoning about queries
:- use_module(library(semweb/rdf_db)). % reasoning about queries
:- use_module(external/owl).
:- use_module(external/rdfs).
:- use_module(external/owl_2).

:- rdf_db:rdf_register_ns(rdfs,    'http://www.w3.org/2000/01/rdf-schema#',     [keep(true)]).
:- rdf_db:rdf_register_ns(owl,     'http://www.w3.org/2002/07/owl#',            [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).

:- rdf_db:rdf_register_prefix(kitchen, 'http://knowrob.org/kb/iai-kitchen.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rs_components, 'http://knowrob.org/kb/rs_components.owl#', [keep(true)]).

:- rdf_load('../owl/rs_components.owl').
:- rdf_load('../owl/iai-kitchen-objects.owl').

:- use_module(rs_query_reasoning). % reasoning about queries

