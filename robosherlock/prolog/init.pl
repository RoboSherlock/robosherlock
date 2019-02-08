%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies


:- use_module(library(semweb/rdfs)). % reasoning about queries
:- use_module(library(semweb/rdf_db)). % reasoning about queries
:- use_module(owl).
:- use_module(rdfs).
:- use_module(owl_2).
:- rdf_db:rdf_register_prefix(kitchen, 'http://knowrob.org/kb/iai-kitchen.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rs_components, 'http://knowrob.org/kb/rs_components.owl#', [keep(true)]).

:- rdf_load('../owl/rs_components.owl').
:- rdf_load('../owl/iai-kitchen-objects.owl').

:- use_module(rs_query_reasoning). % reasoning about queries

%:- owl_parser:owl_parse('../owl/rs_components.owl').

