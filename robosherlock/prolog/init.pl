%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies
:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_vis).
:- register_ros_package(knowrob_srdl).
:- register_ros_package(robosherlock).

:- owl_parser:owl_parse('package://iai_kitchen/owl/iai-kitchen-objects.owl').
:- rdf_db:rdf_register_prefix(kitchen, 'http://knowrob.org/kb/iai-kitchen.owl#', [keep(true)]).

:- owl_parser:owl_parse('package://robosherlock/owl/rs_components.owl').
:- rdf_db:rdf_register_prefix(rs_components, 'http://knowrob.org/kb/rs_components.owl#', [keep(true)]).


:- use_module(library(rs_interface)). % a ROS-prolog interface for interacting with RobOSherlock
:- use_module(library(rs_query_reasoning)). % reasoning about queries
:- use_module(library(rs_similar_objects)).
