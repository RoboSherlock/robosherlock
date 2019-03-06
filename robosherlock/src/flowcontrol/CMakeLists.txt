project(rs_flowcontrol)

add_library(rs_flowcontrol
    src/RSProcessManager.cpp
    src/RSParallelPipelinePlanner.cpp
    src/RSAggregateAnalysisEngine.cpp
    src/RSXMLParser.cpp
    src/YamlToXMLConverter.cpp)
add_dependencies(rs_flowcontrol robosherlock_msgs_generate_messages_cpp)
target_link_libraries(rs_flowcontrol rs_core rs_io rs_queryanswering
  ${LIBAPR_LIBRARY} ${SWIPL_LIBRARIES} ${UIMA_LIBRARY} ${ICUUC_LIBRARY} ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

rs_add_executable(run src/run.cpp)
target_link_libraries(run rs_flowcontrol ${catkin_LIBRARIES})

