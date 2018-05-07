project(rs_queryanswering)


add_library(rs_queryanswering SHARED
        src/DesignatorWrapper.cpp
        src/PrologInterface.cpp
        src/QueryInterface.cpp)
add_dependencies(rs_queryanswering
        designator_integration_msgs_gencpp json_prolog)
target_link_libraries(rs_queryanswering
        ${catkin_LIBRARIES}
        ${SWIPL_LIBRARIES}
        ${json_prolog_LIBRARIES})

