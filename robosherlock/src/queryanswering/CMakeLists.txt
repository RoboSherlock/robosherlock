project(rs_queryanswering)

add_library(rs_queryanswering SHARED
        src/ObjectDesignatorFactory.cpp
        src/JsonPrologInterface.cpp
        src/SWIPLInterface.cpp
        src/QueryInterface.cpp)
target_link_libraries(rs_queryanswering
        ${json_prolog_LIBRARIES}
        ${SWIPL_LIBRARIES}
        ${catkin_LIBRARIES})

add_library(rs_prologQueries SHARED src/PrologCPPQueries.cpp)
target_link_libraries(rs_prologQueries ${robosherlock_LIBRARIES} 
    ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(rs_prologQueries ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
