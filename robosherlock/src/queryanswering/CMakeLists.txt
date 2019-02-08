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

add_executable(swi_test src/swiprolog_test.cpp)
target_link_libraries(swi_test
        ${SWIPL_LIBRARIES}
        ${catkin_LIBRARIES})
