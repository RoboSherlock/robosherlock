project(rs_queryanswering)

add_library(rs_queryanswering SHARED
        src/ObjectDesignatorFactory.cpp
        src/JsonPrologInterface.cpp
	src/SWIPLInterface.cpp 
        src/QueryInterface.cpp)
target_link_libraries(rs_queryanswering
        ${SWIPL_LIBRARIES}
        ${catkin_LIBRARIES}
        ${json_prolog_LIBRARIES})

