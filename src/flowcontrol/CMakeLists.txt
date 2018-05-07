project(rs_flowcontrol)

add_library(rs_flowcontrol
    src/RSAnalysisEngine.cpp
    src/RSPipelineManager.cpp
    src/RSControledAnalysisEngine.cpp
    src/RSProcessManager.cpp)
target_link_libraries(rs_flowcontrol rs_core rs_io rs_queryanswering
    ${LIBAPR_LIBRARY} ${SWIPL_LIBRARIES} ${UIMA_LIBRARY} ${ICUUC_LIBRARY} ${catkin_LIBRARIES})



rs_add_executable(run
        src/run.cpp)
target_link_libraries(run
        rs_flowcontrol ${catkin_LIBRARIES})
