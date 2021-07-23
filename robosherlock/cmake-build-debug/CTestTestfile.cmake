# CMake generated Testfile for 
# Source directory: /home/hassouna/perception/rs_ws/src/robosherlock/robosherlock
# Build directory: /home/hassouna/perception/rs_ws/src/robosherlock/robosherlock/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_robosherlock_gtest_robosherlock-test "/home/hassouna/perception/rs_ws/src/robosherlock/robosherlock/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/hassouna/perception/rs_ws/src/robosherlock/robosherlock/cmake-build-debug/test_results/robosherlock/gtest-robosherlock-test.xml" "--return-code" "/home/hassouna/perception/rs_ws/src/robosherlock/robosherlock/cmake-build-debug/devel/lib/robosherlock/robosherlock-test --gtest_output=xml:/home/hassouna/perception/rs_ws/src/robosherlock/robosherlock/cmake-build-debug/test_results/robosherlock/gtest-robosherlock-test.xml")
set_tests_properties(_ctest_robosherlock_gtest_robosherlock-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/hassouna/perception/rs_ws/src/robosherlock/robosherlock/CMakeLists.txt;194;catkin_add_gtest;/home/hassouna/perception/rs_ws/src/robosherlock/robosherlock/CMakeLists.txt;0;")
subdirs("gtest")
subdirs("src/core")
subdirs("src/flowcontrol")
subdirs("src/io")
subdirs("src/queryanswering")
subdirs("src/segmentation")
subdirs("src/utils")
