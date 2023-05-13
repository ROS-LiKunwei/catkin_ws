# CMake generated Testfile for 
# Source directory: /home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo
# Build directory: /home/lkw/catkin_ws/build/moveit_servo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_moveit_servo_rostest_test_basic_servo_tests.test "/home/lkw/catkin_ws/build/moveit_servo/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/lkw/catkin_ws/build/moveit_servo/test_results/moveit_servo/rostest-test_basic_servo_tests.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo --package=moveit_servo --results-filename test_basic_servo_tests.xml --results-base-dir \"/home/lkw/catkin_ws/build/moveit_servo/test_results\" /home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo/test/basic_servo_tests.test ")
set_tests_properties(_ctest_moveit_servo_rostest_test_basic_servo_tests.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;80;add_rostest;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;100;_add_rostest_google_test;/home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo/CMakeLists.txt;169;add_rostest_gtest;/home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo/CMakeLists.txt;0;")
add_test(_ctest_moveit_servo_rostest_test_servo_cpp_interface_test.test "/home/lkw/catkin_ws/build/moveit_servo/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/lkw/catkin_ws/build/moveit_servo/test_results/moveit_servo/rostest-test_servo_cpp_interface_test.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo --package=moveit_servo --results-filename test_servo_cpp_interface_test.xml --results-base-dir \"/home/lkw/catkin_ws/build/moveit_servo/test_results\" /home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo/test/servo_cpp_interface_test.test ")
set_tests_properties(_ctest_moveit_servo_rostest_test_servo_cpp_interface_test.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;80;add_rostest;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;100;_add_rostest_google_test;/home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo/CMakeLists.txt;179;add_rostest_gtest;/home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo/CMakeLists.txt;0;")
add_test(_ctest_moveit_servo_rostest_test_pose_tracking_test.test "/home/lkw/catkin_ws/build/moveit_servo/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/lkw/catkin_ws/build/moveit_servo/test_results/moveit_servo/rostest-test_pose_tracking_test.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo --package=moveit_servo --results-filename test_pose_tracking_test.xml --results-base-dir \"/home/lkw/catkin_ws/build/moveit_servo/test_results\" /home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo/test/pose_tracking_test.test ")
set_tests_properties(_ctest_moveit_servo_rostest_test_pose_tracking_test.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;80;add_rostest;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;100;_add_rostest_google_test;/home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo/CMakeLists.txt;189;add_rostest_gtest;/home/lkw/catkin_ws/src/moveit/moveit_ros/moveit_servo/CMakeLists.txt;0;")
subdirs("gtest")
