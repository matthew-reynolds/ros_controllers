cmake_minimum_required(VERSION 2.8.3)
project(ackermann_steering_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  diff_drive_controller
  hardware_interface
  nav_msgs
  pluginlib
  realtime_tools
  roscpp
  tf
  urdf
)

# Include header-only Boost libraries (Boost.Accumulators, Boost.Bind, and Boost.Function)
find_package(Boost REQUIRED)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    diff_drive_controller
    hardware_interface
    nav_msgs
    realtime_tools
    roscpp
    tf
  DEPENDS
    Boost
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/ackermann_steering_controller.cpp src/odometry.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  find_package(controller_manager REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  include_directories(
    ${controller_manager_INCLUDE_DIRS}
    ${controller_manager_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
  )

  add_executable(${PROJECT_NAME}_ackermann_steering_bot EXCLUDE_FROM_ALL test/common/src/ackermann_steering_bot.cpp)
  add_dependencies(${PROJECT_NAME}_ackermann_steering_bot ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_ackermann_steering_bot
    ${catkin_LIBRARIES}
    ${controller_manager_LIBRARIES}
  )
  add_dependencies(tests ${PROJECT_NAME}_ackermann_steering_bot)

  add_rostest_gtest(${PROJECT_NAME}_test
    test/ackermann_steering_controller_test/ackermann_steering_controller.test
    test/ackermann_steering_controller_test/ackermann_steering_controller_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}_nan_test
    test/ackermann_steering_controller_nan_test/ackermann_steering_controller_nan.test
    test/ackermann_steering_controller_nan_test/ackermann_steering_controller_nan_test.cpp)
  target_link_libraries(${PROJECT_NAME}_nan_test ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}_limits_test
    test/ackermann_steering_controller_limits_test/ackermann_steering_controller_limits.test
    test/ackermann_steering_controller_limits_test/ackermann_steering_controller_limits_test.cpp)
  target_link_libraries(${PROJECT_NAME}_limits_test ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}_timeout_test
    test/ackermann_steering_controller_timeout_test/ackermann_steering_controller_timeout.test
    test/ackermann_steering_controller_timeout_test/ackermann_steering_controller_timeout_test.cpp)
  target_link_libraries(${PROJECT_NAME}_timeout_test ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_steering_controller_fail_test
    test/ackermann_steering_controller_fail_test/ackermann_steering_controller_wrong.test
    test/ackermann_steering_controller_fail_test/ackermann_steering_controller_fail_test.cpp)
  target_link_libraries(ackermann_steering_controller_fail_test ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}_odom_tf_test
    test/ackermann_steering_controller_odom_tf_test/ackermann_steering_controller_odom_tf.test
    test/ackermann_steering_controller_odom_tf_test/ackermann_steering_controller_odom_tf_test.cpp)
  target_link_libraries(${PROJECT_NAME}_odom_tf_test ${catkin_LIBRARIES})

  add_rostest_gtest(${PROJECT_NAME}_default_odom_frame_test
    test/ackermann_steering_controller_default_odom_frame_test/ackermann_steering_controller_default_odom_frame.test
    test/ackermann_steering_controller_default_odom_frame_test/ackermann_steering_controller_default_odom_frame_test.cpp)
  target_link_libraries(${PROJECT_NAME}_default_odom_frame_test ${catkin_LIBRARIES})

  add_rostest_gtest(ackermann_steering_controller_odom_frame_test
    test/ackermann_steering_controller_odom_frame_test/ackermann_steering_controller_odom_frame.test
    test/ackermann_steering_controller_odom_frame_test/ackermann_steering_controller_odom_frame_test.cpp)
  target_link_libraries(ackermann_steering_controller_odom_frame_test ${catkin_LIBRARIES})

  add_rostest(test/ackermann_steering_controller_open_loop_test/ackermann_steering_controller_open_loop.test)
  add_rostest(test/ackermann_steering_controller_no_wheel_test/ackermann_steering_controller_no_wheel.test)
  add_rostest(test/ackermann_steering_controller_no_steer_test/ackermann_steering_controller_no_steer.test)
  add_rostest(test/ackermann_steering_controller_radius_param_test/ackermann_steering_controller_radius_param.test)
  add_rostest(test/ackermann_steering_controller_radius_param_fail_test/ackermann_steering_controller_radius_param_fail.test)
  add_rostest(test/ackermann_steering_controller_separation_param_test/ackermann_steering_controller_separation_param.test)
endif()


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES ackermann_steering_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
