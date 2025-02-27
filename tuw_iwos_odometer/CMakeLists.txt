cmake_minimum_required(VERSION 3.0.2)
project(tuw_iwos_odometer)

add_compile_options(-std=c++14)

find_package(
  catkin
  REQUIRED
  roscpp
  rospy
  roslib
  roslint
  dynamic_reconfigure
  tf2
  tuw_geometry
  sensor_msgs
  nav_msgs
  message_filters
  tuw_iwos_tools
  tuw_nav_msgs
)

add_service_files(
  FILES
  OdometerMotorService.srv
  OdometerSensorService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  config/odometer_node.cfg
  config/odometer_service_node.cfg
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  tuw_iwos_tools
)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/odometer.cpp
  src/${PROJECT_NAME}/odometer_motor.cpp
  src/${PROJECT_NAME}/odometer_sensor.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)

  catkin_add_gtest(
    ${PROJECT_NAME}_tests
    test/all_tests.cpp
    test/${PROJECT_NAME}/motor_odometer_test.cpp
  )

  target_link_libraries(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    gmock
  )

  # Create a target ${PROJECT_NAME}_coverage_report
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/${PROJECT_NAME}/other_dir_i_dont_care_about*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES tests
    )
  endif()

endif()

add_executable(
  ${PROJECT_NAME}_node
  src/odometer_node.cpp
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  ${PROJECT_NAME}_service_node
  src/odometer_service_node.cpp
)

add_dependencies(
  ${PROJECT_NAME}_service_node
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(
  ${PROJECT_NAME}_service_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)