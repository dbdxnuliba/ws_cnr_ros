cmake_minimum_required(VERSION 2.8.3)
project(robust_inverse_dynamics_controller)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O4")

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  robust_inverse_dynamics
  subscription_notifier
  roscpp
  sensor_msgs
  std_msgs
  name_sorting
  )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS   controller_interface
  hardware_interface
  robust_inverse_dynamics
  subscription_notifier
  roscpp
  sensor_msgs
  std_msgs
  name_sorting
  DEPENDS
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/robust_inverse_dynamics_controller.cpp
  )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
