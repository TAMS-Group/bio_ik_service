project(bio_ik_service)

cmake_minimum_required(VERSION 3.5)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  tf
  tf_conversions
  bio_ik
  bio_ik_msgs
  eigen_conversions
)

catkin_package(CATKIN_DEPENDS
  roscpp
  moveit_core
  tf
  tf_conversions
  bio_ik
  bio_ik_msgs
  eigen_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bio_ik_service src/bio_ik_service.cpp)
add_dependencies(bio_ik_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bio_ik_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
