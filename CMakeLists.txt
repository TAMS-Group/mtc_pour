cmake_minimum_required(VERSION 2.8.3)
project(mtc_pour)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  moveit_task_constructor_core
  roscpp
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mtc_pour_into
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mtc_pour_into src/pour_into.cpp)
add_dependencies(mtc_pour_into ${catkin_EXPORTED_TARGETS})
target_link_libraries(mtc_pour_into ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_demo src/tams_ur5_pouring_mtc_demo.cpp)
add_dependencies(${PROJECT_NAME}_demo ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_demo
	mtc_pour_into
	${catkin_LIBRARIES}
)
