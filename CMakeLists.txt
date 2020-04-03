cmake_minimum_required(VERSION 3.0.2)
project(mtc_pour)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz_marker_tools
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mtc_pour_into
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include(CheckIncludeFileCXX)
set(CMAKE_REQUIRED_INCLUDES ${moveit_core_INCLUDE_DIRS})
set(CMAKE_REQUIRED_FLAGS "-std=c++11")
check_include_file_cxx(moveit/robot_state/cartesian_interpolator.h MOVEIT_CARTESIAN_INTERPOLATOR)
if(NOT MOVEIT_CARTESIAN_INTERPOLATOR)
    set(MOVEIT_CARTESIAN_INTERPOLATOR 0)
endif()

add_library(mtc_pour_into src/pour_into.cpp)
add_dependencies(mtc_pour_into ${catkin_EXPORTED_TARGETS})
target_link_libraries(mtc_pour_into ${catkin_LIBRARIES})
target_compile_definitions(mtc_pour_into PRIVATE -DMOVEIT_CARTESIAN_INTERPOLATOR=${MOVEIT_CARTESIAN_INTERPOLATOR})

add_executable(${PROJECT_NAME}_tams_ur5_demo src/mtc_pour_tams_ur5_demo.cpp)
add_dependencies(${PROJECT_NAME}_tams_ur5_demo ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_tams_ur5_demo
	mtc_pour_into
	${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_baxter_demo src/mtc_pour_baxter_demo.cpp)
add_dependencies(${PROJECT_NAME}_baxter_demo ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_baxter_demo
	mtc_pour_into
	${catkin_LIBRARIES}
)
