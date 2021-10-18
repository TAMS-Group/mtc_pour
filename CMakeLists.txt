cmake_minimum_required(VERSION 3.0.2)
project(mtc_pour)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz_marker_tools
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  tf2_eigen
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES mtc_pour_into
   CATKIN_DEPENDS
      roscpp
      rviz_marker_tools
      moveit_core
      moveit_ros_planning_interface
      moveit_task_constructor_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mtc_pour_into src/pour_into.cpp)
add_dependencies(mtc_pour_into ${catkin_EXPORTED_TARGETS})
target_link_libraries(mtc_pour_into ${catkin_LIBRARIES})

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
