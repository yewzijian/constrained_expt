cmake_minimum_required(VERSION 2.8.3)
project(constrained_expt)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(simple_demo src/simple_demo.cpp)
<<<<<<< HEAD
add_executable(simple_demo3 src/simple_demo3.cpp)
add_executable(arc_demo1 src/arc_demo1.cpp)
add_executable(arc_demo2 src/arc_demo2.cpp)
add_executable(arc_demo3 src/arc_demo3.cpp)
add_executable(arc_demo4 src/arc_demo4.cpp)
add_executable(arc_demo5 src/arc_demo5.cpp)


target_link_libraries(simple_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(simple_demo3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(arc_demo1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(arc_demo2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(arc_demo3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(arc_demo4 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(arc_demo5 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
=======
target_link_libraries(simple_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(generate_problems src/generate_problems.cpp)
target_link_libraries(generate_problems ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(run_eval src/run_eval.cpp)
target_link_libraries(run_eval ${catkin_LIBRARIES} ${Boost_LIBRARIES})
>>>>>>> b657db97d95f69387ecc4cab3f782fbf674a2017
