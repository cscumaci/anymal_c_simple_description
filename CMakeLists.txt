cmake_minimum_required(VERSION 3.5.1)
project(anymal_c_simple_description)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

find_package(catkin REQUIRED COMPONENTS
controller_interface
hardware_interface
pluginlib
roscpp
rospy
cmake_modules
gazebo_ros
)

SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")

FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)


#link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_BUILD_TYPE Release)

catkin_package(
CATKIN_DEPENDS
controller_interface
hardware_interface
message_runtime
pluginlib
roscpp
gazebo_ros
DEPENDS Eigen
)

include_directories(include
#  ${Boost_INCLUDE_DIR} 
#  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)


    

add_executable(set_command src/set_command_FB.cpp)
target_link_libraries(set_command ${catkin_LIBRARIES})

add_executable(state_estimator src/state_estimator.cpp)
target_link_libraries(state_estimator ${catkin_LIBRARIES})

add_executable(anymal_c_simple_description src/floating_base_controller.cpp)
target_link_libraries(anymal_c_simple_description 
                                                  ${catkin_LIBRARIES}
                                                  ${RBDL_LIBRARY}
                                                  ${RBDL_URDFReader_LIBRARY}
                                                  OsqpEigen::OsqpEigen)
                                                  #rbdl_urdfreader)

install(DIRECTORY config launch meshes urdf models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      roslaunch
  )
  roslaunch_add_file_check(launch/load.launch)
endif()
