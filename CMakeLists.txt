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
std_msgs
message_generation
)

SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")

FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(osqp REQUIRED)
#find_package(OsqpEigen REQUIRED)


#link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_BUILD_TYPE Release)

add_message_files(
  FILES
  QP.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS
controller_interface
hardware_interface
message_runtime
pluginlib
roscpp
std_msgs
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



add_library(floating_lib src/floating_base_controller.cpp)

target_link_libraries(floating_lib ${catkin_LIBRARIES}
                                     ${RBDL_LIBRARY}
                                  ${RBDL_URDFReader_LIBRARY})
   #                               OsqpEigen::OsqpEigen)
                                  
   

add_executable(set_command src/set_command_FB.cpp)
target_link_libraries(set_command ${catkin_LIBRARIES})

add_executable(state_estimator src/state_estimator.cpp)
target_link_libraries(state_estimator ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS src/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#add_executable(anymal_c_simple_description src/floating_base_controller.cpp)
#target_link_libraries(anymal_c_simple_description 
 #                                                 ${catkin_LIBRARIES}
  #                                               ${RBDL_LIBRARY}
   #                                               ${RBDL_URDFReader_LIBRARY})
                                                   #OsqpEigen::OsqpEigen)

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
