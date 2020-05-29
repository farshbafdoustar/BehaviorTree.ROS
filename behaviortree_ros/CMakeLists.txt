cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty
project(behaviortree_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(ROS_DEPENDENCIES
    roscpp std_msgs
    behaviortree_cpp_v3
    behaviortree_ros_msgs
    actionlib_msgs
    actionlib
    message_generation)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )
find_package(GTest)

#add_message_files(
#  FILES
#)

add_service_files(
   FILES
   AddTwoInts.srv
 )

add_action_files(
   DIRECTORY action
   FILES     Fibonacci.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES behaviortree_ros
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS})

######################################################
# LIBRARIES

#header only for the time being

add_library(behaviortree_ros
    src/loggers/rosout_logger.cpp
    src/loggers/topic_publisher_logger.cpp
 )
add_dependencies(behaviortree_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(behaviortree_ros ${catkin_LIBRARIES})

######################################################
# TESTS

#add_executable(test_bt test/test_bt.cpp)
#add_dependencies(test_bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_bt ${catkin_LIBRARIES} )

add_executable(test_server test/test_server.cpp)
add_dependencies(test_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_server ${catkin_LIBRARIES} )

######################################################
# INSTALL

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

######################################################
# EXAMPLES and TOOLS
