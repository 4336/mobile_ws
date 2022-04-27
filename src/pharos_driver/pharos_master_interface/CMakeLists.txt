cmake_minimum_required(VERSION 2.8.3)
project(pharos_master_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  pharos_msgs
#  message_generation
)

generate_messages(
        DEPENDENCIES
        std_msgs

#        pharos_msgs
)

# catkin_package(
#   CATKIN_DEPENDS
#     serial
#     std_msgs
#     pharos_msgs
# #    geometry_msgs
# )


#add_message_files(
#
#
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(master_driver_node src/master_driver.cpp)

target_link_libraries(master_driver_node ${catkin_LIBRARIES})

add_dependencies(master_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
