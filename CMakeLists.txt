# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(nano_quad)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs genmsg geometry_msgs dynamic_reconfigure)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)
generate_dynamic_reconfigure_options(cfg/default_bebop.cfg)

## Declare a catkin package
catkin_package()

## Build controller and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake) 

add_executable(controller src/controller.cpp)

target_link_libraries(controller ${catkin_LIBRARIES})

add_dependencies(controller beginner_tutorials_generate_messages_cpp)
add_dependencies(controller ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TRAGETS})


# %EndTag(FULLTEXT)%

