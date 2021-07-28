cmake_minimum_required(VERSION 3.0.2)
project(serial_port)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        serial
        sensor_msgs
        message_generation

)

add_message_files(FILES RangeInformation.msg Range.msg BDSephem.msg Gprmc.msg)

generate_messages(DEPENDENCIES std_msgs serial_port )


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_port
#  CATKIN_DEPENDS roscpp seiral
#  DEPENDS system_lib
)

add_executable(serial_port src/serial_port.cpp ) 
add_dependencies(serial_port ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_port ${catkin_LIBRARIES})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
