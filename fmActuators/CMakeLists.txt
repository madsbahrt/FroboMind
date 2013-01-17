cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()

#rosbuild_add_executable(roboteq src/Motor_Controllers/RoboteQ/rtq_node.cpp)
rosbuild_add_executable(roboteq_controller src/Motor_Controllers/RoboteQ/roboteq.cpp)
rosbuild_add_executable(hbl2350_controller src/Motor_Controllers/RoboteQ/hbl2350.cpp)
rosbuild_link_boost(hbl2350_controller system thread)

rosbuild_add_executable(frobit_vel_to_nmea src/Motor_Controllers/Frobit/frobit_motors.cpp)

IF(EXISTS /usr/lib/libJAIFactory.so)
    rosbuild_add_executable(sprayer src/IO_Actuators/JAI_Camera/sdu_sprayer.cpp src/IO_Actuators/JAI_Camera/jai_ad-080ge.cc)
    target_link_libraries(sprayer JAIFactory) 
ENDIF(EXISTS /usr/lib/libJAIFactory.so)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()

#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
