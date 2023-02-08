cmake_minimum_required(VERSION 2.8.3)
project(ros_mv119)

## this package is tested on mv_1.1.9


set(CXX "/x86_64-oesdk-linux/usr/bin/arm-oemllib32-linux-gnueabi-g++")

set(SYSROOT "/")



## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
)

catkin_package()
###########
## Build ##
###########
message( "MV_SDK: MV_1.1.9"  )

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
    src/mvVISLAM
    src/mvGateway/platform/
   ${catkin_INCLUDE_DIRS}
   /usr/include
   /usr/lib/hw
   $ENV{MV_SDK}/usr/include/mv)


add_definitions(-std=c++11) 

## Declare a C++ library



# Make sure we include lib dir for Linux build
add_library(mv1 SHARED IMPORTED)
set_property(TARGET mv1 PROPERTY IMPORTED_LOCATION /usr/lib/libmv1.so)



## Declare a C++ executable
add_executable(mvVISLAM 
src/mvVISLAM/main.cpp 
src/mvGateway/platform/EagleCamera.cpp
src/mvGateway/platform/EagleImu.cpp)




## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(mvVISLAM ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories( mvVISLAM PRIVATE 
  src/mvGateway/platform/
  src/mvVISLAM/
  /usr/include/
  $ENV{MV_SDK}/usr/include/mv )

## Specify libraries to link a library or executable target against
 target_link_libraries(mvVISLAM
   mv1 
   camera
   sensor_imu
   pthread
   rt
   m
   ssl
   crypto
   dl
   z
   OpenCL
   ${catkin_LIBRARIES}
)




set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -Wl,-rpath-link,${CMAKE_SYSROOT}/linaro-rootfs/lib/arm-linux-gnueabihf -Wl,-rpath-link,${CMAKE_SYSROOT}/linaro-rootfs/usr/lib/arm-linux-gnueabihf")

#############
## Install ##
#############

