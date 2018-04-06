#---------------------------------------------------------------------------------------------------------------------
# @file       CMakeLists.txt
#
# @brief      Core CMake definitions.
#
# @author     Sebastian Krusche
# @author     Fraunhofer IWU
#
# @version    0.1
# @date       17-02-2016
#--------------------------------------------------------------------------------------------------------------------- 

# required version
#cmake_minimum_required(VERSION 3.0.0)
PROJECT(test_openni2)
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE off)

add_definitions(-DBOOST_ALL_NO_LIB)




SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORT_SHARED_LIBS TRUE)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(CMAKE_CONFIGURATION_TYPES Release CACHE TYPE INTERNAL FORCE)
SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)
SET(OpenCV_DIR "D:/depend/opencv-3.4.0/build/")
Find_package(OpenCV REQUIRED)





# this is the project
project(libfrenect2 CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Qpar")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GL")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /LTCG")
set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} /LTCG")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LTCG /DEBUG")
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /LTCG")

# additional cmake modules
LIST(APPEND CMAKE_MODULE_PATH ${MY_DIR}/cmake_modules)

######################
## Set Include Dirs ##
######################



include_directories(
	include
	src
	${CMAKE_CURRENT_SOURCE_DIR}
	${OpenCV_INCLUDE_DIR}

)

link_directories("lib/Release")
link_directories("lib/Debug")

############################
## Build Lib Kinect Calib ##
############################

add_library(Kinect2Calib STATIC
	src/Kinect2Calib/kinect2_calib.cpp
	src/Kinect2Calib/kinect2_calib.h
	)
target_link_libraries(Kinect2Calib ${OpenCV_LIBRARIES}
)


###########################
## Build Lib World Frame ##
###########################

add_library(WorldFrame STATIC
	src/WorldFrame/WorldFrame.cpp
	src/WorldFrame/WorldFrame.h 
)
target_link_libraries(WorldFrame ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES})


########################
## Build Multi Sensor ##
########################

ADD_EXECUTABLE(kinect_v2
  src/kinect_v2.cpp
  src/WorldFrame.h
  src/WorldFrame.cpp
  src/VideoProcessor.h
  src/VideoProcessor.cpp
)
message("${OpenCV_LIBRARIES}")

TARGET_LINK_LIBRARIES(kinect_v2
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
  Kinect2Calib
  WorldFrame
  freenect2
)




