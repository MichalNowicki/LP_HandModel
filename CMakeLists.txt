# cmake requirements
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

# Estimator of Hand Configuration
PROJECT(HandEstimator)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")

# Math
file(GLOB MATH_SOURCES
    "${CMAKE_SOURCE_DIR}/src/Core/Math/*.cpp"
)
file(GLOB MATH_HEADERS
    "${CMAKE_SOURCE_DIR}/include/Core/Math/*.h"
)

ADD_LIBRARY(Math STATIC ${MATH_SOURCES} ${MATH_HEADERS})
#TARGET_LINK_LIBRARIES(Math)
INSTALL(TARGETS Math RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${MATH_HEADERS} DESTINATION include/handest/Core/Math/)

# Grabber interface
SET(GRABBER_HEADERS ./include/Grabber/grabber.h)

# Kinect Grabber
SET(GRABBERKINECT_SOURCES ./src/Grabber/kinect_grabber.cpp)
SET(GRABBERKINECT_HEADERS ./include/Grabber/kinect_grabber.h)
ADD_LIBRARY(GrabberKinect STATIC ${GRABBERKINECT_SOURCES} ${GRABBER_HEADERS} ${GRABBERKINECT_HEADERS})
#TARGET_LINK_LIBRARIES(GrabberKinect)
INSTALL(TARGETS GrabberKinect RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${GRABBER_HEADERS} DESTINATION include/handest/Grabber/)

# Forward Kinematics
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#find_package(Eigen3 REQUIRED)
SET(FK_SOURCES ./src/Kinematic/kinematicLIE.cpp)
SET(FK_HEADERS ./include/Kinematic/kinematic.h )
SET(EIGEN3_INCLUDE_DIR ./dependencies/*)
include_directories("${EIGEN3_INCLUDE_DIR}")
ADD_LIBRARY(ForwardKinematics STATIC ${FK_SOURCES} ${FK_HEADERS})
INSTALL(TARGETS ForwardKinematics RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)

# Demo
SET(DEMO_SOURCES ./main.cpp)
ADD_EXECUTABLE(Demo ${DEMO_SOURCES})
TARGET_LINK_LIBRARIES(Demo Math GrabberKinect ForwardKinematics)
INSTALL(TARGETS Demo RUNTIME DESTINATION bin)
