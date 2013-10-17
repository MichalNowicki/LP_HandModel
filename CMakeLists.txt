# cmake requirements
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

# Estimator of Hand Configuration
PROJECT(HandEstimator)

# Settings
INCLUDE_DIRECTORIES(./include)

# Grabber interface
SET(GRABBER_HEADERS ./include/Grabber/grabber.h)

# Kinect Grabber
SET(GRABBERKINECT_SOURCES ./src/Grabber/kinect_grabber.cpp)
SET(GRABBERKINECT_HEADERS ./include/Grabber/kinect_grabber.h)
ADD_LIBRARY(GrabberKinect STATIC ${GRABBERKINECT_SOURCES} ${GRABBER_HEADERS} ${GRABBERKINECT_HEADERS})
#TARGET_LINK_LIBRARIES(GrabberKinect)
INSTALL(TARGETS GrabberKinect RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${GRABBER_HEADERS} DESTINATION include/handest/Grabber/)

# Demo
SET(DEMO_SOURCES ./main.cpp)
ADD_EXECUTABLE(Demo ${DEMO_SOURCES})
TARGET_LINK_LIBRARIES(Demo GrabberKinect)
INSTALL(TARGETS Demo RUNTIME DESTINATION bin)
