cmake_minimum_required(VERSION 3.5)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

PROJECT(activeInference3D)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECT_NAME} main.cpp ../FreeEnergyOptimization.cpp ../ICubControl.cpp ../ICubKinematics.cpp ../ICubImage.cpp ../YarpPeriodic.cpp ../ICubTouch.cpp FirstDerivative.cpp SecondDerivative.cpp Attractor.cpp Action.cpp)

TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE /usr/include/eigen3/)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${YARP_LIBRARIES} iKin ${OpenCV_LIBS})
