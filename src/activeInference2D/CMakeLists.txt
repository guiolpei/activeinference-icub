cmake_minimum_required(VERSION 3.5)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

PROJECT(activeInference2D)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB)
# add include directories
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS}) #deprecated, handled automatically by CMake targets.
ADD_EXECUTABLE(${PROJECT_NAME} main.cpp ../FreeEnergyOptimization.cpp ../ICubControl.cpp ../ICubKinematics.cpp ../ICubImage.cpp ../YarpPeriodic.cpp ../ICubTouch.cpp FirstDerivative.cpp SecondDerivative.cpp Attractor.cpp Action.cpp)
# include directories
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE /usr/include/eigen3/)
# we now add the YARP libraries to our project.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${YARP_LIBRARIES} iKin ${OpenCV_LIBS})
