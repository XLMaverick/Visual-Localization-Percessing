CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(disparity)

set(CMAKE_CXX_FLAGS "-std=c++11")

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} 
                    ${Pangolin_INCLUDE_DIRS}
                    "/usr/include/eigen3/")

ADD_EXECUTABLE(${PROJECT_NAME} disparity.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS}
                                      ${Pangolin_LIBRARIES})

