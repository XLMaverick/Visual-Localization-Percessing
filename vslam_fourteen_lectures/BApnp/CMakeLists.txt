CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(BUILD_TYPE Release)

PROJECT(pngBA)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(Sophus_DIR /home/yxl/workspace/gaoxiangSlambook/slambook/3rdparty/Sophus/build/)
FIND_PACKAGE(Sophus REQUIRED)

INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS}
                    "/usr/include/eigen3")

MESSAGE(STATUS "Sophus INCLUD: ${Sophus_INCLUDE_DIRS} \n ${Sophus_LIBRARIES}")


ADD_EXECUTABLE(${PROJECT_NAME} GN-BA.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
                      ${Sophus_LIBRARIES})