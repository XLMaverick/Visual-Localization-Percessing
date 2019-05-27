CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(BUILD_TYPE Release)

PROJECT(direct_method)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")


set(Sophus_DIR /home/yxl/workspace/gaoxiangSlambook/slambook/3rdparty/Sophus/build/)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)

INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    "/usr/include/eigen3")

MESSAGE(STATUS "OpenCV INCLUD: ${OpenCV_INCLUDE_DIRS} \n ${OpenCV_LIBRARIES}")
MESSAGE(STATUS "Sophus INCLUD: ${Sophus_INCLUDE_DIRS} \n ${Sophus_LIBRARIES}")
message(STATUS "Pangolin include dirs: ${Pangolin_INCLUDE_DIRS}\n Sophus include : ${Sophus_INCLUDE_DIRS}")



ADD_EXECUTABLE(${PROJECT_NAME} direct_method.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
                      ${OpenCV_LIBRARIES}
                      ${Sophus_LIBRARIES}
                      ${Pangolin_LIBRARIES})
   
                      
                      
                      
                      
