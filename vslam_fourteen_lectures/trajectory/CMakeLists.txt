CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(draw_trajectory)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

ADD_EXECUTABLE(${PROJECT_NAME} draw_trajectory.cpp)

#FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)

INCLUDE_DIRECTORIES(
#${EIGEN3_INCLUDE_DIR}
"/usr/include/eigen3"
${Pangolin_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS})

message(STATUS "Pangolin include dirs: ${Pangolin_INCLUDE_DIRS}\n Sophus include : ${Sophus_INCLUDE_DIRS}")

TARGET_LINK_LIBRARIES(
${PROJECT_NAME}
${Sophus_LIBRARIES}
#${EIGEN3_LIBS}
${Pangolin_LIBRARIES})

