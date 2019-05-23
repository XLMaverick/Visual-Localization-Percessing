CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(gauessnewton)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS}  
                     "/usr/include/eigen3/")
ADD_EXECUTABLE(${PROJECT_NAME} gaussnewton.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
