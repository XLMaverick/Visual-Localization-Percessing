
# CMake 最低版本号要求
cmake_minimum_required(VERSION 3.0)  

# 项目名称
PROJECT(HarrisDetector)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}") 

SET(SRC_LIST main.cpp)  


#添加opencv头文件的搜索路径 根据自己的具体目录设置
set(OpenCV_DIR /home/yxl/opencv/build)
find_package(OpenCV 3.2 REQUIRED)  
include_directories(${OpenCV_INCLUDE_DIRS} /usr/local/include/opencv)  

include_directories(.)
include_directories(${CMAKE_BINARY_DIR})


# 指定生成目标 告诉工程生成一个可执行文件。
ADD_EXECUTABLE(HarrisDetector ${SRC_LIST})  


#add_library 则告诉生成一个库文件。
#注意：CMakeList.txt 文件中，命令名字是不区分大小写的，而参数和变量是大小写相关的。




# 添加链接库
target_link_libraries(HarrisDetector ${OpenCV_LIBS})
