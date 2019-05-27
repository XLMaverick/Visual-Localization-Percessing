CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(BUILD_TYPE Release)

PROJECT(icp)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

set(Sophus_DIR /home/yxl/workspace/gaoxiangSlambook/slambook/3rdparty/Sophus/build/)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(G2O REQUIRED)
FIND_PACKAGE(CSparse REQUIRED)

INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS}
                    "/usr/include/eigen3"
                    ${Pangolin_INCLUDE_DIRS}
                    ${G2O_INCLUDE_DIRS}
                    ${CSPARSE_INCLUDE_DIR}
                    )

MESSAGE(STATUS "Sophus INCLUD: ${Sophus_INCLUDE_DIRS} \n ${Sophus_LIBRARIES}")

MESSAGE(STATUS "Pangolin INCLUD: ${Pangolin_INCLUDE_DIRS} \n ${Pangolin_LIBRARIES}")

MESSAGE(STATUS "G2O INCLUD: ${G2O_INCLUDE_DIRS} ")


ADD_EXECUTABLE(${PROJECT_NAME} icp.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
                      ${Sophus_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension 
                      ${CSPARSE_LIBRARY}
                    )