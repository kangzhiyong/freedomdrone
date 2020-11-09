# CMakeList.txt: freedomdrone 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE "Release")

project (freedomdrone)

include_directories(./algorithm)
include_directories(./connection)
include_directories(./messaging)
include_directories(./logging)
include_directories(./lib)
include_directories(./lib/matplotlib-cpp)
include_directories(.)

include_directories(${PYTHONHOME}/include)
include_directories(${PYTHONHOME}/Lib/site-packages/numpy/core/include)
link_directories(${PYTHONHOME}/libs)

add_definitions(-DMATPLOTLIBCPP_PYTHON_HEADER=Python.h)

FILE(GLOB SOURCES
        connection/*.cpp
        messaging/*.cpp
        algorithm/*.cpp
        ./*.cpp)

FILE(GLOB HEADERS
        connection/*.hpp
        messaging/*.hpp
        algorithm/*.hpp
        *.hpp
        *.h)

add_executable (freedomdrone 
        ${SOURCES} 
        ${HEADERS})

target_link_libraries(freedomdrone
        )