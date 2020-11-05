# CMakeList.txt: freedomdrone 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_STANDARD 11)

project (freedomdrone)

include_directories(./algorithm)
include_directories(./connection)
include_directories(./messaging)
include_directories(./logging)
include_directories(./lib)
include_directories(.)

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