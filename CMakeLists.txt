# CMakeList.txt: freedomdrone 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

project (freedomdrone)

message(STATUS "platform: ${CMAKE_SYSTEM_NAME}")
if (UNIX AND NOT APPLE)
    message(STATUS "unix")
elseif (WIN32)
    message(STATUS "windows")
    set(PYTHON_INCLUDE_DIR "C:\\Users\\Administrator\\AppData\\Local\\Programs\\Python\\Python36\\include")
    set(PYTHON_NUMPY_INCLUDE_DIR "C:\\Users\\Administrator\\AppData\\Local\\Programs\\Python\\Python36\\lib\\site-packages\\numpy\\core\\include")
    set(PYTHON_LIBRARIES "C:\\Users\\Administrator\\AppData\\Local\\Programs\\Python\\Python36\\libs\\")
    set(QT_LIBRARIES "F:\\Qt\\5.15.2\\winrt_x64_msvc2019\\lib")
    set(PYTHON_LIBCFCLASS  Python3)
    set(MSGPACK_INCLUDE_DIR "D:\\alg\\git\\freedomdrone\\lib\\msgpack-c\\include")
elseif (APPLE)
    message(STATUS "mac os")
    set(PYTHON_INCLUDE_DIR "/Applications/anaconda3/envs/fcnd/include/python3.6m")
    set(PYTHON_NUMPY_INCLUDE_DIR "/Applications/anaconda3/envs/fcnd/lib/python3.6/site-packages/numpy/core/include")
    set(PYTHON_LIBRARIES "/Applications/anaconda3/envs/fcnd/lib")
    set(PYTHON_LIBCFCLASS  -lpython3.6m)

    set(CMAKE_PREFIX_PATH "/usr/local/Cellar/qt/5.14.1/lib/cmake/Qt5")
    set(MSGPACK_INCLUDE_DIR "/usr/local/include")
else ()
    MESSAGE(STATUS "other platform")
endif ()

find_package(Qt5 COMPONENTS Core Gui Qml Quick REQUIRED Positioning)

include_directories(${MSGPACK_INCLUDE_DIR})
include_directories(${PYTHON_INCLUDE_DIR})
include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
include_directories(./planning)
include_directories(./controls)
include_directories(./controls/crazyflie)
include_directories(./estimator)
include_directories(./estimator/ekf)
include_directories(./connection)
include_directories(./messaging)
include_directories(./logging)
include_directories(./drone)
include_directories(./drone/crazyflie)
include_directories(./math)
include_directories(./lib)
include_directories(./lib/matplotlib-cpp)
include_directories(.)

link_directories(${PYTHON_LIBRARIES})
link_directories(${QT_LIBRARIES})

FILE(GLOB SOURCES
        connection/*.cpp
        messaging/*.cpp
        planning/*.cpp
        controls/*.cpp
        controls/crazyflie/*.cpp
        estimator/*.cpp
        estimator/ekf/*.cpp
        drone/*.cpp
        drone/crazyflie/*.cpp
        math/*.cpp
        ./*.cpp)

FILE(GLOB HEADERS
        connection/*.hpp
        messaging/*.hpp
        planning/*.hpp
        controls/*.hpp
        controls/crazyflie/*.hpp
        estimator/*.hpp
        estimator/ekf/*.hpp
        drone/*.hpp
        drone/crazyflie/*.hpp
        math/*.hpp
        *.hpp
        *.h)
#set(PYTHONLIBS /Applications/anaconda3/envs/fcnd/lib/)

add_executable (freedomdrone 
        ${SOURCES} 
        ${HEADERS})

target_link_libraries(freedomdrone
        ${PYTHON_LIBCFCLASS}
        Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick Qt5::Positioning
        )
