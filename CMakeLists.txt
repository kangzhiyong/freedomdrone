# CMakeList.txt: freedomdrone 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

project ("freedomdrone")

include_directories(./algorithm)
include_directories(./connection)
include_directories(./messaging)
include_directories(./logging)
include_directories(./lib)
include_directories(.)

set(CONNECTION_FILE "./connection/mavlink_connection.cpp" "./connection/mavlink_connection.hpp" 
                    "./connection/message_types.cpp" "./connection/message_types.hpp"
                    "./connection/mavlink_socket.cpp" "./connection/mavlink_socket.hpp"
                    "./connection/mavlink_utils.hpp" )
set(MESSAGEING_FILE "./messaging/message_ids.hpp")
set(LOGGING_FILE "")
set(ALGORITHM_FILE "./algorithm/free_point.cpp" "./algorithm/free_point.hpp")

# 查找指定目录下的所有.cpp与.hpp文件 并存放到指定变量名SC_FILES中
FILE(GLOB SC_FILES "*.cpp" "*.hpp")

# 将源代码添加到此项目的可执行文件。
add_executable (freedomdrone ${SC_FILES} ${CONNECTION_FILE} ${MESSAGEING_FILE} ${LOGGING_FILE} ${ALGORITHM_FILE})

# TODO: 如有需要，请添加测试并安装目标。
