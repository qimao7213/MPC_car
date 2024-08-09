cmake_minimum_required(VERSION 3.10)

# 项目名称和版本
project(TCPCommunication VERSION 1.0)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加源文件
set(SOURCES
    src/tcp_communication.cpp
)

# 添加头文件目录
include_directories(include())

# 生成可执行文件 server 和 client
add_executable(mpc_server src/server_main.cpp ${SOURCES})
add_executable(mpc_client src/client_main.cpp ${SOURCES})

