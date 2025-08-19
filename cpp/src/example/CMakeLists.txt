cmake_minimum_required(VERSION 3.8)
project(example)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(example src/node.cpp)

install(TARGETS example RUNTIME DESTINATION bin)
