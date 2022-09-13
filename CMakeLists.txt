cmake_minimum_required(VERSION 3.15)

project(graph)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall")

add_executable(main main.cpp solution.cpp)
add_executable(checker checker.cpp)
add_executable(generator generator.cpp)

