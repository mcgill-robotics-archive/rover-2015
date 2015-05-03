cmake_minimum_required(VERSION 3.0)
project(RS485)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    Rs485Test.c)

add_executable(RS485 ${SOURCE_FILES} Rs485Test.h helper.h helper.c)
add_subdirectory(ifdefs)
add_executable(FunctionPointers FunctionPointers.c FunctionPointers.h)