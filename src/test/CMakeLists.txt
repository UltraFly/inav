cmake_minimum_required(VERSION 3.10)

project(INAVTests)

set(MAIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../main")
set(MAIN_SRC_DIR "${MAIN_DIR}")
set(MAIN_UTILS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../utils")
set(MAIN_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib")

include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/main.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/settings.cmake")

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/gtest.cmake)

enable_testing()
include(GoogleTest)
add_subdirectory(unit)
