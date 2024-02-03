cmake_minimum_required(VERSION 3.22)

# set the project name
project(math VERSION 1.0)

# TODO: Provide a C++ interface for the shapes functionality.
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE "${PROJECT_SOURCE_DIR}/include")