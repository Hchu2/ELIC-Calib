cmake_minimum_required(VERSION 3.10)

project(logger VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_subdirectory(${CMAKE_SOURCE_DIR}/src)

set(CMAKE_INSTALL_PREFIX /usr/local)

install(FILES ${CMAKE_SOURCE_DIR}/src/include/logger.h DESTINATION include/artwork/logger)